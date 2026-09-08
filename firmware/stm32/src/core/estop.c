/*
 * estop.c — Fungo E-STOP hardware su PC6 (JONNY5 SHIELD rev3, solo G474)
 *
 * Elettrica FAIL-SAFE: contatto NC verso GND, pull-up esterno R8 10k a 3V3,
 * filtro C10 100n. Linea ALTA = STOP ATTIVO (fungo premuto O filo staccato/
 * tagliato); bassa = marcia. Il LED blu della WeAct e' sulla stessa rete e
 * si accende da solo quando lo stop e' attivo: nessun codice LED.
 *
 * Fronte di ATTIVAZIONE (stessa via del comando UART STOP, senza parser):
 *   coppia via subito (servo_disable_all + abort SETPOSE + disarm VR),
 *   poi latch STATE_STOPPED + pickplace_safe_off, LOG_WRN; "ESTOP" verso il
 *   Pi via flag, emesso dal main loop (estop_notify_poll) per non bloccare
 *   il tick RT con uart_poll_out.
 * Fronte di RILASCIO: solo log + flag "ESTOP_CLEAR". NESSUNA auto-ripresa:
 *   recovery con la sequenza esistente SAFE -> ENABLE.
 * Init FALLITA = FAIL-SAFE: stato ATTIVO latchato + STOPPED mantenuto, il
 *   robot non funziona mai con il fungo silenziosamente morto.
 * Finche' ATTIVO lo stato viene ri-forzato a STOPPED a ogni tick: neutralizza
 * SAFE/ENABLE via UART e la auto-transizione SAFE->IDLE del RT loop.
 *
 * Guardia devicetree stile pickplace.c: senza alias "estop" (build F446)
 * compila lo stub no-op in fondo, zero cambi di comportamento.
 */

#include "core/estop.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(estop, LOG_LEVEL_INF);

#define ESTOP_NODE DT_ALIAS(estop)

#if DT_NODE_EXISTS(ESTOP_NODE)

#include "core/state_machine.h"
#include "core/rt_loop.h"
#include "servo/servo_control.h"
#include "servo/motion_planner.h"
#include "servo/j5vr_setpose.h"
#include "servo/pickplace.h"
#include "uart/uart_control.h"

#define ESTOP_GPIO_PORT  DT_GPIO_CTLR(ESTOP_NODE, gpios)
#define ESTOP_GPIO_PIN   DT_GPIO_PIN(ESTOP_NODE, gpios)
#define ESTOP_GPIO_FLAGS DT_GPIO_FLAGS(ESTOP_NODE, gpios)

static const struct device *const estop_dev = DEVICE_DT_GET(ESTOP_GPIO_PORT);
static bool estop_initialized = false;

/* Debounce a N campioni consecutivi @ 1 kHz = 10 ms totali. C10 filtra gia'
 * i glitch sub-ms: qui si copre solo il rimbalzo meccanico del fungo. */
#define ESTOP_DEBOUNCE_SAMPLES 10U
static uint8_t estop_debounce_counter = 0U;
/* volatile: letto anche dal thread UART (gate comandi) oltre che dal RT loop */
static volatile bool estop_debounced_state = false;

/* Notifiche ESTOP/ESTOP_CLEAR verso il Pi: i fronti (thread RT 1 kHz) settano
 * solo questi flag; l'emissione UART (uart_poll_out busy-wait, ~0.5-1 ms a
 * 115200) avviene nel main loop via estop_notify_poll(). Cosi' il tick RT non
 * subisce overrun e la riga non si interleava con una risposta in corso. */
static volatile bool estop_notify_engage_pending = false;
static volatile bool estop_notify_clear_pending  = false;

/* Sequenza di stop, identica al ramo UART STOP ma senza parser. Tutte chiamate
 * non-bloccanti (scritture PWM/stato): sicura nel tick 1 kHz.
 * servo_disable_all() qui e' idempotente rispetto alla barriera STOPPED del
 * RT loop, ma accorcia la latenza di coppia-via a questo stesso tick. */
static void estop_engage(void)
{
    servo_disable_all();        /* (a) coppia via IMMEDIATA: PWM=0 sui 6 servo */
    motion_planner_stop_all();  /*     stub legacy (no-op) */
    j5vr_setpose_abort();       /*     ABORT traiettoria SETPOSE: senza questo
                                 *     g_setpose_state.active resterebbe true e
                                 *     il vecchio target riprenderebbe da solo
                                 *     al primo SAFE post-rilascio (auto-ripresa
                                 *     vietata dalla specifica). */
    g_vr_armed = 0;             /*     disarm pipeline VR */
    state_machine_set_stopped();/* (b) latch STOPPED (via del comando UART STOP) */
    pickplace_safe_off();       /*     valvola + vacuum off, come ramo STOP */
}

/* Init fallita = dispositivo di sicurezza assente: FAIL-SAFE, non degradare a
 * "nessun E-STOP". Stato debounced forzato ATTIVO (mai piu' aggiornabile senza
 * GPIO), gate UART chiuso via estop_is_active(), latch STOPPED mantenuto da
 * estop_poll_tick(), notifica "ESTOP" al Pi via flag. */
static void estop_init_failed(void)
{
    estop_debounced_state = true;
    estop_notify_engage_pending = true;
}

void estop_init(void)
{
    if (!device_is_ready(estop_dev))
    {
        LOG_ERR("[ESTOP] GPIO device non pronto -- FAIL-SAFE: STOP latchato");
        estop_init_failed();
        return;
    }
    /* Input SENZA bias interno: il pull-up e' esterno (R8 10k a 3V3). */
    int ret = gpio_pin_configure(estop_dev, ESTOP_GPIO_PIN,
                                 GPIO_INPUT | ESTOP_GPIO_FLAGS);
    if (ret != 0)
    {
        LOG_ERR("[ESTOP] configure PC6 fallita (%d) -- FAIL-SAFE: STOP latchato",
                ret);
        estop_init_failed();
        return;
    }
    /* Pre-carica il debounce con lo stato reale al boot. Errore di lettura =
     * STOP attivo (fail-safe). Con GPIO_ACTIVE_HIGH: 1 = linea alta = STOP. */
    ret = gpio_pin_get(estop_dev, ESTOP_GPIO_PIN);
    estop_debounced_state = (ret != 0);
    estop_initialized = true;
    LOG_INF("[ESTOP] PC6 pronto, stato=%s",
            estop_debounced_state ? "ATTIVO" : "rilasciato");
    /* Se il fungo e' gia' premuto al boot il latch STOPPED scatta al primo
     * estop_poll_tick() (check "attivo && stato != STOPPED" sotto). */
}

void estop_poll_tick(void)
{
    if (!estop_initialized)
    {
        /* Init fallita: niente GPIO da campionare, ma se estop_init_failed()
         * ha latchato lo stato ATTIVO il latch STOPPED va comunque mantenuto
         * (fail-safe: robot fermo finche' l'hardware E-STOP non torna sano). */
        if (estop_debounced_state &&
            state_machine_get_state() != STATE_STOPPED)
        {
            estop_engage();
        }
        return;
    }

    int ret = gpio_pin_get(estop_dev, ESTOP_GPIO_PIN);
    /* Errore di lettura trattato come STOP attivo (fail-safe). */
    bool raw_active = (ret != 0);

    if (raw_active == estop_debounced_state)
    {
        estop_debounce_counter = 0U;
    }
    else
    {
        estop_debounce_counter++;
        if (estop_debounce_counter >= ESTOP_DEBOUNCE_SAMPLES)
        {
            estop_debounced_state = raw_active;
            estop_debounce_counter = 0U;
            if (raw_active)
            {
                /* Fronte di ATTIVAZIONE: coppia via SUBITO nel tick; la
                 * notifica "ESTOP" e' solo un flag, emesso dal main loop
                 * (estop_notify_poll): l'uart_poll_out busy-wait (~0.5 ms)
                 * causerebbe un overrun garantito del tick 1 kHz. */
                estop_engage();
                estop_notify_engage_pending = true;
                LOG_WRN("[ESTOP] fungo premuto/linea aperta -- STOPPED");
            }
            else
            {
                /* Fronte di RILASCIO: NESSUNA auto-ripresa. Si riparte con la
                 * sequenza esistente SAFE -> ENABLE dal Pi. */
                estop_notify_clear_pending = true;
                LOG_WRN("[ESTOP] rilasciato -- recovery: SAFE -> ENABLE");
            }
        }
    }

    /* Latch mentre ATTIVO: ri-forza STOPPED se qualcuno ha tentato SAFE/ENABLE
     * via UART o se la auto-transizione SAFE->IDLE e' scattata nel frattempo.
     * Copre anche il caso fungo gia' premuto al boot. */
    if (estop_debounced_state &&
        state_machine_get_state() != STATE_STOPPED)
    {
        estop_engage();
    }
}

bool estop_is_active(void)
{
    return estop_debounced_state;
}

void estop_notify_poll(void)
{
    /* Chiamata dal main loop (stesso thread di uart_control_process): nessun
     * interleaving con le risposte ai comandi, e il busy-wait di uart_poll_out
     * paga sul thread main, non nel tick RT. Se entrambi i flag sono pendenti
     * (pressione+rilascio dentro una finestra del main loop, ~10 ms) l'ordine
     * di emissione e' scelto in modo che l'ULTIMO token corrisponda allo stato
     * debounced corrente: il Pi resta coerente. Formato invariato:
     * "ESTOP\n" / "ESTOP_CLEAR\n" senza seq. */
    const bool eng = estop_notify_engage_pending;
    const bool clr = estop_notify_clear_pending;
    if (!eng && !clr)
    {
        return;
    }
    estop_notify_engage_pending = false;
    estop_notify_clear_pending  = false;

    if (eng && clr)
    {
        if (estop_debounced_state)
        {
            uart_send_unsolicited("ESTOP_CLEAR");
            uart_send_unsolicited("ESTOP");
        }
        else
        {
            uart_send_unsolicited("ESTOP");
            uart_send_unsolicited("ESTOP_CLEAR");
        }
    }
    else if (eng)
    {
        uart_send_unsolicited("ESTOP");
    }
    else
    {
        uart_send_unsolicited("ESTOP_CLEAR");
    }
}

#else /* !DT_NODE_EXISTS(ESTOP_NODE) — build senza E-STOP (es. F446) */

void estop_init(void)
{
    /* Nessun alias DT "estop" su questa board: stub attivo, nessun E-STOP HW. */
    LOG_INF("[ESTOP] assente su questa board: stub attivo");
}

void estop_poll_tick(void)
{
}

bool estop_is_active(void)
{
    return false;
}

void estop_notify_poll(void)
{
}

#endif /* DT_NODE_EXISTS(ESTOP_NODE) */
