/*
 * RT Loop - Implementation
 *
 * Loop realtime a 1 kHz tramite workqueue Zephyr.
 * Refresh watchdog, gestione state machine, boundary layer RX/TX.
 * Step 5.2: port rev2 logic, scheduling neutro (periodo/prioritÃƒÆ’Ã‚Â  = rev1).
 *
 * Architettura JONNY5 v1.0 - Sezione 5.1 - FASE 2
 *
 * NOTE [Refactor-Phase1]:
 *   - Il corpo di rt_loop_step (1 kHz) e il scheduling dei thread RT/IMU
 *     costituiscono il critical path e NON vanno modificati nei refactor.
 *   - Gli interventi consentiti sono limitati a commenti/documentazione
 *     intorno alle funzioni, senza cambiare codice eseguibile.
 */

#include "core/rt_loop.h"
#include "core/state_machine.h"
#include "spi/boundary_buffers.h"
#include "spi/hal_spi_slave.h"
#include "spi/j5_protocol.h"
#include "servo/servo_control.h"
#include "servo/j5vr_actuation.h"
#include "servo/j5vr_manual.h"
#include "servo/motion_planner.h"
#include "imu/imu.h"
#include "uart/uart_control.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rt_loop, LOG_LEVEL_INF);

/* ── Verbosità diagnostica del RT loop ─────────────────────────────────────
 * I LOG_INF diagnostici lungo il path 1 kHz (CTRL / ARM / mode / GUARD /
 * HEAD calib) hanno costo variabile (copia degli argomenti nel buffer di log
 * deferred) e introducono jitter sporadico nelle iterazioni in cui scattano.
 * Con RT_LOOP_VERBOSE=0 (default) RT_DBG() si espande a no-op: gli argomenti
 * non vengono nemmeno valutati e il loop resta deterministico.
 * I LOG di SICUREZZA (WATCHDOG) e gli errori restano SEMPRE attivi.
 * Per il debug sul campo basta ricompilare con -DRT_LOOP_VERBOSE=1. */
#ifndef RT_LOOP_VERBOSE
#define RT_LOOP_VERBOSE 0
#endif
#if RT_LOOP_VERBOSE
#define RT_DBG(...) LOG_INF(__VA_ARGS__)
#else
#define RT_DBG(...) ((void)0)
#endif

/* RT loop thread dedicato (evita dipendenze/limiti system workqueue) */
static struct k_thread rt_thread;
static k_tid_t rt_tid;
K_THREAD_STACK_DEFINE(rt_thread_stack, 4096);

/* ── Sorgente di tick hardware a 1 kHz (TIM6) ──────────────────────────────
 * Il tick dello scheduler RTOS e' 1 ms (CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000):
 * un k_usleep() sub-tick viene arrotondato fino a ~2 tick -> il loop misurava
 * ~512 Hz. Usiamo invece TIM6 (basic timer, nessun pin I/O -> non tocca i PWM)
 * come counter: genera un IRQ di UPDATE ogni 1000 us ESATTI, la cui ISR sblocca
 * il thread RT tramite questo semaforo. Periodo deterministico e indipendente
 * dalla granularita' dello scheduler; il thread dorme fra un tick e l'altro
 * (0% CPU sprecata). Vedi zephyr/overlays/rt_timer.overlay e prj.conf. */
static const struct device *const rt_tick_dev =
    DEVICE_DT_GET(DT_NODELABEL(rt_tick_counter));
K_SEM_DEFINE(rt_tick_sem, 0, 1);
/* Overrun = il tick e' arrivato mentre eravamo ancora dentro rt_loop_step().
 * Con step ~43 us non dovrebbe mai accadere: e' la prova quantitativa che il
 * loop non perde tick. Pubblicato nella riga [RTPERF]. */
static volatile uint32_t rt_tick_overruns;

static void rt_tick_isr_cb(const struct device *dev, void *user_data)
{
    (void)dev; (void)user_data;
    k_sem_give(&rt_tick_sem);
}


/* Periodo RT loop: 1 ms = 1000 Hz */
#define RT_LOOP_PERIOD_MS 1
#define RT_LOOP_PERIOD_US 1000

/* Thread IMU dedicato 400 Hz (fuori dal RT loop; prioritÃƒÆ’Ã‚Â  < system workqueue 5) */
/* Priorita' di scheduling (Zephyr: numero piu' basso = priorita' piu' alta;
 * tutte preemptible): RT(4) > system workqueue(5) > IMU(6) > SPI service(7).
 *
 * RT_THREAD_PRIO=4 tiene il tick di controllo 1 kHz una tacca SOPRA il system
 * workqueue (CONFIG_SYSTEM_WORKQUEUE_PRIORITY=5): prima erano entrambi a 5 e,
 * a parita' di priorita', il flush dei log deferred poteva ritardare il loop
 * oltre il tick TIM6 -> overrun. Restando preemptible (NON cooperativo) il loop
 * e' sempre prelazionabile dagli ISR (TIM6, UART, I2C, DMA SPI) e, essendo
 * cortissimo (~43 us) e auto-sospeso sul semaforo, non affama il workqueue
 * (che mantiene >95% CPU). NB: cooperativo (priorita' negativa) e' stato
 * scartato di proposito: congelerebbe tutto, console inclusa, se un domani il
 * loop smettesse di bloccarsi sul semaforo.
 *
 * IMU e' unico consumatore del bus I2C1 (BNO085); tenerlo sopra spi_service
 * evita che le wake-up del DMA SPI lo preemprano durante il path I2C e il
 * rischio di wedge del bus (SDA trattenuto basso) osservato in campo. */
#define RT_THREAD_PRIO 4
#define IMU_THREAD_STACK_SZ 8192
#define IMU_THREAD_PRIO 6
#define IMU_PERIOD_US 2500
#define IMU_DT_S 0.0025f

K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SZ);
static struct k_thread imu_thread;
static k_tid_t imu_tid;

/* Quanti tick RT devono passare prima che il thread IMU tenti l'init.
 * A 1000 Hz: 2000 tick = 2 s ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â il RT loop ÃƒÆ’Ã‚Â¨ giÃƒÆ’Ã‚Â  a regime quando partiamo. */
#define IMU_INIT_AFTER_RT_TICKS 2000U

static void imu_thread_fn(void *a, void *b, void *c)
{
	/* --- FASE 1: attesa RT loop a regime ---
	 * Dormiamo a fette da 50 ms e contiamo i tick del RT thread.
	 * Quando g_rt_loop_ticks supera la soglia, il RT ÃƒÆ’Ã‚Â¨ sicuramente
	 * a 1000 Hz e possiamo tentare l'init I2C senza rischiare
	 * di bloccare il sistema durante il boot critico. */
	g_imu_thread_stage = 8; /* stage=8: in attesa RT a regime */
	while (g_rt_loop_ticks < IMU_INIT_AFTER_RT_TICKS) {
		g_imu_thread_ticks++;
		k_sleep(K_MSEC(50));
	}
	/* --- FASE 1b: verifica flag IMU abilitata ---
	 * g_imu_reads_enabled parte a 1 (ON di default).
	 * Il comando IMUOFF puÃƒÆ’Ã‚Â² disabilitarla runtime; in quel caso aspettiamo
	 * che venga riabilitata prima di tentare l'init I2C. */
	g_imu_thread_stage = 8; /* stage=8: verifica flag IMU */
	while (!g_imu_reads_enabled) {
		g_imu_thread_ticks++;
		k_sleep(K_MSEC(50));
	}
	/* --- FASE 2: init IMU con retry ---
	 * Se il sensore o il bus I2C non sono pronti subito dopo il boot, non
	 * restiamo bloccati in stage 10 fino a un reset hardware: ritentiamo
	 * periodicamente finche' IMUON resta attivo. */
	while (1) {
		LOG_INF("[IMU] avvio init I2C (rt_ticks=%u, imu_reads_enabled=%u)",
			(unsigned)g_rt_loop_ticks, (unsigned)g_imu_reads_enabled);
		g_imu_thread_stage = 9;
		{
			int rc = imu_init();
			if (rc == 0) {
				break;
			}
			LOG_ERR("[IMU] init fallita rc=%d - retry tra 1 s", rc);
		}

		while (!g_imu_reads_enabled) {
			g_imu_thread_ticks++;
			g_imu_thread_stage = 8;
			k_sleep(K_MSEC(100));
		}

		g_imu_thread_ticks++;
		g_imu_thread_stage = 10;
		k_sleep(K_MSEC(1000));
	}
	LOG_INF("[IMU] init ok - avvio loop 400 Hz");


	int64_t next_us = (int64_t)k_uptime_get() * 1000 + IMU_PERIOD_US;

	while (1) {
		g_imu_thread_ticks++;

		if (g_imu_reads_enabled)
		{
			g_imu_thread_stage = 1;
			imu_update_orientation(IMU_DT_S);
			g_imu_thread_stage = 2;
		}
		else
		{
			g_imu_thread_stage = 0;
		}

		int64_t now_us = (int64_t)k_uptime_get() * 1000;
		int64_t sleep_us = next_us - now_us;
		if (sleep_us > 0) {
			k_usleep((uint32_t)sleep_us);
		}
		next_us += IMU_PERIOD_US;
	}
}


/* Flag legacy rimosso: j5vr_center_demo() eliminata, non piÃƒÆ’Ã‚Â¹ necessario. */

/* Diagnostica: tick counter del RT loop (1 incremento per handler) */
volatile uint32_t g_rt_loop_ticks = 0;
volatile uint8_t g_rt_loop_stage = 0;

/* Periodo runtime del RT loop in microsecondi (EWMA, finestra ~16 campioni).
 * 0 = ancora non misurato (warm-up). Aggiornato da rt_thread_fn dopo ogni
 * iterazione, pubblicato in TELEMETRY reserved[62-63] da j5_build_frame. */
volatile uint16_t g_rt_loop_period_us = 0;

/* Tempo di esecuzione PURO di rt_loop_step() in microsecondi (EWMA ~16 iter).
 * Distinto da g_rt_loop_period_us (che include anche lo sleep/quantizzazione
 * tick): serve a separare il costo di CALCOLO dal costo di SCHEDULING. Se
 * g_rt_step_us << g_rt_loop_period_us, il collo di bottiglia non è il calcolo
 * ma la granularità del tick del kernel. 0 = non ancora misurato. */
volatile uint16_t g_rt_step_us = 0;

/* Diagnostica teleop VR gating */
volatile uint8_t g_vr_armed = 0;
volatile uint8_t g_vr_freeze_active = 0;
volatile uint32_t g_vr_guard_block_count = 0;
volatile uint8_t g_vr_input_active = 0;

volatile uint32_t g_imu_thread_ticks = 0;
volatile uint8_t g_imu_thread_stage = 0;
/* IMU abilitata di default: il thread IMU avvia l'init I2C non appena il RT loop
 * ÃƒÆ’Ã‚Â¨ a regime (>2s). Il comando IMUOFF puÃƒÆ’Ã‚Â² disabilitarla runtime, IMUON la riabilita. */
volatile uint8_t g_imu_reads_enabled = 1;

static inline int iabs_i32(int v) { return (v < 0) ? -v : v; }

/* Guard/freeze tuning */
#define VR_INPUT_CHANGE_EPS_I16  200    /* se cambia meno di cosÃƒÆ’Ã‚Â¬, consideriamo invariato */
/* |norm| dopo j5vr_int16_to_normalized_dz: allineato a j5vr_apply_setpoint_incremental (stessi > 0.01f). */
#define VR_STICK_NORM_ACTIVE_EPS 0.01f
#define VR_FREEZE_AFTER_MS       200    /* freeze PWM dopo stabilitÃƒÆ’Ã‚Â  input */

/* Pulsante user Nucleo (B1 / PC13) - alias sw0 */
#define CENTER_BUTTON_NODE DT_ALIAS(sw0)
#if DT_NODE_EXISTS(CENTER_BUTTON_NODE)
#define CENTER_BUTTON_GPIO_PORT DT_GPIO_CTLR(CENTER_BUTTON_NODE, gpios)
#define CENTER_BUTTON_GPIO_PIN DT_GPIO_PIN(CENTER_BUTTON_NODE, gpios)
#define CENTER_BUTTON_GPIO_FLAGS DT_GPIO_FLAGS(CENTER_BUTTON_NODE, gpios)
static const struct device *center_button_dev = DEVICE_DT_GET(CENTER_BUTTON_GPIO_PORT);
static bool center_button_initialized = false;
#define CENTER_BUTTON_DEBOUNCE_SAMPLES 5
static uint8_t center_button_debounce_counter = 0;
static bool center_button_prev_state = false;
static bool center_button_debounced_state = false;
#endif

/* Tracciamento stato precedente per rilevare entry in STATE_SAFE */
static system_state_t rt_prev_state = STATE_SAFE;
/* Reset a ogni nuova entry in STATE_SAFE, cosÃƒÆ’Ã‚Â¬ SAFEÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢IDLE puÃƒÆ’Ã‚Â² avvenire piÃƒÆ’Ã‚Â¹ volte */
static bool auto_transition_done = false;

static void rt_loop_step(void)
{
    g_rt_loop_ticks++;
    g_rt_loop_stage = 1;

    /* SPI frame watchdog: ogni 100 tick (= 100 ms) verifica che il Pi stia
     * ancora inviando frame. Se il timeout scade e il sistema e' in IDLE,
     * forza STATE_SAFE — i servo vengono disabilitati nel case STATE_SAFE. */
    if ((g_rt_loop_ticks % 100U) == 0U)
    {
        if (state_machine_is_movement_allowed() &&
            hal_spi_last_frame_age_ms() > SPI_FRAME_TIMEOUT_MS)
        {
            LOG_WRN("[WATCHDOG] SPI timeout %u ms -- SAFE",
                    (unsigned)hal_spi_last_frame_age_ms());
            state_machine_set_safe();
        }
    }

    /* BARRIERA DI SICUREZZA: in STOPPED nessun movimento ÃƒÆ’Ã‚Â¨ consentito.
     * Questo check ha prioritÃƒÆ’Ã‚Â  assoluta su SETPOSE e pipeline VR.
     * servo_disable_all() viene giÃƒÆ’Ã‚Â  chiamato nel case STATE_STOPPED del
     * switch sottostante, ma uscire prima garantisce che nessun tick di
     * traiettoria modifichi desired_positions[] prima dello stop. */
    if (state_machine_get_state() == STATE_STOPPED)
    {
        servo_disable_all();
        return;
    }

    /* SETPOSE tick ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â ha prioritÃƒÆ’Ã‚Â  su VR manual/head/hybrid quando attivo.
     * Se j5vr_setpose_tick() ritorna true significa che un comando SETPOSE
     * ÃƒÆ’Ã‚Â¨ in esecuzione: l'intero blocco STATE_IDLE viene saltato per questo
     * ciclo, la pipeline VR resta intatta (desired_positions[] viene
     * aggiornato dalla traiettoria SETPOSE, non dagli stick VR). */
    if (j5vr_setpose_tick(g_rt_loop_ticks))
    {
        return;
    }



    system_state_t cur_state = state_machine_get_state();
    /* Reset flag transizione automatica ogni volta che si entra in STATE_SAFE
     * da uno stato diverso, cosÃƒÆ’Ã‚Â¬ SAFEÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢IDLE puÃƒÆ’Ã‚Â² avvenire piÃƒÆ’Ã‚Â¹ volte. */
    if (cur_state == STATE_SAFE && rt_prev_state != STATE_SAFE)
    {
        auto_transition_done = false;
    }
    rt_prev_state = cur_state;

    switch (cur_state)
    {
        case STATE_SAFE:
            g_rt_loop_stage = 20;
            servo_disable_all();
            g_rt_loop_stage = 21;
            {
                struct j5vr_state j5vr_check;
                j5vr_check = g_j5vr_latest;
                if (j5vr_check.mode != 0 || j5vr_check.vr_heartbeat > 0)
                {
                    if (!auto_transition_done)
                    {
                        if (state_machine_set_idle())
                        {
                            auto_transition_done = true;
                        }
                    }
                }
            }
            break;

        case STATE_IDLE:
            {
                g_rt_loop_stage = 30;
                bool movement_allowed = state_machine_is_movement_allowed();
                struct j5vr_state j5vr_current;
                j5vr_current = g_j5vr_latest;
                g_rt_loop_stage = 31;

                bool grip_left = false;
                bool grip_right = false;
                bool deadman_active = false;
                if (j5vr_current.mode <= 255)
                {
                    grip_left = (j5vr_current.buttons_left & (1U << 1)) != 0;
                    grip_right = (j5vr_current.buttons_right & (1U << 1)) != 0;
                    /* Deadman classico: doppio grip contemporaneo anche in mode 5. */
                    deadman_active = grip_left && grip_right;
                }
                g_rt_loop_stage = 32;

                /* Input guard/freeze: valuta cambi e attivitÃƒÆ’Ã‚Â  stick */
                static bool last_inited = false;
                static int16_t last_jx = 0, last_jy = 0, last_p = 0, last_y = 0;
                static uint32_t last_change_ms = 0;
                static uint32_t last_active_ms = 0;
                static bool servos_disabled_latched = false;
                static bool last_freeze = false;
                static bool last_guarded = false;
                const uint32_t now_ms = k_uptime_get_32();

                const bool changed =
                    (!last_inited) ||
                    (iabs_i32((int)j5vr_current.joy_x - (int)last_jx) > VR_INPUT_CHANGE_EPS_I16) ||
                    (iabs_i32((int)j5vr_current.joy_y - (int)last_jy) > VR_INPUT_CHANGE_EPS_I16) ||
                    (iabs_i32((int)j5vr_current.pitch - (int)last_p) > VR_INPUT_CHANGE_EPS_I16) ||
                    (iabs_i32((int)j5vr_current.yaw - (int)last_y) > VR_INPUT_CHANGE_EPS_I16);

                if (!last_inited || changed) {
                    last_change_ms = now_ms;
                    last_jx = j5vr_current.joy_x;
                    last_jy = j5vr_current.joy_y;
                    last_p  = j5vr_current.pitch;
                    last_y  = j5vr_current.yaw;
                    last_inited = true;
                }

                /* Stick "attivi" = stessa deadzone joy_dz + soglia norm di MANUAL (no zona grigia vs actuation). */
                const float joy_dz_rt = j5vr_head_get_params()->joy_dz;
                const float jx_n = j5vr_int16_to_normalized_dz(j5vr_current.joy_x, joy_dz_rt);
                const float jy_n = j5vr_int16_to_normalized_dz(j5vr_current.joy_y, joy_dz_rt);
                const float pt_n = j5vr_int16_to_normalized_dz(j5vr_current.pitch, joy_dz_rt);
                const float yw_n = j5vr_int16_to_normalized_dz(j5vr_current.yaw, joy_dz_rt);
                const bool stick_active =
                    (fabsf(jx_n) > VR_STICK_NORM_ACTIVE_EPS) ||
                    (fabsf(jy_n) > VR_STICK_NORM_ACTIVE_EPS) ||
                    (fabsf(pt_n) > VR_STICK_NORM_ACTIVE_EPS) ||
                    (fabsf(yw_n) > VR_STICK_NORM_ACTIVE_EPS);
                const uint16_t btn_mask_no_grip = (1U << 0) | (1U << 3) | (1U << 4) | (1U << 5); /* trigger, thumbstick, X/A, Y/B */
                const bool buttons_active =
                    ((j5vr_current.buttons_left & btn_mask_no_grip) != 0U) ||
                    ((j5vr_current.buttons_right & btn_mask_no_grip) != 0U);
                const bool inputs_active = stick_active || buttons_active;

                g_vr_input_active = inputs_active ? 1U : 0U;

                if (inputs_active) {
                    last_active_ms = now_ms;
                }

                /* ARM quando deadman e input diventano attivi (ordine libero).
                 * Per sicurezza usiamo edge (deadman 0->1 o input_active 0->1) e inizializziamo i prev
                 * al primo campione per evitare "false edge" al boot su stato stale. */
                {
                    static bool deadman_inited = false;
                    static bool deadman_prev = false;
                    static bool inputs_inited = false;
                    static bool inputs_prev = false;
                    bool deadman_now = deadman_active;
                    bool deadman_edge = false;
                    bool inputs_now = inputs_active;
                    bool inputs_edge = false;
                    if (!deadman_inited)
                    {
                        deadman_prev = deadman_now;
                        deadman_inited = true;
                    }
                    else
                    {
                        deadman_edge = (!deadman_prev && deadman_now);
                    }

                    if (!inputs_inited)
                    {
                        inputs_prev = inputs_now;
                        inputs_inited = true;
                    }
                    else
                    {
                        inputs_edge = (!inputs_prev && inputs_now);
                    }

                    const uint8_t arm_before = g_vr_armed;

                    /* DEARM immediato */
                    if (!deadman_now || !movement_allowed)
                    {
                        g_vr_armed = 0;
                    }
                    else if (!g_vr_armed)
                    {
                        /* Arming: edge classico OPPURE in Manual VR (1) basta deadman + heartbeat (no edge) */
                        bool arm_by_edge =
                            (j5vr_current.vr_heartbeat > 0) &&
                            ((deadman_edge && inputs_now) || (inputs_edge && deadman_now));
                        bool arm_in_manual_vr =
                            (j5vr_current.mode == 2U) && (j5vr_current.vr_heartbeat > 0U);
                        bool arm_in_ik_vr =
                            (j5vr_current.mode == 5U) && (j5vr_current.vr_heartbeat > 0U);
                        if (arm_by_edge || arm_in_manual_vr || arm_in_ik_vr)
                        {
                            g_vr_armed = 1;
                            servos_disabled_latched = false;
                        }
                    }

                    deadman_prev = deadman_now;
                    inputs_prev = inputs_now;

                    /* Log solo quando cambia arm (no spam) */
                    if (g_vr_armed != arm_before)
                    {
                        RT_DBG("[ARM] d_edge=%u i_edge=%u deadman=%u allowed=%u input=%u -> arm=%u",
                               (unsigned)(deadman_edge ? 1U : 0U),
                               (unsigned)(inputs_edge ? 1U : 0U),
                               (unsigned)(deadman_now ? 1U : 0U),
                               (unsigned)(movement_allowed ? 1U : 0U),
                               (unsigned)(inputs_active ? 1U : 0U),
                               (unsigned)g_vr_armed);
                    }
                }

                /* Se non armato, disabilita servos; eccezione: Manual VR (2) o IK VR (5) con deadman+heartbeat */
                bool allow_servos_in_manual = (j5vr_current.mode == 2U) && deadman_active && (j5vr_current.vr_heartbeat > 0U);
                bool allow_servos_in_mode5  = (j5vr_current.mode == 5U) && deadman_active && (j5vr_current.vr_heartbeat > 0U);
                if (!g_vr_armed && !allow_servos_in_manual && !allow_servos_in_mode5)
                {
                    if (!servos_disabled_latched)
                    {
                        servo_disable_all();
                        servos_disabled_latched = true;
                    }
                }

                /* Log controllo (rate-limited): stato gating + input */
                {
                    static uint32_t ctrl_log = 0;
                    if ((ctrl_log++ % 1000U) == 0U)
                    {
                        RT_DBG("[CTRL] st=%u m=%u dm=%u ok=%u jx=%d jy=%d p=%d y=%d i=%u",
                               (unsigned)state_machine_get_state(),
                               (unsigned)j5vr_current.mode,
                               (unsigned)(deadman_active ? 1U : 0U),
                               (unsigned)(movement_allowed ? 1U : 0U),
                               (int)j5vr_current.joy_x,
                               (int)j5vr_current.joy_y,
                               (int)j5vr_current.pitch,
                               (int)j5vr_current.yaw,
                               (unsigned)j5vr_current.intensity);
                    }
                }

                /* Log mode change only on edge (no per-tick spam).
                 * Transizione 4 (POSE_VR) -> 1 (MANUAL_VR): resetta freeze e re-arm cosÃƒÆ’Ã‚Â¬ il robot
                 * risponde subito senza dover uscire/rientrare o muovere lo stick per riavere l'edge. */
                {
                    static uint8_t last_mode = 0xFF;
                    if (j5vr_current.mode != last_mode)
                    {
                        if (last_mode == 1U && j5vr_current.mode == 2U)
                        {
                            last_active_ms = now_ms;
                            if (deadman_active && movement_allowed && (j5vr_current.vr_heartbeat > 0U))
                            {
                                g_vr_armed = 1;
                                servos_disabled_latched = false;
                            }
                        }
                        /* Ingresso in HEAD (3) o HYBRID (4): reset calibrazione testa
                         * e accumulatori EMA così l’errore parte dalla posa corrente. */
                        if (j5vr_current.mode == 3U || j5vr_current.mode == 4U || j5vr_current.mode == 5U)
                        {
                            j5vr_reset_head_calib();
                            RT_DBG("[J5VR] HEAD calib reset (mode=%u)", j5vr_current.mode);
                        }
                        RT_DBG("[J5VR] mode=%u", j5vr_current.mode);
                        last_mode = j5vr_current.mode;
                    }
                }

                if (!deadman_active)
                {
                    motion_planner_stop_all();
                }
                else if (!movement_allowed)
                {
                    /* No actuation when movement not allowed */
                }
                else
                {
                    switch (j5vr_current.mode)
                    {
                    case 0: /* CALIBRATION_CAMERA: no teleop actuation */
                        motion_planner_stop_all();
                        break;
                    case 2: /* MANUAL_VR */
                        {
                            g_rt_loop_stage = 33;
                            /* Re-arm di riserva: in Manual VR con deadman e heartbeat ma non armato
                             * (es. dopo Pose VR se il frame 4->1 ÃƒÆ’Ã‚Â¨ andato perso o cÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã‚Â¨ stato un mode 0 in mezzo). */
                            if (!g_vr_armed && deadman_active && movement_allowed && (j5vr_current.vr_heartbeat > 0U))
                            {
                                g_vr_armed = 1;
                                servos_disabled_latched = false;
                                last_active_ms = now_ms;
                            }
                            /* setpoint non viene letto dopo la chiamata; ok viene usato
                             * piÃƒÆ’Ã‚Â¹ in basso per condizionare j5vr_apply_setpoint_incremental. */
                            servo_setpoint_t setpoint;
                            bool ok = j5vr_to_servo_setpoint(&j5vr_current, &setpoint);
                            /* In Manual VR blocchiamo solo per freeze (200ms senza input).
                             * Non usiamo g_vr_armed qui cosÃƒÆ’Ã‚Â¬ dopo Pose VR il robot risponde subito (deadman giÃƒÆ’Ã‚Â  richiesto per entrare nel switch). */
                            bool freeze = false;
                            bool guard_block = false;
                            if (!inputs_active && (now_ms - last_active_ms) > VR_FREEZE_AFTER_MS)
                            {
                                freeze = true;
                                guard_block = true;
                            }

                            g_vr_freeze_active = freeze ? 1U : 0U;

                            if (guard_block)
                            {
                                g_vr_guard_block_count++;
                                /* Azzera incrementi diagnostici quando blocchiamo */
                                g_j5vr_last_inc_mdeg_yaw = 0;
                                g_j5vr_last_inc_mdeg_pitch = 0;
                                g_j5vr_last_inc_mdeg_spalla = 0;
                                g_j5vr_last_inc_mdeg_gomito = 0;

                                /* RUNTIME_EXPERIMENT [PR_FREEZE_NO_RELAX]: non chiamare
                                 * servo_relax_digital() qui. Quella routine agisce solo su
                                 * PITCH/ROLL (DIGITAL_SERVO_MASK); saltarla isola causalmente
                                 * se il relax digitale nel freeze/guard alimenta lo stato tossico.
                                 * Revertire questa riga per ripristinare il buzz-stop su P/R. */
                                /* servo_relax_digital(); */

                                if (!last_guarded || (freeze != last_freeze))
                                {
                                    RT_DBG("[GUARD] armed=%u freeze=%u input=%u stable_ms=%u",
                                           (unsigned)g_vr_armed,
                                           (unsigned)(freeze ? 1U : 0U),
                                           (unsigned)(inputs_active ? 1U : 0U),
                                           (unsigned)(now_ms - last_change_ms));
                                }
                                last_guarded = true;
                                last_freeze = freeze;
                            }
                            else
                            {
                                last_guarded = false;
                                last_freeze = false;
                                if (ok)
                                {
                                    j5vr_apply_setpoint_incremental(&j5vr_current, movement_allowed);
                                }
                            }
                            g_rt_loop_stage = 34;
                        }
                        break;
                    case 1: /* POSE_VR: no joystick actuation; pose via one-shot TELEOPPOSE only */
                        motion_planner_stop_all();
                        break;
                    case 3: /* HEAD_VR */
                        j5vr_apply_head_tracking(&j5vr_current);
                        break;
                    case 4: /* HYBRID_VR */
                        /* HYBRID è governata dalla mode VR (4) senza consenso UART separato.
                         * Questo allinea il comportamento a MANUAL/HEAD lato UI. */
                        j5vr_apply_hybrid(&j5vr_current);
                        break;
                    case 5: /* HEAD ASSIST: B/S/G da J5VR (marker I), polso da pipeline HEAD */
                        j5vr_apply_mode5_arm_head(&j5vr_current);
                        break;
                    default:
                        /* Unknown mode: safe fallback to MANUAL */
                        {
                            /* setpoint non viene usato direttamente; la chiamata
                             * valida l'input prima di procedere con l'incrementale. */
                            servo_setpoint_t setpoint;
                            if (j5vr_to_servo_setpoint(&j5vr_current, &setpoint))
                                j5vr_apply_setpoint_incremental(&j5vr_current, movement_allowed);
                        }
                        break;
                    }
                }
            }
            break;

        case STATE_STOPPED:
            /* Raggiunto solo se lo stato cambia nel mezzo del ciclo ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â
             * il return anticipato in cima a rt_loop_step() gestisce il caso normale. */
            servo_disable_all();
            break;
    }

    g_rt_loop_stage = 80;
#if DT_NODE_EXISTS(CENTER_BUTTON_NODE)
    /* Polling bottone decimato a ~50 Hz (ogni 20 tick). Un pulsante fisico non
     * richiede campionamento a 1 kHz, e la lettura GPIO è I/O che aggiunge
     * overhead fisso a OGNI iterazione del loop. Con debounce a 5 campioni @
     * 50 Hz si ottengono ~100 ms di filtro, adeguati per un tasto manuale. */
    if (((g_rt_loop_ticks % 20U) == 0U) &&
        center_button_initialized && device_is_ready(center_button_dev))
    {
        int ret = gpio_pin_get(center_button_dev, CENTER_BUTTON_GPIO_PIN);
        bool button_pressed = (ret < 0) ? false : (ret == 0);

        if (button_pressed == center_button_debounced_state)
            center_button_debounce_counter = 0;
        else
        {
            center_button_debounce_counter++;
            if (center_button_debounce_counter >= CENTER_BUTTON_DEBOUNCE_SAMPLES)
            {
                center_button_debounced_state = button_pressed;
                center_button_debounce_counter = 0;
            }
        }

        if (center_button_debounced_state && !center_button_prev_state)
        {
            /* Bottone Nucleo: porta a HOME via SETPOSE (RTR5 + 40%) se in IDLE */
            if (state_machine_get_state() == STATE_IDLE)
            {
                j5vr_center_all_servos();
            }
        }
        center_button_prev_state = center_button_debounced_state;
    }
#endif

    g_rt_loop_stage = 81;
}

static void rt_thread_fn(void *a, void *b, void *c)
{
    (void)a; (void)b; (void)c;

    /* EWMA (microsecondi * 16, alpha = 1/16) di due metriche distinte:
     *  - period: tempo wall-clock fra inizio di due iterazioni consecutive
     *            (= step + sleep + jitter scheduler) → g_rt_loop_period_us
     *  - step:   tempo PURO di rt_loop_step() → g_rt_step_us
     * Il confronto fra i due isola il costo di calcolo da quello di scheduling. */
    uint32_t prev_iter_cyc = k_cycle_get_32();
    uint32_t period_us_ewma_x16 = 0;
    uint32_t step_us_ewma_x16 = 0;

    while (1) {
        /* Attendi il tick hardware a 1 kHz (TIM6 -> ISR -> semaforo). Timeout
         * 2 ms = rete di sicurezza: se la sorgente HW si fermasse il loop non
         * si blocca all'infinito ma prosegue (degradato) tenendo vivi servo,
         * watchdog e servizio SPI. In funzionamento normale il semaforo arriva
         * ogni 1000 us esatti, ben dentro il timeout. */
        if (k_sem_count_get(&rt_tick_sem) > 0u) {
            rt_tick_overruns++; /* tick gia' pendente: l'iter precedente ha sforato 1 ms */
        }
        (void)k_sem_take(&rt_tick_sem, K_MSEC(2));

        uint32_t iter_start_cyc = k_cycle_get_32();

        /* Periodo = inizio_iter(N) - inizio_iter(N-1) */
        uint32_t period_dt_us = (uint32_t)k_cyc_to_us_floor32(iter_start_cyc - prev_iter_cyc);
        prev_iter_cyc = iter_start_cyc;
        /* Sanity: scarta outlier (boot, pause debugger). 100..60000 us = 16-10000 Hz. */
        if (period_dt_us >= 100u && period_dt_us <= 60000u) {
            if (period_us_ewma_x16 == 0u) {
                period_us_ewma_x16 = period_dt_us * 16u;
            } else {
                period_us_ewma_x16 -= (period_us_ewma_x16 >> 4);
                period_us_ewma_x16 += period_dt_us;
            }
            g_rt_loop_period_us = (uint16_t)(period_us_ewma_x16 >> 4);
        }

        rt_loop_step();

        /* S = tempo puro di rt_loop_step() */
        uint32_t step_dt_us = (uint32_t)k_cyc_to_us_floor32(k_cycle_get_32() - iter_start_cyc);
        if (step_dt_us <= 60000u) {
            if (step_us_ewma_x16 == 0u) {
                step_us_ewma_x16 = step_dt_us * 16u;
            } else {
                step_us_ewma_x16 -= (step_us_ewma_x16 >> 4);
                step_us_ewma_x16 += step_dt_us;
            }
            g_rt_step_us = (uint16_t)(step_us_ewma_x16 >> 4);
        }

        /* Telemetria prestazioni su console (USART2 / ST-Link VCP) ogni ~5 s.
         * Fuori dal punto critico (scatta 1 volta ogni 5000 iter); utile per
         * leggere periodo/step/freq durante il test senza la dashboard. */
        if ((g_rt_loop_ticks % 5000U) == 0U) {
            LOG_INF("[RTPERF] period=%u us  step=%u us  overruns=%u  (~%u Hz)",
                    (unsigned)g_rt_loop_period_us,
                    (unsigned)g_rt_step_us,
                    (unsigned)rt_tick_overruns,
                    (unsigned)(g_rt_loop_period_us ? (1000000U / g_rt_loop_period_us) : 0U));
        }

        /* Nessun k_usleep qui: la cadenza e' data dal tick hardware TIM6,
         * atteso in cima al loop. Il thread torna a dormire sul semaforo,
         * quindi non spreca CPU fra un tick e l'altro. */
    }
}

void rt_loop_init(void)
{
    if (!servo_control_init())
        LOG_ERR("[RT] Servo control init failed");

    j5vr_actuation_init();

    /* IMPORTANT:
     * Non scrivere PWM non-zero al boot: i servo restano disabilitati (pulse=0)
     * finchÃƒÆ’Ã‚Â© non arriva un intent VR valido con deadman=1 e input attivo. */
    LOG_INF("[RT] Servo PWM kept disabled at boot (await VR arm)");


#if DT_NODE_EXISTS(CENTER_BUTTON_NODE)
    if (device_is_ready(center_button_dev))
    {
        int ret = gpio_pin_configure(center_button_dev, CENTER_BUTTON_GPIO_PIN,
                                    GPIO_INPUT | CENTER_BUTTON_GPIO_FLAGS);
        if (ret == 0)
        {
            center_button_initialized = true;
            int pin_state = gpio_pin_get(center_button_dev, CENTER_BUTTON_GPIO_PIN);
            center_button_debounced_state = (pin_state < 0) ? false : (pin_state == 0);
            center_button_prev_state = center_button_debounced_state;
            LOG_INF("[RT] Center button (PC13) initialized");
        }
        else
            LOG_ERR("[RT] Failed to configure center button GPIO: %d", ret);
    }
    else
        LOG_WRN("[RT] Center button GPIO device not ready");
#else
    LOG_WRN("[RT] Center button node not found in device tree");
#endif

    if (!imu_tid) {
        imu_tid = k_thread_create(
            &imu_thread,
            imu_thread_stack,
            K_THREAD_STACK_SIZEOF(imu_thread_stack),
            imu_thread_fn,
            NULL, NULL, NULL,
            IMU_THREAD_PRIO,
            0,
            K_NO_WAIT
        );
        k_thread_name_set(imu_tid, "imu_400hz");
    }
}

void rt_loop_start(void)
{
    if (!rt_tid) {
        /* Avvia la sorgente di tick hardware (TIM6) PRIMA del thread RT.
         * counter_us_to_ticks() converte usando la frequenza REALE del timer
         * letta dal driver, quindi il periodo resta esattamente
         * RT_LOOP_PERIOD_US a prescindere dal clock di bus APB1.
         * Se il counter non parte, il thread RT non resta bloccato: usa
         * k_sem_take con timeout 2 ms (vedi rt_thread_fn) e prosegue in
         * modalita' degradata tenendo vivi servo/WDT/SPI. */
        if (device_is_ready(rt_tick_dev)) {
            struct counter_top_cfg top_cfg = {
                .ticks = counter_us_to_ticks(rt_tick_dev, RT_LOOP_PERIOD_US),
                .callback = rt_tick_isr_cb,
                .user_data = NULL,
                .flags = 0,
            };
            int rc = counter_set_top_value(rt_tick_dev, &top_cfg);
            if (rc == 0) {
                rc = counter_start(rt_tick_dev);
            }
            if (rc == 0) {
                LOG_INF("[RT] 1 kHz tick = TIM6 hw (%u counts/period @ %u Hz)",
                        (unsigned)top_cfg.ticks,
                        (unsigned)counter_get_frequency(rt_tick_dev));
            } else {
                LOG_ERR("[RT] TIM6 tick init failed (%d) - degraded timeout mode", rc);
            }
        } else {
            LOG_ERR("[RT] TIM6 counter not ready - degraded timeout mode");
        }

        rt_tid = k_thread_create(
            &rt_thread,
            rt_thread_stack,
            K_THREAD_STACK_SIZEOF(rt_thread_stack),
            rt_thread_fn,
            NULL, NULL, NULL,
            RT_THREAD_PRIO, /* 4 = una tacca sopra il system workqueue (5); vedi nota a inizio file */
            0,
            K_NO_WAIT
        );
        k_thread_name_set(rt_tid, "rt_loop_1khz");
    }
}

