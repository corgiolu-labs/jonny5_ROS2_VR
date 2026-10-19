/*
 * E-STOP hardware - Header
 *
 * Fungo di emergenza su PC6 (JONNY5 SHIELD rev3, solo build G474).
 * Linea ALTA = STOP ATTIVO (fail-safe: NC verso GND + pull-up esterno R8).
 * Sul F446 (nessun alias DT "estop") l'implementazione e' uno stub no-op
 * con estop_is_active() == false: zero cambi di comportamento.
 */

#ifndef ESTOP_H
#define ESTOP_H

#include <stdbool.h>
#include <stdint.h>

/* Configura il GPIO da DT_ALIAS(estop) e pre-carica lo stato debounced.
 * Da chiamare una volta al boot (main.c), dopo uart_control_init(). */
void estop_init(void);

/* Campionamento + debounce + azioni di stop. Da chiamare a ogni tick 1 kHz
 * del RT loop (rt_thread_fn), PRIMA di rt_loop_step(): cosi' viene valutato
 * sempre, anche in STOPPED o durante un SETPOSE. Non-bloccante. */
void estop_poll_tick(void);

/* Stato debounced corrente: true = STOP attivo (fungo premuto o filo aperto),
 * SALVO bypass attivo (estop_bypass_set): allora sempre false. */
bool estop_is_active(void);

/* Stato debounced della LINEA, ignorando il bypass (telemetria / ESTOP?). */
bool estop_line_active(void);

/* Bypass temporaneo del fungo (prove sul campo senza il box E-STOP collegato,
 * 2026-10-19): la linea viene ignorata (niente latch STOPPED, niente gate UART)
 * fino alla scadenza o a ESTOP_BYPASS OFF. Non persiste al riavvio. Alla
 * scadenza il robot viene fermato come per una pressione del fungo e il Pi
 * riceve "ESTOP_BYPASS_EXPIRED". Ritorna i secondi residui (0 = off),
 * -1 = nessun E-STOP hardware su questa board (stub F446). */
int32_t estop_bypass_set(bool on, uint32_t minutes);
bool estop_bypass_active(void);
int32_t estop_bypass_remaining_s(void);

/* Emette le notifiche "ESTOP"/"ESTOP_CLEAR" pendenti verso il Pi. Da chiamare
 * dal main loop (stesso thread di uart_control_process): i fronti nel tick RT
 * settano solo flag, cosi' il busy-wait di uart_poll_out non causa overrun del
 * tick 1 kHz ne' interleaving con le risposte in corso. Stub no-op sul F446. */
void estop_notify_poll(void);

#endif /* ESTOP_H */
