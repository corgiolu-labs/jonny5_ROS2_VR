/*
 * State Machine - Header
 * 
 * State machine realtime minimale per FASE 1.
 * Stati: SAFE, IDLE, STOPPED
 * 
 * Architettura JONNY5 v1.0 - Sezione 5.3
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

/* Stati della state machine */
typedef enum
{
    STATE_SAFE = 0,      /* Stato iniziale passivo (attuatori disabilitati).
                          * Uscita: SPI frame valido con mode != 0 o heartbeat > 0
                          * -> transizione automatica a IDLE in rt_loop_step. */
    STATE_IDLE,          /* Sistema attivo, pronto per comandi e pipeline VR. */
    STATE_STOPPED        /* Arresto immediato per fault (STOP/UART o emergenza).
                          * INTENZIONALE: non ha uscita software. L'unico recovery
                          * e' il reset hardware (power cycle o NRST).
                          * Garantisce che un fault non sia mascherabile via software. */
} system_state_t;

/* Inizializzazione state machine */
void state_machine_init(void);

/* Ottiene stato corrente */
system_state_t state_machine_get_state(void);

/* Transizione a SAFE (da qualsiasi stato; idempotente). Ritorna true. */
bool state_machine_set_safe(void);

/* Transizione a IDLE: da SAFE→IDLE o IDLE→IDLE (idempotente); da STOPPED ritorna false. Sequenza: STOPPED→SAFE→ENABLE→IDLE. */
bool state_machine_set_idle(void);

/* Transizione a STOPPED (sempre possibile) */
void state_machine_set_stopped(void);

/* Verifica se il movimento è consentito */
bool state_machine_is_movement_allowed(void);


#endif /* STATE_MACHINE_H */
