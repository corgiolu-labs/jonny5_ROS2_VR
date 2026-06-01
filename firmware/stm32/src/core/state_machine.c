/*
 * State Machine - Implementation
 * 
 * State machine realtime minimale.
 * Stato iniziale: SAFE
 * 
 * Architettura JONNY5 v1.0 - Sezione 5.3
 */

#include "core/state_machine.h"
#include "spi/j5_protocol.h"
#include <stddef.h>

/* Stato corrente */
static volatile system_state_t current_state = STATE_SAFE;

/* Inizializzazione state machine (design industrial-grade: stato iniziale SAFE) */
void state_machine_init(void)
{
    current_state = STATE_SAFE;
}

/* Ottiene stato corrente.
 * Sanity check: se per qualche motivo (corruzione memoria, stack overflow di
 * altri thread, upset) current_state finisce fuori dai valori enum validi,
 * lo ricuciniamo a STATE_SAFE invece di propagare un UNKNOWN all'esterno.
 */
system_state_t state_machine_get_state(void)
{
    system_state_t s = current_state;
    if (s != STATE_SAFE && s != STATE_IDLE && s != STATE_STOPPED)
    {
        current_state = STATE_SAFE;
        s = STATE_SAFE;
    }
    return s;
}

/* Transizione a SAFE (da qualsiasi stato; idempotente) */
bool state_machine_set_safe(void)
{
    current_state = STATE_SAFE;
    return true;
}

/* Transizione a IDLE: SAFE→IDLE o IDLE→IDLE (idempotente); da STOPPED ritorna false (sequenza: STOPPED→SAFE→ENABLE→IDLE). */
bool state_machine_set_idle(void)
{
    if (current_state == STATE_SAFE)
    {
        current_state = STATE_IDLE;
        return true;
    }
    if (current_state == STATE_IDLE)
    {
        return true;  /* idempotente */
    }
    return false;  /* STOPPED: nessun cambio */
}

/* Transizione a STOPPED (sempre possibile) */
void state_machine_set_stopped(void)
{
    current_state = STATE_STOPPED;
}

/* Verifica se il movimento è consentito */
bool state_machine_is_movement_allowed(void)
{
    /* Movimento consentito solo in IDLE */
    return (current_state == STATE_IDLE);
}
