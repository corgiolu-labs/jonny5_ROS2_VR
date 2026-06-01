/*
 * j5vr_setpose.h — Traiettorie SETPOSE / SETPOSE_T
 *
 * Modulo estratto da j5vr_actuation.c: pianificazione e tick di traiettorie
 * punto-punto con profili RTR3/RTR5/BB/BCB.
 *
 * Dipendenze globali di actuation.c esposte come extern:
 *   - desired_positions[]   (array float, SERVO_COUNT)
 *   - step_accumulator[]    (array float, SERVO_COUNT)
 * La definizione rimane in j5vr_actuation.c; questo header li dichiara
 * per l'uso interno di j5vr_setpose.c.
 *
 * JONNY5-4.0 — Step 3.3 refactor (2026-02-25)
 */

#ifndef J5VR_SETPOSE_H
#define J5VR_SETPOSE_H

#include "servo/j5vr_actuation.h"   /* j5_profile_t, servo_setpoint_t */
#include "servo/servo_control.h"    /* SERVO_COUNT, servo_joint_t */
#include <stdint.h>
#include <stdbool.h>

/* Safety limits from servo_control.h: SERVO_SAFETY_MIN_DEG / SERVO_SAFETY_MAX_DEG */
#define J5SP_SAFETY_MIN_DEG  SERVO_SAFETY_MIN_DEG
#define J5SP_SAFETY_MAX_DEG  SERVO_SAFETY_MAX_DEG
#define J5SP_VEL_MIN_DEG_S        1.0f
#define J5SP_VEL_MAX_DEG_S      120.0f

/* -----------------------------------------------------------------------
 * Variabili globali definite in j5vr_actuation.c, usate da questo modulo
 * ----------------------------------------------------------------------- */
extern float desired_positions[SERVO_COUNT];
extern float step_accumulator[SERVO_COUNT];

/* -----------------------------------------------------------------------
 * API pubblica — firme identiche a quelle già dichiarate in j5vr_actuation.h
 * (le dichiarazioni qui servono all'unità setpose.c per la compilazione
 *  stand-alone; i chiamanti continuano ad includere j5vr_actuation.h)
 * ----------------------------------------------------------------------- */

/**
 * Tick SETPOSE — chiamata dal RT loop a ogni ciclo (1 kHz).
 * Ritorna true se un movimento SETPOSE è attivo.
 */
bool j5vr_setpose_tick(uint32_t rt_tick);

/**
 * Posa assoluta 6-DOF con velocità percentuale e profilo di moto.
 */
void j5vr_go_setpose(
    uint8_t base_deg,
    uint8_t spalla_deg,
    uint8_t gomito_deg,
    uint8_t yaw_deg,
    uint8_t pitch_deg,
    uint8_t roll_deg,
    uint8_t vel_pct,
    j5_profile_t profile
);

/**
 * Variante a tempo fisso: esegue la posa in esattamente time_ms millisecondi.
 */
void j5vr_go_setpose_time(
    const uint32_t *q_target_deg,
    int             count,
    uint32_t        time_ms,
    j5_profile_t    prof
);

/**
 * Variante a tempo fisso, setpoint sub-degree (HR — high resolution).
 * Usata dal comando UART SETPOSE_T_HR. Stessa semantica di j5vr_go_setpose_time
 * ma accetta float per i target (es. 90.4°), preservando frazione decimale.
 * Combinata con servo_set_angle_f nel tick → fluidità massima delle interpolazioni.
 */
void j5vr_go_setpose_time_f(
    const float    *q_target_deg,
    int             count,
    uint32_t        time_ms,
    j5_profile_t    prof
);

/**
 * Opt-in: al termine del setpose in corso, rilascia (PWM=0) i servo digitali
 * PITCH/ROLL per ridurre il surriscaldamento (usa servo_relax_digital).
 * La flag viene azzerata ad ogni nuovo j5vr_go_setpose/_time: il chiamante deve
 * invocare questa funzione DOPO aver avviato il setpose. Non avvia/completa
 * alcun movimento da sola.
 */
void j5vr_setpose_request_relax_digital_on_finish(void);

#endif /* J5VR_SETPOSE_H */
