/*
 * j5vr_actuation.h — API pubblica del modulo j5vr_actuation
 *
 * Espone:
 *   - Tipi: j5_profile_t, servo_setpoint_t
 *   - Inizializzazione e utility servo
 *   - Modalità MANUAL e HYBRID (mode 1 e 3)
 *   - Modulo HEAD (via include di j5vr_head.h per retrocompatibilità
 *     dei chiamanti uart_control.c e rt_loop.c)
 *   - Modulo SETPOSE (j5vr_setpose_tick, j5vr_go_setpose, j5vr_go_setpose_time)
 */

#ifndef J5VR_ACTUATION_H
#define J5VR_ACTUATION_H

#include "spi/j5_protocol.h"
#include <stdbool.h>
#include <stdint.h>

/* =========================================================
 * TIPI PUBBLICI
 * ========================================================= */

/**
 * j5_profile_t — profilo di pianificazione per j5vr_go_setpose().
 *
 *  RTR3: Hermite cubico          3τ²−2τ³       (C¹, S-curve minima)
 *  RTR5: Polinomio quintico      10τ³−15τ⁴+6τ⁵ (C², minimum-jerk)
 *  BB  : Bang-Bang               2τ² | 1−2(1−τ)²  (accel ±cost. a tratti, baseline)
 *  BCB : Raised-cosine           (1−cos(πτ))/2  (C², crociera dolce)
 */
typedef enum {
    J5_PROFILE_RTR3 = 0,
    J5_PROFILE_RTR5 = 1,
    J5_PROFILE_BB   = 2,
    J5_PROFILE_BCB  = 3,
} j5_profile_t;

/**
 * servo_setpoint_t — setpoint angolare a 6 giunti [0–180 gradi].
 * Prodotta da j5vr_to_servo_setpoint() come validazione input nel RT loop.
 */
typedef struct {
    uint8_t base;
    uint8_t spalla;
    uint8_t gomito;
    uint8_t roll;
    uint8_t pitch;
    uint8_t yaw;
} servo_setpoint_t;

/* =========================================================
 * DIAGNOSTICA RUNTIME (extern — letta da uart_control.c)
 * ========================================================= */

extern volatile uint32_t g_j5vr_apply_incr_calls;
extern volatile int16_t  g_j5vr_last_inc_mdeg_yaw;
extern volatile int16_t  g_j5vr_last_inc_mdeg_pitch;
extern volatile int16_t  g_j5vr_last_inc_mdeg_spalla;
extern volatile int16_t  g_j5vr_last_inc_mdeg_gomito;

/* =========================================================
 * API — inizializzazione e utility
 * ========================================================= */

/** Inizializza desired_positions[], velocità max e parametri HEAD. */
void j5vr_actuation_init(void);

/** Porta tutti i servo alla posa HOME tramite SETPOSE (RTR5, non bloccante). */
void j5vr_center_all_servos(void);

/** Porta il robot in VR/Teleop Pose tramite SETPOSE (RTR5, non bloccante). */
void j5vr_go_teleop_pose(void);

/** Aggiorna i limiti per-giunto a runtime. Clamp automatico ai limiti fisici. */
void j5vr_set_joint_limits(int joint, float lim_min, float lim_max);

/**
 * Converte stato J5VR in setpoint servo.
 * Usato dal RT loop come guard di validazione input prima di MANUAL.
 */
bool j5vr_to_servo_setpoint(const struct j5vr_state *j5vr, servo_setpoint_t *setpoint);

/* =========================================================
 * API — modalità di teleop (chiamate da rt_loop.c)
 * ========================================================= */

/** MANUAL mode (1): incrementa desired_positions[] dai joystick/pulsanti. */
bool j5vr_apply_setpoint_incremental(const struct j5vr_state *j5vr, bool movement_allowed);

/** HYBRID mode (3): testa open-loop + joystick. Sperimentale. */
void j5vr_apply_hybrid(const struct j5vr_state *s);

/**
 * IK MODE (5): B/S/G da estensione J5VR (marker 'I', bit valid in flags);
 * Y/P/R dalla stessa pipeline HEAD del mode 2 (quaternione visore + IMU).
 */
void j5vr_apply_mode5_arm_head(const struct j5vr_state *s);

/** Target 6-DOF via frame SPI J5IK (legacy / diagnostica; non usato dal path IK mode=5 su J5VR). */
void j5ik_apply_direct_target(const struct j5ik_state *ik);

/* HEAD mode (2): j5vr_apply_head_closed_loop, j5vr_reset_head_calib,
 * j5vr_set_vr_params — dichiarate in servo/j5vr_head.h.
 * Incluso qui per retrocompatibilità di uart_control.c e rt_loop.c. */
#include "servo/j5vr_head.h"

/* =========================================================
 * API — SETPOSE (mode 4, implementata in j5vr_setpose.c)
 * ========================================================= */

/**
 * j5vr_go_setpose — avvia una posa assoluta a 6 giunti.
 *
 *   base_deg .. roll_deg : angoli target [0, 180] deg (clamp automatico)
 *   vel_pct              : velocità [1, 100]% (0 → 10 come minimo sicuro)
 *   profile              : profilo di pianificazione (j5_profile_t)
 *
 * Non bloccante: aggiorna g_setpose_state e ritorna subito.
 * Il RT loop chiama j5vr_setpose_tick() ogni ms per eseguire la traiettoria.
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
 * j5vr_go_setpose_time — variante a durata fissa.
 *
 *   q_target_deg : array di 6 angoli [deg], ordine B S G Y P R
 *   count        : deve essere SERVO_COUNT (6)
 *   time_ms      : durata totale [20, 60000] ms
 *   prof         : profilo di pianificazione
 */
void j5vr_go_setpose_time(
    const uint32_t *q_target_deg,
    int             count,
    uint32_t        time_ms,
    j5_profile_t    prof
);

/**
 * j5vr_setpose_tick — tick SETPOSE, chiamato dal RT loop a 1 kHz.
 *
 * Ritorna true  se un SETPOSE è attivo (il RT loop non deve eseguire VR).
 * Ritorna false se nessun SETPOSE è in corso (pipeline VR normale).
 */
bool j5vr_setpose_tick(uint32_t rt_tick);

#endif /* J5VR_ACTUATION_H */
