/*
 * j5vr_head.h — Modulo HEAD mode (mode 2): stato, parametri e logica
 *
 * Pipeline HEAD: quaternione visore → errore → EMA + deadzone → servo.
 * L'IMU è usata solo per telemetria e self-test (rt_loop.c / imu.c).
 *
 * JONNY5-4.0
 */

#ifndef J5VR_HEAD_H
#define J5VR_HEAD_H

#include "servo/j5vr_quat_utils.h"   /* quat_t */
#include "servo/servo_control.h"     /* SERVO_COUNT, servo_joint_t */
#include "spi/j5_protocol.h"         /* struct j5vr_state */
#include <stdbool.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Parametri configurabili HEAD loop (modificabili via SET_VR_PARAMS)
 * ----------------------------------------------------------------------- */
typedef struct {
    float gain_yaw;
    float gain_pitch;
    float gain_roll;
    float alpha_small;
    float alpha_large;
    float deadzone_deg;
    float max_step;
    float joy_dz;
    float head_sensitivity;
    float sign_yaw;
    float sign_pitch;
    float sign_roll;
    int   src_roll;
    int   src_pitch;
    int   src_yaw;
    int   en_roll;
    int   en_pitch;
    int   en_yaw;
} j5vr_head_params_t;

/* -----------------------------------------------------------------------
 * Stato runtime HEAD loop
 * ----------------------------------------------------------------------- */
typedef struct {
    bool    calibrated;
    quat_t  q_offset;
    quat_t  q_vis_ref;
    float   ema_yaw;
    float   ema_pitch;
    float   ema_roll;
    float   center_roll;
    float   center_pitch;
    float   center_yaw;
} j5vr_head_state_t;

/* -----------------------------------------------------------------------
 * Variabili di actuation.c usate dalle funzioni HEAD
 * ----------------------------------------------------------------------- */
extern float desired_positions[SERVO_COUNT];
extern float joint_min_deg[SERVO_COUNT];
extern float joint_max_deg[SERVO_COUNT];
extern float joint_max_vel_deg_s[SERVO_COUNT];
extern float joint_lpf_alpha[SERVO_COUNT];
extern float current_max_velocity_deg_per_sec;

/* -----------------------------------------------------------------------
 * Funzioni helper di actuation.c usate da j5vr_head.c
 * ----------------------------------------------------------------------- */
struct j5vr_state;

void j5vr_actuation_update_max_vel_buttons(const struct j5vr_state *s);
void j5vr_actuation_apply_desired(const struct j5vr_state *j5vr);
void j5vr_actuation_set_wrist_vel_head(float yaw_deg_s, float pitch_deg_s, float roll_deg_s);
void j5vr_actuation_set_arm_vel_head(float base_deg_s, float spalla_deg_s, float gomito_deg_s);

/* -----------------------------------------------------------------------
 * API stato e parametri HEAD
 * ----------------------------------------------------------------------- */
void j5vr_head_init_defaults(j5vr_head_params_t *params, j5vr_head_state_t *st);
j5vr_head_params_t *j5vr_head_get_params(void);
j5vr_head_state_t  *j5vr_head_get_state(void);

/* -----------------------------------------------------------------------
 * API logica HEAD
 * ----------------------------------------------------------------------- */

/** Loop HEAD mode: quaternione visore → desired_positions[] → servo. */
void j5vr_apply_head_tracking(const struct j5vr_state *s);

/** Aggiorna solo giunti testa (YAW/PITCH/ROLL) senza applicare servo.
 *  Usata da HYBRID: testa HEAD + braccio MANUAL. */
void j5vr_apply_head_orientation_only(const struct j5vr_state *s);

/** Reset calibrazione HEAD: azzera q_offset, EMA, cattura centro corrente. */
void j5vr_reset_head_calib(void);

/** Imposta parametri VR a runtime (chiamato da uart_control). */
void j5vr_set_vr_params(float yaw_gain, float pitch_gain, float roll_gain,
                         float alpha_small, float alpha_large, float deadzone_deg,
                         float max_step_deg_s, float vel_max, float vel_digital,
                         float lpf_pitch, float lpf_roll, float joy_dz,
                         float head_sensitivity,
                         float sign_yaw, float sign_pitch, float sign_roll,
                         int src_roll, int src_pitch, int src_yaw,
                         int en_roll, int en_pitch, int en_yaw,
                         int vel_base, int vel_spalla, int vel_gomito,
                         int vel_yaw, int vel_pitch, int vel_roll,
                         int vel_yaw_head, int vel_pitch_head, int vel_roll_head,
                         int vel_base_head, int vel_spalla_head, int vel_gomito_head);

#endif /* J5VR_HEAD_H */
