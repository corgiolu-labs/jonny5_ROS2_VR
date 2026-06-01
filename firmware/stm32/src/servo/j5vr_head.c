/*
 * j5vr_head.c — Modulo HEAD mode (mode 2): stato, parametri e logica
 *
 * Pipeline HEAD: quaternione visore → errore relativo → EMA + deadzone → servo.
 * L'IMU è usata solo per telemetria (rt_loop.c) e self-test; non viene usata
 * nel path di controllo del polso.
 *
 * JONNY5-4.0
 */

#include "servo/j5vr_head.h"
#include "servo/j5vr_actuation.h"
#include "servo/j5vr_quat_utils.h"
#include <math.h>
#include <stddef.h>

#define J5VR_VEL_MAX_MIN 1.0f
#define J5VR_VEL_MAX_MAX 600.0f
#define J5VR_JOINT_VEL_CAP_MIN 0.0f
#define J5VR_JOINT_VEL_CAP_MAX 600.0f

/* -----------------------------------------------------------------------
 * Istanze statiche (private al modulo)
 * ----------------------------------------------------------------------- */
static j5vr_head_params_t g_head_params;
static j5vr_head_state_t  g_head_state;

static float j5vr_head_apply_soft_deadzone(float value_deg, float deadzone_deg)
{
    const float av = fabsf(value_deg);
    if (av <= deadzone_deg)
    {
        return 0.0f;
    }
    return copysignf(av - deadzone_deg, value_deg);
}

static quat_t j5vr_head_compute_error_quat(const quat_t q_vis,
                                           j5vr_head_params_t *p,
                                           j5vr_head_state_t *st)
{
    (void)p;
    if (!st)
    {
        return (quat_t){ 1.0f, 0.0f, 0.0f, 0.0f };
    }

    if (!st->calibrated)
    {
        st->q_vis_ref  = q_vis;
        st->q_offset   = (quat_t){ 1.0f, 0.0f, 0.0f, 0.0f };
        st->calibrated = true;
    }

    return quat_normalize(quat_multiply(q_vis, quat_conjugate(st->q_vis_ref)));
}

/* -----------------------------------------------------------------------
 * API: init / get
 * ----------------------------------------------------------------------- */

void j5vr_head_init_defaults(j5vr_head_params_t *p, j5vr_head_state_t *st)
{
    if (!p || !st) { return; }

    p->gain_yaw        = 1.0f;
    p->gain_pitch      = 1.0f;
    p->gain_roll       = 1.0f;
    p->alpha_small     = 0.05f;
    p->alpha_large     = 0.35f;
    p->deadzone_deg    = 3.0f;
    p->max_step        = 0.060f;
    p->joy_dz          = 0.10f;
    p->head_sensitivity = 1.0f;
    /* Routing polso — default allineato alla config operativa verificata
     * (routing_config.json), così il polso è corretto gia' al boot e al cambio
     * modalita' senza dipendere dal push SET_VR_PARAMS dal Pi. La patch-bay VR
     * espone la CORRISPONDENZA FISICA REALE asse-per-asse (niente diagonale
     * "cosmetica"): ogni cella dice quale asse del visore guida davvero il servo:
     *   ROLL(robot)  <- YAW(visore)   [sign_yaw = -1: ROLL in opposizione a YAW visore]
     *   PITCH(robot) <- ROLL(visore)
     *   YAW(robot)   <- PITCH(visore) */
    p->sign_yaw        = -1.0f;
    p->sign_pitch      = 1.0f;
    p->sign_roll       = 1.0f;
    p->src_roll        = 0;
    p->src_pitch       = 2;
    p->src_yaw         = 1;
    p->en_roll         = 1;
    p->en_pitch        = 1;
    p->en_yaw          = 1;

    /* Polso: LPF leggero per smorzare il rumore dell'asse visore su PITCH/ROLL. */
    joint_lpf_alpha[SERVO_PITCH] = 0.35f;
    joint_lpf_alpha[SERVO_ROLL]  = 0.35f;

    st->calibrated   = false;
    st->q_offset     = (quat_t){ 1.0f, 0.0f, 0.0f, 0.0f };
    st->q_vis_ref    = (quat_t){ 1.0f, 0.0f, 0.0f, 0.0f };
    st->ema_yaw      = 0.0f;
    st->ema_pitch    = 0.0f;
    st->ema_roll     = 0.0f;
    st->center_roll  = 90.0f;
    st->center_pitch = 90.0f;
    st->center_yaw   = 90.0f;
}

j5vr_head_params_t *j5vr_head_get_params(void)
{
    return &g_head_params;
}

j5vr_head_state_t *j5vr_head_get_state(void)
{
    return &g_head_state;
}

/* -----------------------------------------------------------------------
 * j5vr_reset_head_calib
 * ----------------------------------------------------------------------- */
void j5vr_reset_head_calib(void)
{
    j5vr_head_state_t *st = j5vr_head_get_state();
    st->calibrated  = false;
    st->q_offset    = (quat_t){ 1.0f, 0.0f, 0.0f, 0.0f };
    st->q_vis_ref   = (quat_t){ 1.0f, 0.0f, 0.0f, 0.0f };
    st->ema_yaw     = 0.0f;
    st->ema_pitch   = 0.0f;
    st->ema_roll    = 0.0f;
    st->center_roll  = desired_positions[SERVO_ROLL];
    st->center_pitch = desired_positions[SERVO_PITCH];
    st->center_yaw   = desired_positions[SERVO_YAW];
}

/* -----------------------------------------------------------------------
 * j5vr_set_vr_params
 * -----------------------------------------------------------------------
 * Imposta parametri HEAD a runtime (chiamato da uart_control).
 * ----------------------------------------------------------------------- */
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
                         int vel_base_head, int vel_spalla_head, int vel_gomito_head)
{
    const float RT_LOOP_PERIOD_SEC = 0.001f;
    const float vel_max_clamped =
        (vel_max < J5VR_VEL_MAX_MIN) ? J5VR_VEL_MAX_MIN :
        (vel_max > J5VR_VEL_MAX_MAX) ? J5VR_VEL_MAX_MAX : vel_max;
    const float vel_digital_clamped =
        (vel_digital < J5VR_JOINT_VEL_CAP_MIN) ? J5VR_JOINT_VEL_CAP_MIN :
        (vel_digital > J5VR_JOINT_VEL_CAP_MAX) ? J5VR_JOINT_VEL_CAP_MAX : vel_digital;

#define VEL_CLAMP(_v) \
    (((_v) <= 0) ? vel_digital_clamped : \
     ((_v) > J5VR_JOINT_VEL_CAP_MAX) ? J5VR_JOINT_VEL_CAP_MAX : (float)(_v))
#define VEL_HEAD_CLAMP(_v) \
    (((_v) <= 0) ? 0.0f : \
     ((_v) > J5VR_JOINT_VEL_CAP_MAX) ? J5VR_JOINT_VEL_CAP_MAX : (float)(_v))
#define VEL_ARM_CLAMP(_v) \
    (((_v) < J5VR_JOINT_VEL_CAP_MIN) ? J5VR_JOINT_VEL_CAP_MIN : \
     ((_v) > J5VR_JOINT_VEL_CAP_MAX) ? J5VR_JOINT_VEL_CAP_MAX : (float)(_v))

    j5vr_actuation_set_wrist_vel_head(VEL_HEAD_CLAMP(vel_yaw_head),
                                      VEL_HEAD_CLAMP(vel_pitch_head),
                                      VEL_HEAD_CLAMP(vel_roll_head));
    j5vr_actuation_set_arm_vel_head(VEL_HEAD_CLAMP(vel_base_head),
                                    VEL_HEAD_CLAMP(vel_spalla_head),
                                    VEL_HEAD_CLAMP(vel_gomito_head));

    j5vr_head_params_t *p = j5vr_head_get_params();
    p->gain_yaw        = yaw_gain;
    p->gain_pitch      = pitch_gain;
    p->gain_roll       = roll_gain;
    p->alpha_small     = alpha_small;
    p->alpha_large     = alpha_large;
    p->deadzone_deg    = deadzone_deg;
    p->max_step        = max_step_deg_s * RT_LOOP_PERIOD_SEC;
    current_max_velocity_deg_per_sec = vel_max_clamped;
    joint_max_vel_deg_s[SERVO_PITCH] = VEL_CLAMP(vel_pitch);
    joint_max_vel_deg_s[SERVO_ROLL]  = VEL_CLAMP(vel_roll);
    joint_max_vel_deg_s[SERVO_YAW]   = VEL_CLAMP(vel_yaw);
    joint_max_vel_deg_s[SERVO_BASE]  = VEL_ARM_CLAMP(vel_base);
    joint_max_vel_deg_s[SERVO_SPALLA]= VEL_ARM_CLAMP(vel_spalla);
    joint_max_vel_deg_s[SERVO_GOMITO]= VEL_ARM_CLAMP(vel_gomito);
    /* lpf_pitch/lpf_roll legacy: ignorati, il polso usa 0.35 fisso su PITCH/ROLL. */
    (void)lpf_pitch;
    (void)lpf_roll;
    p->joy_dz = joy_dz;
    {
        const float s = (head_sensitivity < 0.25f) ? 0.25f :
                        (head_sensitivity > 4.0f)  ? 4.0f  : head_sensitivity;
        p->head_sensitivity = s;
    }
    p->sign_yaw   = (sign_yaw   >= 0.0f) ?  1.0f : -1.0f;
    p->sign_pitch = (sign_pitch >= 0.0f) ?  1.0f : -1.0f;
    p->sign_roll  = (sign_roll  >= 0.0f) ?  1.0f : -1.0f;
    if (src_roll  >= 0 && src_roll  < 3) { p->src_roll  = src_roll;  }
    if (src_pitch >= 0 && src_pitch < 3) { p->src_pitch = src_pitch; }
    if (src_yaw   >= 0 && src_yaw   < 3) { p->src_yaw   = src_yaw;   }
    p->en_roll  = en_roll  ? 1 : 0;
    p->en_pitch = en_pitch ? 1 : 0;
    p->en_yaw   = en_yaw   ? 1 : 0;

#undef VEL_CLAMP
#undef VEL_HEAD_CLAMP
#undef VEL_ARM_CLAMP
}

/* -----------------------------------------------------------------------
 * j5vr_apply_head_tracking
 * -----------------------------------------------------------------------
 * Loop principale HEAD mode (mode 2):
 *   1) Quaternione visore → errore rispetto alla posa di calibrazione
 *   2) Errore → yaw/pitch/roll con segni configurabili
 *   3) Filtro EMA adattivo + deadzone
 *   4) Controllo diretto desired_positions[] → servo
 * ----------------------------------------------------------------------- */
void j5vr_apply_head_tracking(const struct j5vr_state *s)
{
    j5vr_head_params_t *p  = j5vr_head_get_params();
    j5vr_head_state_t  *st = j5vr_head_get_state();

    if (s == NULL)
    {
        return;
    }

    j5vr_actuation_update_max_vel_buttons(s);

    quat_t q_vis = quat_normalize(quat_from_j5vr(s));
    quat_t q_err = j5vr_head_compute_error_quat(q_vis, p, st);

    float yaw_err_deg = 0.0f, pitch_err_deg = 0.0f, roll_err_deg = 0.0f;
    quat_to_twist_ypr_deg(q_err.w, q_err.x, q_err.y, q_err.z,
                          &yaw_err_deg, &pitch_err_deg, &roll_err_deg);
    yaw_err_deg   *= p->sign_yaw;
    pitch_err_deg *= p->sign_pitch;
    roll_err_deg  *= p->sign_roll;

    const float deadzone = p->deadzone_deg;
    const float g_small  = p->alpha_small;
    const float g_large  = p->alpha_large;

    const float g_y = (fabsf(yaw_err_deg)   < deadzone) ? g_small : g_large;
    const float g_p = (fabsf(pitch_err_deg) < deadzone) ? g_small : g_large;
    const float g_r = (fabsf(roll_err_deg)  < deadzone) ? g_small : g_large;

    st->ema_yaw   = (1.0f - g_y) * st->ema_yaw   + g_y * yaw_err_deg;
    st->ema_pitch = (1.0f - g_p) * st->ema_pitch + g_p * pitch_err_deg;
    st->ema_roll  = (1.0f - g_r) * st->ema_roll  + g_r * roll_err_deg;

    const float ye = j5vr_head_apply_soft_deadzone(st->ema_yaw,   deadzone);
    const float pe = j5vr_head_apply_soft_deadzone(st->ema_pitch, deadzone);
    const float re = j5vr_head_apply_soft_deadzone(st->ema_roll,  deadzone);

    const float err_src[3] = { ye, pe, re };
    const int sr = (p->src_roll  >= 0 && p->src_roll  < 3) ? p->src_roll  : 2;
    const int sp = (p->src_pitch >= 0 && p->src_pitch < 3) ? p->src_pitch : 1;
    const int sy = (p->src_yaw   >= 0 && p->src_yaw   < 3) ? p->src_yaw   : 0;

    const float sens = p->head_sensitivity;

#define CLAMP_F(_v, _lo, _hi) (((_v) < (_lo)) ? (_lo) : (((_v) > (_hi)) ? (_hi) : (_v)))

    if (p->en_roll)
    {
        const float ctrl = err_src[sr];
        if (fabsf(ctrl) > 1e-6f)
        {
            float target = st->center_roll + p->gain_roll * (sens * ctrl);
            desired_positions[SERVO_ROLL] = CLAMP_F(target,
                joint_min_deg[SERVO_ROLL], joint_max_deg[SERVO_ROLL]);
        }
    }
    if (p->en_pitch)
    {
        const float ctrl = err_src[sp];
        if (fabsf(ctrl) > 1e-6f)
        {
            float target = st->center_pitch + p->gain_pitch * (sens * ctrl);
            desired_positions[SERVO_PITCH] = CLAMP_F(target,
                joint_min_deg[SERVO_PITCH], joint_max_deg[SERVO_PITCH]);
        }
    }
    if (p->en_yaw)
    {
        const float ctrl = err_src[sy];
        if (fabsf(ctrl) > 1e-6f)
        {
            float target = st->center_yaw + p->gain_yaw * (sens * ctrl);
            desired_positions[SERVO_YAW] = CLAMP_F(target,
                joint_min_deg[SERVO_YAW], joint_max_deg[SERVO_YAW]);
        }
    }

#undef CLAMP_F

    j5vr_actuation_apply_desired(s);
}

/* -----------------------------------------------------------------------
 * j5vr_apply_head_orientation_only
 * -----------------------------------------------------------------------
 * Aggiorna solo i target testa (YAW/PITCH/ROLL) senza applicare subito
 * ai servo. Usata da HYBRID per combinare testa HEAD con braccio MANUAL.
 * ----------------------------------------------------------------------- */
void j5vr_apply_head_orientation_only(const struct j5vr_state *s)
{
    j5vr_head_params_t *p  = j5vr_head_get_params();
    j5vr_head_state_t  *st = j5vr_head_get_state();

    if (s == NULL)
    {
        return;
    }

    quat_t q_vis = quat_normalize(quat_from_j5vr(s));
    quat_t q_err = j5vr_head_compute_error_quat(q_vis, p, st);

    float yaw_err_deg = 0.0f, pitch_err_deg = 0.0f, roll_err_deg = 0.0f;
    quat_to_twist_ypr_deg(q_err.w, q_err.x, q_err.y, q_err.z,
                          &yaw_err_deg, &pitch_err_deg, &roll_err_deg);
    yaw_err_deg   *= p->sign_yaw;
    pitch_err_deg *= p->sign_pitch;
    roll_err_deg  *= p->sign_roll;

    const float deadzone = p->deadzone_deg;
    const float g_small  = p->alpha_small;
    const float g_large  = p->alpha_large;

    const float g_y = (fabsf(yaw_err_deg)   < deadzone) ? g_small : g_large;
    const float g_p = (fabsf(pitch_err_deg) < deadzone) ? g_small : g_large;
    const float g_r = (fabsf(roll_err_deg)  < deadzone) ? g_small : g_large;

    st->ema_yaw   = (1.0f - g_y) * st->ema_yaw   + g_y * yaw_err_deg;
    st->ema_pitch = (1.0f - g_p) * st->ema_pitch + g_p * pitch_err_deg;
    st->ema_roll  = (1.0f - g_r) * st->ema_roll  + g_r * roll_err_deg;

    const float ye = j5vr_head_apply_soft_deadzone(st->ema_yaw,   deadzone);
    const float pe = j5vr_head_apply_soft_deadzone(st->ema_pitch, deadzone);
    const float re = j5vr_head_apply_soft_deadzone(st->ema_roll,  deadzone);

    const float err_src[3] = { ye, pe, re };
    const int sr = (p->src_roll  >= 0 && p->src_roll  < 3) ? p->src_roll  : 2;
    const int sp = (p->src_pitch >= 0 && p->src_pitch < 3) ? p->src_pitch : 1;
    const int sy = (p->src_yaw   >= 0 && p->src_yaw   < 3) ? p->src_yaw   : 0;

    const float sens = p->head_sensitivity;

#define CLAMP_F(_v, _lo, _hi) (((_v) < (_lo)) ? (_lo) : (((_v) > (_hi)) ? (_hi) : (_v)))

    if (p->en_roll)
    {
        const float ctrl = err_src[sr];
        if (fabsf(ctrl) > 1e-6f)
        {
            float target = st->center_roll + p->gain_roll * (sens * ctrl);
            desired_positions[SERVO_ROLL] = CLAMP_F(target,
                joint_min_deg[SERVO_ROLL], joint_max_deg[SERVO_ROLL]);
        }
    }
    if (p->en_pitch)
    {
        const float ctrl = err_src[sp];
        if (fabsf(ctrl) > 1e-6f)
        {
            float target = st->center_pitch + p->gain_pitch * (sens * ctrl);
            desired_positions[SERVO_PITCH] = CLAMP_F(target,
                joint_min_deg[SERVO_PITCH], joint_max_deg[SERVO_PITCH]);
        }
    }
    if (p->en_yaw)
    {
        const float ctrl = err_src[sy];
        if (fabsf(ctrl) > 1e-6f)
        {
            float target = st->center_yaw + p->gain_yaw * (sens * ctrl);
            desired_positions[SERVO_YAW] = CLAMP_F(target,
                joint_min_deg[SERVO_YAW], joint_max_deg[SERVO_YAW]);
        }
    }

#undef CLAMP_F
}
