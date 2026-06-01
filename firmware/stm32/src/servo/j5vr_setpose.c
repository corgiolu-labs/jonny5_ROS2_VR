/*
 * j5vr_setpose.c — Traiettorie SETPOSE / SETPOSE_T
 *
 * Implementazione estratta da j5vr_actuation.c.
 * Contiene:
 *   - Profili di moto: j5_profile_rtr3/rtr5/bb/bcb
 *   - Stato interno g_setpose_state (static, privato)
 *   - j5vr_setpose_tick  (tick RT loop, 1 kHz)
 *   - j5vr_go_setpose    (comando posa con vel%)
 *   - j5vr_go_setpose_time (comando posa con durata ms)
 *
 * Dipende da:
 *   - desired_positions[] e step_accumulator[] (extern, definiti in actuation.c)
 *   - g_rt_loop_ticks (extern in core/rt_loop.h)
 *   - servo_get_angle / servo_set_angle (servo/servo_control.h)
 *   - uart_send_unsolicited (uart/uart_control.h)
 *
 * JONNY5-4.0 — Step 3.3 refactor (2026-02-25)
 */

#include "servo/j5vr_setpose.h"
#include "servo/servo_control.h"
#include "core/rt_loop.h"
#include "uart/uart_control.h"
#include <zephyr/sys/printk.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Limiti runtime autorevoli condivisi con MANUAL/HEAD/HYBRID/ASSIST.
 * Definiti in j5vr_actuation.c e aggiornati da SET_JOINT_LIMITS. */
extern float joint_min_deg[SERVO_COUNT];
extern float joint_max_deg[SERVO_COUNT];

/* -----------------------------------------------------------------------
 * Helper: angolo fisico polso (clamp 0-180, arrotondamento)
 * Duplicato static da j5vr_actuation.c (3 righe, nessuno stato).
 * ----------------------------------------------------------------------- */
static uint8_t sp_wrist_physical_angle(float logical_deg)
{
    if (logical_deg < 0.0f)   logical_deg = 0.0f;
    if (logical_deg > 180.0f) logical_deg = 180.0f;
    return (uint8_t)(int)(logical_deg + 0.5f);
}

/* Versione float — clamp [0, 180] ma senza arrotondamento. Usata dal tick a 1 kHz. */
static float sp_wrist_physical_angle_f(float logical_deg)
{
    if (logical_deg < 0.0f)   logical_deg = 0.0f;
    if (logical_deg > 180.0f) logical_deg = 180.0f;
    return logical_deg;
}

static float sp_clamp_joint_runtime_deg(int joint, float angle_deg)
{
    float runtime_min = J5SP_SAFETY_MIN_DEG;
    float runtime_max = J5SP_SAFETY_MAX_DEG;

    if (joint >= 0 && joint < SERVO_COUNT)
    {
        runtime_min = joint_min_deg[joint];
        runtime_max = joint_max_deg[joint];

        if (runtime_min < J5SP_SAFETY_MIN_DEG) { runtime_min = J5SP_SAFETY_MIN_DEG; }
        if (runtime_min > J5SP_SAFETY_MAX_DEG) { runtime_min = J5SP_SAFETY_MAX_DEG; }
        if (runtime_max < J5SP_SAFETY_MIN_DEG) { runtime_max = J5SP_SAFETY_MIN_DEG; }
        if (runtime_max > J5SP_SAFETY_MAX_DEG) { runtime_max = J5SP_SAFETY_MAX_DEG; }
        if (runtime_min > runtime_max)
        {
            runtime_min = J5SP_SAFETY_MIN_DEG;
            runtime_max = J5SP_SAFETY_MAX_DEG;
        }
    }

    if (angle_deg < runtime_min) { angle_deg = runtime_min; }
    if (angle_deg > runtime_max) { angle_deg = runtime_max; }
    if (angle_deg < J5SP_SAFETY_MIN_DEG) { angle_deg = J5SP_SAFETY_MIN_DEG; }
    if (angle_deg > J5SP_SAFETY_MAX_DEG) { angle_deg = J5SP_SAFETY_MAX_DEG; }
    return angle_deg;
}

/* -----------------------------------------------------------------------
 * Profili di moto normalizzati: tau ∈ [0,1] → s ∈ [0,1]
 * ----------------------------------------------------------------------- */

/* RTR3 — Hermite cubico S-curve (C¹, derivata zero agli estremi) */
static float j5_profile_rtr3(float tau)
{
    return 3.0f*tau*tau - 2.0f*tau*tau*tau;
}

/* RTR5 — Quintico minimum-jerk (C²) */
static float j5_profile_rtr5(float tau)
{
    float t2 = tau * tau;
    float t3 = t2 * tau;
    float t4 = t3 * tau;
    float t5 = t4 * tau;
    return 10.0f*t3 - 15.0f*t4 + 6.0f*t5;
}

/* BCB — raised-cosine s(τ) = (1-cos(π·τ))/2 (C², nessun punto di giunzione) */
static float j5_profile_bcb(float tau)
{
    if (tau <= 0.0f) return 0.0f;
    if (tau >= 1.0f) return 1.0f;
    return (1.0f - cosf(3.14159265f * tau)) * 0.5f;
}

/* BB — Bang-Bang: accelerazione costante a tratti (time-optimal a vincolo a_max).
 *   τ ∈ [0, 0.5]: s(τ) = 2·τ²            (acc = +4 normalizzata)
 *   τ ∈ [0.5, 1]: s(τ) = 1 − 2·(1−τ)²    (acc = −4 normalizzata)
 *   v_picco = 2 a τ=0.5; jerk infinito ai 3 punti di commutazione (0, 0.5, 1).
 *   Profilo "rude" — usato come baseline di confronto vs RTR3/RTR5/BCB. */
static float j5_profile_bb(float tau)
{
    if (tau <= 0.0f) return 0.0f;
    if (tau >= 1.0f) return 1.0f;
    if (tau <= 0.5f) return 2.0f * tau * tau;
    float u = 1.0f - tau;
    return 1.0f - 2.0f * u * u;
}

/* -----------------------------------------------------------------------
 * Stato interno SETPOSE (privato a questo modulo)
 * ----------------------------------------------------------------------- */
typedef struct {
    bool     active;
    uint32_t start_tick;
    uint32_t duration_ticks;
    j5_profile_t profile;
    float    q_start[SERVO_COUNT];
    float    q_target[SERVO_COUNT];
    /* Telemetria esecuzione */
    float    prev_angle[SERVO_COUNT];
    float    prev_vel[SERVO_COUNT];
    float    max_velocity_deg_s;
    float    max_accel_deg_s2;
    /* Opt-in: rilascia i servo digitali (PITCH/ROLL) quando il trajectory
     * finisce. Usato dal comando HOME per evitare surriscaldamento dei due
     * servo più stressati al termine del centraggio. */
    bool     relax_digital_on_finish;
    /* Wake-up tap pre-trajectory per servo SDS1601 su PITCH/ROLL.
     * Compensa la deadband + stiction: prima dell'interpolazione vera,
     * applichiamo per ~100 ms un setpoint preload spostato di 0.8° verso
     * il target → vince l'attrito statico. Quando inizia il trajectory
     * vero, il servo è già "agganciato" e segue il profilo smooth. */
    bool     warming_up;
    uint32_t warm_until_tick;
    float    warm_preload[SERVO_COUNT];   /* setpoint preload (solo PITCH/ROLL usati) */
    bool     warm_pending[SERVO_COUNT];   /* true se quel giunto deve fare wake-up */
} j5_setpose_state_t;

static j5_setpose_state_t g_setpose_state = { .active = false };

/* Wake-up tap config — DISABILITATO (peggiora reattività globale e
 * non risolve il problema PITCH SDS1601). Codice mantenuto inattivo
 * tramite J5SP_WAKEUP_ENABLED=0 per facile riattivazione futura. */
#define J5SP_WAKEUP_ENABLED          0       /* 0 = OFF (ripristino comportamento originale) */
#define J5SP_WAKEUP_DURATION_TICKS  100U
#define J5SP_WAKEUP_DELTA_DEG        0.8f
#define J5SP_WAKEUP_MIN_TARGET_DEG   1.5f
#define J5SP_WAKEUP_DEBUG            0

/* Imposta lo stato di wake-up tap per i servo digitali del polso (PITCH+ROLL).
 * Da chiamare PRIMA di settare q_target nello state. La logica:
 *   - Solo per giunti SERVO_PITCH e SERVO_ROLL (servo SDS1601 con deadband).
 *   - Solo se |q_target - q_start_pre| > J5SP_WAKEUP_MIN_TARGET_DEG.
 *   - Genera un preload = q_start + sign(delta) × J5SP_WAKEUP_DELTA_DEG,
 *     clampato ai limiti runtime.
 * Restituisce true se almeno un giunto richiede warmup. */
static bool sp_setup_wakeup_tap(const float *q_target_real)
{
#if J5SP_WAKEUP_ENABLED == 0
    (void)q_target_real;
    return false;
#else
    bool need = false;
    for (int i = 0; i < SERVO_COUNT; i++) {
        g_setpose_state.warm_pending[i] = false;
        g_setpose_state.warm_preload[i] = desired_positions[i];
    }
    /* Wake-up solo per i giunti del polso con servo SDS1601 (PITCH e ROLL).
     * BASE/SPALLA/GOMITO/YAW sono servo analogici/digitali con deadband
     * trascurabile in modalità PWM standard. */
    const int wrist_joints[2] = { SERVO_PITCH, SERVO_ROLL };
    for (int k = 0; k < 2; k++) {
        const int i = wrist_joints[k];
        const float dq = q_target_real[i] - desired_positions[i];
        const float dq_abs = (dq < 0.0f) ? -dq : dq;
        if (dq_abs < J5SP_WAKEUP_MIN_TARGET_DEG) { continue; }
        const float sign = (dq > 0.0f) ? 1.0f : -1.0f;
        float preload = desired_positions[i] + sign * J5SP_WAKEUP_DELTA_DEG;
        preload = sp_clamp_joint_runtime_deg(i, preload);
        g_setpose_state.warm_preload[i] = preload;
        g_setpose_state.warm_pending[i] = true;
        need = true;
#if J5SP_WAKEUP_DEBUG
        printk("[WAKE] joint=%d cur=%d.%02d tgt=%d.%02d dq=%d.%02d preload=%d.%02d\n",
               i,
               (int)desired_positions[i], (int)((desired_positions[i] - (int)desired_positions[i]) * 100.0f),
               (int)q_target_real[i],     (int)((q_target_real[i] - (int)q_target_real[i]) * 100.0f),
               (int)dq,                   (int)((dq - (int)dq) * 100.0f),
               (int)preload,              (int)((preload - (int)preload) * 100.0f));
#endif
    }
#if J5SP_WAKEUP_DEBUG
    if (need) printk("[WAKE] active for %u ms\n", (unsigned)J5SP_WAKEUP_DURATION_TICKS);
#endif
    return need;
#endif
}

/* -----------------------------------------------------------------------
 * j5vr_setpose_tick — chiamato dal RT loop a ogni ciclo (1 kHz)
 * ----------------------------------------------------------------------- */
bool j5vr_setpose_tick(uint32_t rt_tick)
{
    if (!g_setpose_state.active)
    {
        return false;
    }

    /* Fase WAKE-UP TAP — applichiamo il preload sui servo PITCH/ROLL per
     * vincere stiction + deadband del SDS1601. Dura J5SP_WAKEUP_DURATION_TICKS
     * ms, dopo i quali resettiamo q_start alle desired_positions correnti
     * (= dove il servo è arrivato grazie al preload) e iniziamo trajectory. */
    if (g_setpose_state.warming_up)
    {
        if (rt_tick < g_setpose_state.warm_until_tick)
        {
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (!g_setpose_state.warm_pending[i]) { continue; }
                const float pl = g_setpose_state.warm_preload[i];
                desired_positions[i] = pl;
                servo_set_angle_f((servo_joint_t)i, pl);
                step_accumulator[i] = 0.0f;
            }
            return true;   /* trajectory non parte finché warm-up non finisce */
        }
        /* Warmup terminato: aggiorno q_start e prev_angle ai valori correnti
         * (il PITCH/ROLL si è spostato di ~0.8°). start_tick = ora. */
        g_setpose_state.warming_up = false;
        g_setpose_state.start_tick = rt_tick;
        for (int i = 0; i < SERVO_COUNT; i++) {
            const float now_q = sp_clamp_joint_runtime_deg(i, desired_positions[i]);
            g_setpose_state.q_start[i]    = now_q;
            g_setpose_state.prev_angle[i] = now_q;
            g_setpose_state.prev_vel[i]   = 0.0f;
        }
    }

    uint32_t dt = rt_tick - g_setpose_state.start_tick;
    bool     finished = false;
    float    tau;
    if (dt >= g_setpose_state.duration_ticks)
    {
        tau      = 1.0f;
        finished = true;
        g_setpose_state.active = false;
    }
    else
    {
        tau = (float)dt / (float)g_setpose_state.duration_ticks;
    }

    float s;
    switch (g_setpose_state.profile)
    {
        case J5_PROFILE_RTR3:  s = j5_profile_rtr3(tau); break;
        case J5_PROFILE_RTR5:  s = j5_profile_rtr5(tau); break;
        case J5_PROFILE_BB:    s = j5_profile_bb(tau);   break;
        case J5_PROFILE_BCB:   s = j5_profile_bcb(tau);  break;
        default:               s = j5_profile_rtr3(tau); break;
    }

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        const float interpolated = g_setpose_state.q_start[i]
                                 + s * (g_setpose_state.q_target[i]
                                        - g_setpose_state.q_start[i]);
        desired_positions[i] = sp_clamp_joint_runtime_deg(i, interpolated);
    }

    /* Applica direttamente la posizione interpolata ai servo, bypassando
     * il velocity cap di apply_desired_positions_to_servos().
     * La pipeline SETPOSE ha il proprio controllo temporale tramite duration_ticks. */
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        float new_pos = sp_clamp_joint_runtime_deg(i, desired_positions[i]);
        desired_positions[i] = new_pos;

        /* Telemetria: derivata su desired_positions[] (float, alta risoluzione) */
        {
            float vel = (new_pos - g_setpose_state.prev_angle[i]) * 1000.0f;
            float acc = (vel     - g_setpose_state.prev_vel[i])   * 1000.0f;

            float abs_vel = vel < 0.0f ? -vel : vel;
            float abs_acc = acc < 0.0f ? -acc : acc;

            if (abs_vel > g_setpose_state.max_velocity_deg_s)
            {
                g_setpose_state.max_velocity_deg_s = abs_vel;
            }
            if (abs_acc > g_setpose_state.max_accel_deg_s2)
            {
                g_setpose_state.max_accel_deg_s2 = abs_acc;
            }

            g_setpose_state.prev_angle[i] = new_pos;
            g_setpose_state.prev_vel[i]   = vel;
        }

        /* Risoluzione sub-degree: usiamo direttamente new_pos (float) come setpoint
         * via servo_set_angle_f. Niente arrotondamento a 1° → eliminiamo gli "scatti"
         * interpolativi soprattutto su PITCH/ROLL (servo digitali HPS-0127) durante
         * movimenti di pochi gradi. Il PWM cambia di ~0.012 us/tick = micro-step.
         *
         * Update sempre, anche se l'angolo intero è invariato: il pulse PWM cambia
         * comunque di frazioni di µs durante l'interpolazione. Ed è necessario per
         * riagganciare i giunti dopo STOP/SAFE (last_pulse_us = 0). */
        float send_cmd_f = (i == SERVO_YAW || i == SERVO_PITCH || i == SERVO_ROLL)
                         ? sp_wrist_physical_angle_f(new_pos)
                         : new_pos;
        servo_set_angle_f((servo_joint_t)i, send_cmd_f);

        /* Reset accumulatore: nessun residuo deve passare alla pipeline VR */
        step_accumulator[i] = 0.0f;
    }

    /* Invio telemetria finale via UART non-solicitato */
    if (finished)
    {
        uint32_t elapsed_ms = dt;
        char msg[72];
        snprintf(msg, sizeof(msg),
                 "SETPOSE_DONE time_ms=%u vel_max=%.1f acc_max=%.1f",
                 (unsigned)elapsed_ms,
                 (double)g_setpose_state.max_velocity_deg_s,
                 (double)g_setpose_state.max_accel_deg_s2);
        uart_send_unsolicited(msg);

        /* Post-completion relax, opt-in. Only HOME sets this flag so SETPOSE /
         * SETPOSE_T / TELEOPPOSE / PARK keep their PWM engaged as before. */
        if (g_setpose_state.relax_digital_on_finish)
        {
            g_setpose_state.relax_digital_on_finish = false;
            servo_relax_digital();
            uart_send_unsolicited("RELAX_DIGITAL pitch roll");
        }
    }

    return true;
}

/* -----------------------------------------------------------------------
 * j5vr_go_setpose — posa assoluta 6-DOF con vel% e profilo di moto
 * ----------------------------------------------------------------------- */
void j5vr_go_setpose(
    uint8_t base_deg,
    uint8_t spalla_deg,
    uint8_t gomito_deg,
    uint8_t yaw_deg,
    uint8_t pitch_deg,
    uint8_t roll_deg,
    uint8_t vel_pct,
    j5_profile_t profile
)
{
    uint8_t vp = vel_pct;
    if (vp == 0U)  { vp = 10U; }
    if (vp > 100U) { vp = 100U; }

    float vel_frac = (float)vp / 100.0f;
    float vel_deg_s = J5SP_VEL_MIN_DEG_S
                      + vel_frac * (J5SP_VEL_MAX_DEG_S - J5SP_VEL_MIN_DEG_S);

    float q_target[SERVO_COUNT];
    q_target[SERVO_BASE]   = sp_clamp_joint_runtime_deg(SERVO_BASE,   (float)base_deg);
    q_target[SERVO_SPALLA] = sp_clamp_joint_runtime_deg(SERVO_SPALLA, (float)spalla_deg);
    q_target[SERVO_GOMITO] = sp_clamp_joint_runtime_deg(SERVO_GOMITO, (float)gomito_deg);
    q_target[SERVO_YAW]    = sp_clamp_joint_runtime_deg(SERVO_YAW,    (float)yaw_deg);
    q_target[SERVO_PITCH]  = sp_clamp_joint_runtime_deg(SERVO_PITCH,  (float)pitch_deg);
    q_target[SERVO_ROLL]   = sp_clamp_joint_runtime_deg(SERVO_ROLL,   (float)roll_deg);

    float q_start[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        q_start[i] = sp_clamp_joint_runtime_deg(i, desired_positions[i]);
    }

    float dq_max = 0.0f;
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        float dq = q_target[i] - q_start[i];
        if (dq < 0.0f) { dq = -dq; }
        if (dq > dq_max) { dq_max = dq; }
    }

    float T_s = (dq_max > 0.1f) ? (dq_max / vel_deg_s) : 0.020f;
    uint32_t dur = (uint32_t)(T_s * 1000.0f + 0.5f);
    if (dur < 20U) { dur = 20U; }

    g_setpose_state.active         = false;
    g_setpose_state.profile        = profile;
    g_setpose_state.duration_ticks = dur;

    /* Setup wake-up tap per PITCH/ROLL prima di avviare il trajectory.
     * Se serve warm-up, lo start del trajectory viene posticipato di 100 ms;
     * durante quel tempo il tick mantiene preload sui servo del polso. */
    bool need_warmup = sp_setup_wakeup_tap(q_target);
    g_setpose_state.warming_up = need_warmup;
    g_setpose_state.warm_until_tick = g_rt_loop_ticks + J5SP_WAKEUP_DURATION_TICKS;
    g_setpose_state.start_tick = need_warmup
        ? (g_rt_loop_ticks + J5SP_WAKEUP_DURATION_TICKS)
        : g_rt_loop_ticks;

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        g_setpose_state.q_start[i]  = q_start[i];
        g_setpose_state.q_target[i] = q_target[i];
    }

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        step_accumulator[i] = 0.0f;
        g_setpose_state.prev_angle[i] = q_start[i];
        g_setpose_state.prev_vel[i]   = 0.0f;
    }
    g_setpose_state.max_velocity_deg_s = 0.0f;
    g_setpose_state.max_accel_deg_s2   = 0.0f;
    /* Clear the opt-in relax flag; callers that want it (HOME) must re-request
     * via j5vr_setpose_request_relax_digital_on_finish() AFTER this call. */
    g_setpose_state.relax_digital_on_finish = false;

    g_setpose_state.active = true;
}

void j5vr_setpose_request_relax_digital_on_finish(void)
{
    /* Must be called strictly AFTER j5vr_go_setpose()/j5vr_go_setpose_time() so
     * the start-of-motion reset doesn't clobber the request. Safe to call even
     * if no setpose is active (the flag will just be cleared at next start). */
    g_setpose_state.relax_digital_on_finish = true;
}

/* -----------------------------------------------------------------------
 * j5vr_go_setpose_time — posa assoluta 6-DOF con durata fissa in ms
 * ----------------------------------------------------------------------- */
void j5vr_go_setpose_time(
    const uint32_t *q_target_deg,
    int             count,
    uint32_t        time_ms,
    j5_profile_t    prof)
{
    if (count != SERVO_COUNT || q_target_deg == NULL) { return; }

    if (time_ms < 20U)    { time_ms = 20U; }
    if (time_ms > 60000U) { time_ms = 60000U; }

    volatile bool *pactive = &g_setpose_state.active;
    *pactive = false;

    g_setpose_state.profile        = prof;
    g_setpose_state.duration_ticks = time_ms;

    /* Pre-clamp dei target per il setup wake-up */
    float q_target_clamped[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        q_target_clamped[i] = sp_clamp_joint_runtime_deg(i, (float)q_target_deg[i]);
    }
    bool need_warmup = sp_setup_wakeup_tap(q_target_clamped);
    g_setpose_state.warming_up = need_warmup;
    g_setpose_state.warm_until_tick = g_rt_loop_ticks + J5SP_WAKEUP_DURATION_TICKS;
    g_setpose_state.start_tick = need_warmup
        ? (g_rt_loop_ticks + J5SP_WAKEUP_DURATION_TICKS)
        : g_rt_loop_ticks;

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        const float target = q_target_clamped[i];
        const float start  = sp_clamp_joint_runtime_deg(i, desired_positions[i]);
        g_setpose_state.q_target[i]   = target;
        g_setpose_state.q_start[i]    = start;
        g_setpose_state.prev_angle[i] = start;
        g_setpose_state.prev_vel[i]   = 0.0f;
        step_accumulator[i] = 0.0f;
    }
    g_setpose_state.max_velocity_deg_s = 0.0f;
    g_setpose_state.max_accel_deg_s2   = 0.0f;
    g_setpose_state.relax_digital_on_finish = false;

    *pactive = true;
}

/* -----------------------------------------------------------------------
 * j5vr_go_setpose_time_f — variante float per setpoint sub-degree.
 * Stessa semantica di j5vr_go_setpose_time ma accetta angoli q_target in
 * float (es. 90.4°) preservando la frazione decimale. Usata dal nuovo
 * comando UART SETPOSE_T_HR (high-resolution).
 * ----------------------------------------------------------------------- */
void j5vr_go_setpose_time_f(
    const float    *q_target_deg,
    int             count,
    uint32_t        time_ms,
    j5_profile_t    prof)
{
    if (count != SERVO_COUNT || q_target_deg == NULL) { return; }

    if (time_ms < 20U)    { time_ms = 20U; }
    if (time_ms > 60000U) { time_ms = 60000U; }

    volatile bool *pactive = &g_setpose_state.active;
    *pactive = false;

    g_setpose_state.profile        = prof;
    g_setpose_state.duration_ticks = time_ms;

    /* Wake-up tap setup (vedi commenti in j5vr_go_setpose_time) */
    float q_target_clamped[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        q_target_clamped[i] = sp_clamp_joint_runtime_deg(i, q_target_deg[i]);
    }
    bool need_warmup = sp_setup_wakeup_tap(q_target_clamped);
    g_setpose_state.warming_up = need_warmup;
    g_setpose_state.warm_until_tick = g_rt_loop_ticks + J5SP_WAKEUP_DURATION_TICKS;
    g_setpose_state.start_tick = need_warmup
        ? (g_rt_loop_ticks + J5SP_WAKEUP_DURATION_TICKS)
        : g_rt_loop_ticks;

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        const float target = q_target_clamped[i];
        const float start  = sp_clamp_joint_runtime_deg(i, desired_positions[i]);
        g_setpose_state.q_target[i]   = target;
        g_setpose_state.q_start[i]    = start;
        g_setpose_state.prev_angle[i] = start;
        g_setpose_state.prev_vel[i]   = 0.0f;
        step_accumulator[i] = 0.0f;
    }
    g_setpose_state.max_velocity_deg_s = 0.0f;
    g_setpose_state.max_accel_deg_s2   = 0.0f;
    g_setpose_state.relax_digital_on_finish = false;

    *pactive = true;
}
