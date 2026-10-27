/*
 * j5vr_actuation.c — Stato condiviso servo e modalità MANUAL/HYBRID
 *
 * Responsabilità (dopo il refactor modulare 3.1-3.5):
 *   - Stato condiviso tra moduli: desired_positions[], limiti per-giunto,
 *     velocità massima, LPF alpha, accumulatori step.
 *   - Helper interni: safe_increment, apply_desired, update_max_vel,
 *     j5_clampf, j5_angle_to_u8.
 *   - MANUAL mode (mode 2): j5vr_apply_setpoint_incremental.
 *   - HYBRID mode (mode 4): j5vr_apply_hybrid.
 *   - ASSIST mode (mode 5): j5vr_apply_mode5_arm_head.
 *   - API pubblica: init, center, teleop-pose, set_joint_limits, horizon.
 *
 * Moduli estratti (non toccare qui):
 *   HEAD    → j5vr_head.c/.h
 *   SETPOSE → j5vr_setpose.c/.h
 *   Quat    → j5vr_quat_utils.c/.h
 *   Manual puri → j5vr_manual.c/.h
 *
 * NOTE [Refactor-Phase1]:
 *   - Il path MANUAL/HYBRID in tempo reale (j5vr_apply_setpoint_incremental,
 *     j5vr_apply_hybrid, j5vr_actuation_apply_desired) NON deve essere
 *     modificato in logica né in firme.
 *   - Le routine di diagnostica o test (es. scan IMU, demo avanzate) possono
 *     essere solo documentate/annotate come TEST_ONLY, senza rimozioni.
 */

/* =========================================================
 * INCLUDES
 * ========================================================= */

/* Modulo proprio */
#include "servo/j5vr_actuation.h"

/* Moduli estratti */
#include "servo/j5vr_quat_utils.h"
#include "servo/j5vr_manual.h"
#include "servo/j5vr_setpose.h"
#include "servo/j5vr_head.h"

/* Driver / HAL */
#include "servo/servo_control.h"
#include "imu/imu.h"

/* Core Zephyr / RTOS */
#include "core/rt_loop.h"
#include "uart/uart_control.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* Librerie C */
#include <math.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(j5vr_actuation, LOG_LEVEL_INF);

/* =========================================================
 * MACRO DI DEBUG
 * ========================================================= */

#ifndef J5VR_DEBUG_MINIMAL
#define J5VR_DEBUG_MINIMAL 1
#endif

/* =========================================================
 * COSTANTI
 * ========================================================= */

/* Angoli centrali default (posizione neutra) */

/* Maschere bit pulsanti VR */
#define BUTTON_TRIGGER   (1U << 0)
#define BUTTON_GRIP      (1U << 1)
#define BUTTON_X         (1U << 4)
#define BUTTON_Y         (1U << 5)

/* Velocità massima di movimento (deg/s) */
#define MAX_VELOCITY_DEG_PER_SEC_MIN      1.0f
#define MAX_VELOCITY_DEG_PER_SEC_MAX    600.0f
#define MAX_VELOCITY_DEG_PER_SEC_DEFAULT  60.0f
#define VEL_VR_BTN_A   20.0f   /* pulsante A — modalità lenta */
#define VEL_VR_BTN_B  100.0f   /* pulsante B — modalità normale */

/* RT_LOOP_PERIOD_MS defined in core/rt_loop.h */
#define RT_LOOP_PERIOD_SEC (RT_LOOP_PERIOD_MS / 1000.0f)

/* Joystick deadzone: letta dai parametri HEAD a runtime (modificabile via SET_VR_PARAMS) */
#define JOYSTICK_DEADZONE (j5vr_head_get_params()->joy_dz)

/* SERVO_SAFETY_MIN/MAX_DEG defined in servo/servo_control.h */

/* =========================================================
 * STATE — variabili condivise con i moduli estratti
 *
 * Tutte le variabili di questa sezione sono non-static perché
 * accedute anche da j5vr_head.c e/o j5vr_setpose.c tramite
 * dichiarazioni extern in j5vr_head.h / j5vr_setpose.h.
 *
 *  desired_positions[]          Target angolare continuo per ogni giunto [deg].
 *                               Aggiornato da MANUAL, HEAD, HYBRID, SETPOSE.
 *                               Letto da j5vr_actuation_apply_desired() ogni ciclo.
 *
 *  step_accumulator[]           Residuo frazionario sottogrado tra cicli successivi.
 *                               Evita la perdita di precisione nell'arrotondamento a uint8.
 *
 *  joint_min_deg[] / joint_max_deg[]
 *                               Limiti per-giunto a runtime [deg]. Più restrittivi del
 *                               SERVO_SAFETY_MIN/MAX globale. Scrivibili via
 *                               j5vr_set_joint_limits(). Letti da safe_increment() e
 *                               j5vr_apply_head() (in j5vr_head.c).
 *
 *  joint_max_vel_deg_s[]        Cap di velocità per-giunto [deg/s]. 0 = usa il
 *                               valore globale current_max_velocity_deg_per_sec.
 *                               Usato da j5vr_actuation_apply_desired() e
 *                               j5vr_set_vr_params() (in j5vr_head.c).
 *
 *  joint_lpf_alpha[]            Coefficiente LPF per-giunto (EMA, [0,1]).
 *                               1.0 = bypass. In questa configurazione di
 *                               debug anche PITCH e ROLL restano in bypass
 *                               per allinearli al path diretto degli altri giunti.
 *
 *  current_max_velocity_deg_per_sec
 *                               Velocità massima globale corrente [deg/s].
 *                               Modificata dai pulsanti A/B e da SET_VR_PARAMS.
 * ========================================================= */

/** @brief Target angolare per-giunto [deg], mantenuto tra cicli. Condiviso con HEAD, SETPOSE. */
float desired_positions[SERVO_COUNT] = {
    90.0f,  /* BASE   */
    90.0f,  /* SPALLA */
    90.0f,  /* GOMITO */
    90.0f,  /* ROLL   */
    90.0f,  /* PITCH  */
    90.0f,  /* YAW    */
};

/** @brief Residuo frazionario accumulato per-giunto. Condiviso con SETPOSE. */
float step_accumulator[SERVO_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* Variazione minima di comando inviata al PWM: 0.02 deg = ~0.2 us, sotto la deadband (~2 us)
 * dei servo digitali, cosi' il moto resta continuo senza scrivere il timer a ogni tick a fermo. */
#define SERVO_SUBDEG_MIN_DELTA 0.02f

/** @brief Limite inferiore per-giunto [deg]. Ordine: BASE, SPALLA, GOMITO, YAW, PITCH, ROLL.
 *  Condiviso con j5vr_head.c. */
float joint_min_deg[SERVO_COUNT] = {
    10.0f,  /* BASE   */
    10.0f,  /* SPALLA */
    10.0f,  /* GOMITO */
    10.0f,  /* YAW    */
    45.0f,  /* PITCH  — centro 90 ±45 */
    45.0f,  /* ROLL   — centro 90 ±45 */
};

/** @brief Limite superiore per-giunto [deg]. Condiviso con j5vr_head.c. */
float joint_max_deg[SERVO_COUNT] = {
    170.0f, /* BASE   */
    170.0f, /* SPALLA */
    170.0f, /* GOMITO */
    170.0f, /* YAW    */
    135.0f, /* PITCH  — centro 90 ±45 */
    135.0f, /* ROLL   — centro 90 ±45 */
};

/** @brief Cap velocità per-giunto [deg/s]. 0 = usa current_max_velocity.
 *  Condiviso con j5vr_head.c. */
float joint_max_vel_deg_s[SERVO_COUNT] = {
    0.0f,   /* BASE   — LDX-218    */
    0.0f,   /* SPALLA — LDX-218    */
    0.0f,   /* GOMITO — LDX-218    */
    0.0f,   /* YAW    — LDX-218    */
    35.0f,  /* PITCH  — GIMOD 2065 */
    35.0f,  /* ROLL   — GIMOD 2065 */
};

/** @brief Coefficiente LPF EMA per-giunto (1.0 = bypass). Condiviso con j5vr_head.c. */
float joint_lpf_alpha[SERVO_COUNT] = {
    1.0f,   /* BASE   */
    1.0f,   /* SPALLA */
    1.0f,   /* GOMITO */
    1.0f,   /* YAW    */
    1.0f,   /* PITCH  — bypass diagnostico */
    1.0f,   /* ROLL   — bypass diagnostico */
};

/** @brief Velocità massima corrente [deg/s]. Modificata da pulsanti A/B o SET_VR_PARAMS.
 *  Condiviso con j5vr_head.c. */
float current_max_velocity_deg_per_sec = MAX_VELOCITY_DEG_PER_SEC_DEFAULT;

/** @brief Cap velocità polso [YAW,PITCH,ROLL] per mode HEAD/HYBRID (3/4). 0 = usa globale. */
static float joint_max_vel_deg_s_wrist_head[3] = { 0.0f, 0.0f, 0.0f };
/** @brief Cap velocità braccio [BASE,SPALLA,GOMITO] per mode HEAD/HYBRID (3/4). 0 = usa cap manuale/globale. */
static float joint_max_vel_deg_s_arm_head[3] = { 0.0f, 0.0f, 0.0f };

/* Diagnostica runtime (letta via UART) */
volatile uint32_t g_j5vr_apply_incr_calls  = 0;
volatile int16_t  g_j5vr_last_inc_mdeg_yaw    = 0;
volatile int16_t  g_j5vr_last_inc_mdeg_pitch  = 0;
volatile int16_t  g_j5vr_last_inc_mdeg_spalla = 0;
volatile int16_t  g_j5vr_last_inc_mdeg_gomito = 0;

/* Output LPF per-giunto — privato: non condiviso con altri moduli */
static float desired_positions_filtered[SERVO_COUNT] = {
    90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f
};

/* =========================================================
 * INTERNAL UTILS (static — usate solo da questo file)
 * ========================================================= */

/**
 * j5_clampf — clamp di un valore float in [min_val, max_val].
 * Centralizza tutti i pattern if/min/max presenti nel file.
 */
static inline float j5_clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

/**
 * safe_increment — applica un incremento a desired_positions[joint]
 * con guardia pre-limite e clamp post-incremento.
 */
static void safe_increment(servo_joint_t joint, float increment)
{

    const float lim_min = joint_min_deg[joint];
    const float lim_max = joint_max_deg[joint];

    /* Guardie pre-incremento: blocca se già al limite nella direzione richiesta */
    if (increment > 0.0f && desired_positions[joint] >= lim_max - 0.1f)
    {
        return;
    }
    if (increment < 0.0f && desired_positions[joint] <= lim_min + 0.1f)
    {
        return;
    }

    desired_positions[joint] += increment;

    /* Clamp post-incremento */
    const float clamped = j5_clampf(desired_positions[joint], lim_min, lim_max);
    desired_positions[joint] = clamped;
}

/* =========================================================
 * SHARED UTILS (non-static — usate anche da j5vr_head.c)
 * ========================================================= */

/**
 * j5vr_actuation_update_max_vel_buttons — aggiorna current_max_velocity_deg_per_sec
 * in base ai pulsanti A (lento) / B (normale) del controller VR destro.
 * Esposta a j5vr_head.c tramite j5vr_head.h.
 */
void j5vr_actuation_update_max_vel_buttons(const struct j5vr_state *j5vr)
{
    if (j5vr == NULL) { return; }

    const bool button_a = (j5vr->buttons_right & BUTTON_X) != 0;  /* A = bit4 destro */
    const bool button_b = (j5vr->buttons_right & BUTTON_Y) != 0;  /* B = bit5 destro */

    if      (button_a && !button_b) { current_max_velocity_deg_per_sec = VEL_VR_BTN_A; }
    else if (button_b && !button_a) { current_max_velocity_deg_per_sec = VEL_VR_BTN_B; }
}

/**
 * j5vr_actuation_set_wrist_vel_head — imposta cap velocità polso per HEAD/HYBRID (mode 3/4).
 * Chiamato da j5vr_set_vr_params. 0 = usa current_max_velocity_deg_per_sec.
 */
void j5vr_actuation_set_wrist_vel_head(float yaw_deg_s, float pitch_deg_s, float roll_deg_s)
{
    joint_max_vel_deg_s_wrist_head[0] = (yaw_deg_s   < 0.0f) ? 0.0f : yaw_deg_s;
    joint_max_vel_deg_s_wrist_head[1] = (pitch_deg_s < 0.0f) ? 0.0f : pitch_deg_s;
    joint_max_vel_deg_s_wrist_head[2] = (roll_deg_s  < 0.0f) ? 0.0f : roll_deg_s;
}

void j5vr_actuation_set_arm_vel_head(float base_deg_s, float spalla_deg_s, float gomito_deg_s)
{
    joint_max_vel_deg_s_arm_head[0] = (base_deg_s   < 0.0f) ? 0.0f : base_deg_s;
    joint_max_vel_deg_s_arm_head[1] = (spalla_deg_s < 0.0f) ? 0.0f : spalla_deg_s;
    joint_max_vel_deg_s_arm_head[2] = (gomito_deg_s < 0.0f) ? 0.0f : gomito_deg_s;
}

/**
 * j5vr_actuation_apply_desired — applica desired_positions[] ai servo
 * con rate-limiting (deg/s), LPF EMA per-giunto e accumulatore sottogrado.
 * Se j5vr != NULL e mode 3, 4 o 5, per il polso (YAW/PITCH/ROLL) usa le vel HEAD.
 *
 * Esposta a j5vr_head.c tramite j5vr_head.h.
 */
/* =========================================================
 * ORIZZONTE (vedi j5vr_actuation.h)
 * =========================================================
 * Cinematica della camera (stessa del Pi / dashboard): R = Rz(q1) Ry(q2+q3) Rz(q4) Ry(q5) Rx(q6), q in radianti
 * = (fisico - offset) * dir. L'asse y della camera (orizzontale dell'immagine) e' a livello quando R[2][1] = 0:
 * con A = Rz(q1) Ry(q2+q3) Rz(q4) Ry(q5), q6 = atan2(-A21, A22) (soluzione con l'immagine dritta). Rz(q1) non
 * cambia la riga z, quindi la base non entra. Guardando quasi in verticale (A21^2 + A22^2 piccolo) il roll non e'
 * definito: si lascia quello del modo. */

#define HORIZON_RAMP_S     0.5f
#define HORIZON_DEG2RAD    0.017453292519943295f
#define HORIZON_MIN_HORIZ  0.3f

static struct {
    bool  on;
    float dirs[SERVO_COUNT];
    float trim_rad;
    float u;            /* 0 = roll del modo, 1 = a livello */
    float mode_roll;    /* ultimo roll scritto dal modo */
    float written;      /* ultimo roll scritto qui */
    bool  have_written;
} hz = { .dirs = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f } };

void j5vr_horizon_set(bool on, const float dirs[6], float trim_deg)
{
    if (dirs != NULL)
    {
        for (int i = 0; i < SERVO_COUNT; i++) { hz.dirs[i] = (dirs[i] < 0.0f) ? -1.0f : 1.0f; }
    }
    hz.trim_rad = j5_clampf(trim_deg, -15.0f, 15.0f) * HORIZON_DEG2RAD;
    hz.on = on;
}

static void horizon_apply(void)
{
    const float target = hz.on ? 1.0f : 0.0f;
    if (hz.u == 0.0f && target == 0.0f)
    {
        hz.have_written = false;
        return;
    }
    /* il modo ha scritto un roll nuovo (diverso da quello lasciato qui): e' il suo valore */
    if (!hz.have_written || desired_positions[SERVO_ROLL] != hz.written)
    {
        hz.mode_roll = desired_positions[SERVO_ROLL];
    }
    const float k = RT_LOOP_PERIOD_SEC / HORIZON_RAMP_S;
    hz.u = j5_clampf(hz.u + j5_clampf(target - hz.u, -k, k), 0.0f, 1.0f);

    const float d2r = HORIZON_DEG2RAD;
    float q[5];
    for (int i = 0; i < 5; i++)
    {
        /* posa REALE (in MANUAL i target corrono avanti ai servo: livellare su quelli inclinava l'immagine) */
        q[i] = (servo_get_angle_f((servo_joint_t)i) - (float)servo_offset_deg[i]) * hz.dirs[i] * d2r;
    }
    const float s = q[1] + q[2];
    const float a21 = sinf(s) * sinf(q[3]);
    const float a22 = -sinf(s) * cosf(q[3]) * sinf(q[4]) + cosf(s) * cosf(q[4]);
    float roll = hz.mode_roll;
    if (sqrtf(a21 * a21 + a22 * a22) >= HORIZON_MIN_HORIZ)
    {
        const float q6 = atan2f(-a21, a22) + hz.trim_rad;
        const float level = (float)servo_offset_deg[SERVO_ROLL] + hz.dirs[SERVO_ROLL] * q6 / d2r;
        roll = hz.mode_roll + (level - hz.mode_roll) * hz.u;
    }
    roll = j5_clampf(roll, joint_min_deg[SERVO_ROLL], joint_max_deg[SERVO_ROLL]);
    desired_positions[SERVO_ROLL] = roll;
    hz.written = roll;
    hz.have_written = true;
}

void j5vr_actuation_apply_desired(const struct j5vr_state *j5vr)
{
    horizon_apply();
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        /* Posizione corrente con la frazione: l'uscita e' sub-grado (servo_set_angle_f), come nel
         * trajectory tick di SETPOSE. Prima il comando veniva arrotondato al grado intero:
         * 1 grado = ~11 us di impulso, il servo digitale (deadband ~2 us) saltava a gradini e
         * il polso PITCH/ROLL faceva vibrare il video in HEAD/ASSIST. */
        const float current_pos = servo_get_angle_f((servo_joint_t)i);
        const float diff        = desired_positions[i] - current_pos;

        /* Mode 5 (HEAD ASSIST): target B/S/G arriva a burst da RPi (~100 Hz). Con alpha<1 l'uscita
         * segue solo desired_positions_filtered (EMA) — stesso target sembra "mushy" e in
         * ritardo vs polso (Y/P/R ha alpha=1 da SET_VR_PARAMS). Qui allineiamo il path al polso. */
        const bool mode5_arm_ik = (j5vr != NULL) && (j5vr->mode == 5U) && (j5vr->mode5_arm_valid != 0U) &&
                                  (i <= SERVO_GOMITO);
        float alpha_eff = joint_lpf_alpha[i];
        if (mode5_arm_ik)
        {
            alpha_eff = 1.0f;
        }

        if (fabsf(diff) <= 0.001f)
        {
            step_accumulator[i] = 0.0f;
            continue;
        }

        /* 1. Velocità effettiva per-giunto: polso in HEAD/HYBRID usa set dedicato */
        float joint_vel = current_max_velocity_deg_per_sec;
        if (i == SERVO_YAW || i == SERVO_PITCH || i == SERVO_ROLL)
        {
            const bool use_head_vel = (j5vr != NULL) && (j5vr->mode == 3U || j5vr->mode == 4U || j5vr->mode == 5U);
            const int  idx          = (i == SERVO_YAW) ? 0 : (i == SERVO_PITCH) ? 1 : 2;
            if (use_head_vel && joint_max_vel_deg_s_wrist_head[idx] > 0.0f)
                joint_vel = joint_max_vel_deg_s_wrist_head[idx];
            else if (!use_head_vel && joint_max_vel_deg_s[i] > 0.0f)
                joint_vel = joint_max_vel_deg_s[i];
        }
        else
        {
            const bool use_head_vel = (j5vr != NULL) && (j5vr->mode == 3U || j5vr->mode == 4U || j5vr->mode == 5U);
            if (use_head_vel && i <= SERVO_GOMITO && joint_max_vel_deg_s_arm_head[i] > 0.0f)
                joint_vel = joint_max_vel_deg_s_arm_head[i];
            else if (mode5_arm_ik)
            {
                /* Non applicare il cap MANUAL (vel_base/spalla/gomito) pensato per joystick:
                 * limita solo da vel_max globale VR (come percepito sul polso). */
                joint_vel = current_max_velocity_deg_per_sec;
            }
            else if (joint_max_vel_deg_s[i] > 0.0f && joint_vel > joint_max_vel_deg_s[i])
                joint_vel = joint_max_vel_deg_s[i];
        }
        const float max_step = joint_vel * RT_LOOP_PERIOD_SEC;

        /* 2. Rate-limit dello step */
        const float step = j5_clampf(diff, -max_step, max_step);

        /* 3. LPF EMA (aggiornato sul target, non sullo step) */
        if (alpha_eff < 1.0f)
        {
            desired_positions_filtered[i] = alpha_eff * desired_positions[i]
                                            + (1.0f - alpha_eff) * desired_positions_filtered[i];
        }
        else
        {
            desired_positions_filtered[i] = desired_positions[i];
        }

        /* 4. Accumulo step frazionario */
        step_accumulator[i] += step;

        /* 5. Output: filtrato per servo digitali (alpha<1), raw+acc per analogici */
        float send_pos = (alpha_eff < 1.0f) ? desired_positions_filtered[i]
                                        : (current_pos + step_accumulator[i]);

        if (fabsf(step_accumulator[i]) < 0.05f && fabsf(step) < 0.05f)
        {
            continue; /* accumula ancora: non abbastanza per un passo hardware */
        }

        float new_pos = current_pos + step_accumulator[i];

        /* Clamp ai limiti di sicurezza fisici prima dell'invio */
        send_pos = j5_clampf(send_pos, SERVO_SAFETY_MIN_DEG, SERVO_SAFETY_MAX_DEG);
        new_pos  = j5_clampf(new_pos,  SERVO_SAFETY_MIN_DEG, SERVO_SAFETY_MAX_DEG);

        /* Polso in dominio logico unico 0..180 come nella versione buona. */
        if (i == SERVO_ROLL || i == SERVO_PITCH || i == SERVO_YAW) {
            send_pos = j5_clampf(send_pos, 0.0f, 180.0f);
        }

        if (fabsf(send_pos - current_pos) < SERVO_SUBDEG_MIN_DELTA)
        {
            continue; /* sotto la risoluzione utile del servo: accumula ancora */
        }

        if (!servo_set_angle_f((servo_joint_t)i, send_pos))
        {
            continue;
        }

        /* Log PWM rate-limited per i giunti più critici */
        if (i == SERVO_YAW || i == SERVO_PITCH || i == SERVO_SPALLA)
        {
            static uint32_t pwm_step_log = 0;
            if ((pwm_step_log++ % 50U) == 0U)
            {
                LOG_DBG("[PWM_STEP] j=%d cur=%d send=%d (cdeg)",
                       i, (int)lroundf(current_pos * 100.0f),
                       (int)lroundf(send_pos * 100.0f));
            }
        }

        /* Mantieni il residuo frazionario rispetto all'angolo effettivamente inviato */
        step_accumulator[i] = new_pos - send_pos;

#if J5VR_DEBUG_MINIMAL
        static uint8_t dbg_logged_move[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};
        if (!dbg_logged_move[i])
        {
            LOG_INF("[J5VR][MOVE] joint=%d %.2f->%.2f"
                    " (desired=%.2f diff=%.3f step=%.4f acc_rem=%.4f)",
                    i, (double)current_pos, (double)send_pos,
                    (double)desired_positions[i], (double)diff,
                    (double)step, (double)step_accumulator[i]);
            dbg_logged_move[i] = 1;
        }
#endif
    }
}

/* =========================================================
 * PUBLIC API — inizializzazione e utility
 * ========================================================= */

/**
 * j5vr_actuation_init — inizializza desired_positions[] con le posizioni HOME,
 * resetta la velocità massima al default e inizializza i parametri HEAD.
 * Da chiamare una sola volta all'avvio, prima del RT loop.
 */
void j5vr_actuation_init(void)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        desired_positions[i]          = (float)servo_offset_deg[i];
        desired_positions_filtered[i] = (float)servo_offset_deg[i];
    }
    current_max_velocity_deg_per_sec = MAX_VELOCITY_DEG_PER_SEC_DEFAULT;
    j5vr_head_init_defaults(j5vr_head_get_params(), j5vr_head_get_state());
    LOG_INF("[J5VR] Actuation initialized (max_vel=%.1f deg/s)",
            (double)current_max_velocity_deg_per_sec);
}

void j5vr_actuation_set_init_pose(const float q_deg[6])
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        const float v = j5_clampf(q_deg[i], joint_min_deg[i], joint_max_deg[i]);
        desired_positions[i]          = v;
        desired_positions_filtered[i] = v;
        step_accumulator[i]           = 0.0f;
        servo_set_angle_init((servo_joint_t)i, v);
    }
}

/**
 * j5vr_center_all_servos — porta tutti i servo alla posa HOME tramite SETPOSE (RTR5).
 * Non scrive direttamente desired_positions[]: delega al RT loop via g_setpose_state.
 * Ritorna immediatamente (non bloccante).
 *
 * Dopo che la traiettoria di centraggio è terminata, i servo PITCH e ROLL
 * vengono rilasciati automaticamente (PWM=0) per ridurre il surriscaldamento
 * dei due giunti più stressati. Gli altri servo restano ingaggiati. La richiesta
 * è valida solo per questo setpose: il flag viene consumato in SETPOSE_DONE e
 * azzerato al prossimo j5vr_go_setpose (nessun effetto su SETPOSE/PARK/TELEOPPOSE).
 */
void j5vr_center_all_servos(void)
{
    j5vr_go_setpose(
        servo_offset_deg[SERVO_BASE],
        servo_offset_deg[SERVO_SPALLA],
        servo_offset_deg[SERVO_GOMITO],
        servo_offset_deg[SERVO_YAW],
        servo_offset_deg[SERVO_PITCH],
        servo_offset_deg[SERVO_ROLL],
        40U,
        J5_PROFILE_RTR5
    );
    /* Richiedi il rilascio di PITCH/ROLL al termine di QUESTO setpose (HOME).
     * Deve essere chiamata DOPO j5vr_go_setpose(), che azzera il flag all'avvio. */
    j5vr_setpose_request_relax_digital_on_finish();
}

/**
 * j5vr_go_teleop_pose — porta il robot in VR/Teleop Pose tramite SETPOSE (RTR5).
 * Ritorna immediatamente (non bloccante).
 */
void j5vr_go_teleop_pose(void)
{
    j5vr_go_setpose(
        servo_teleop_deg[SERVO_BASE],
        servo_teleop_deg[SERVO_SPALLA],
        servo_teleop_deg[SERVO_GOMITO],
        servo_teleop_deg[SERVO_YAW],
        servo_teleop_deg[SERVO_PITCH],
        servo_teleop_deg[SERVO_ROLL],
        40U,
        J5_PROFILE_RTR5
    );
}

/* =========================================================
 * Interpolazione fra frame SPI (26/10/2026, scatti in ASSIST sul robot)
 * =========================================================
 * Il master manda un frame J5VR ogni 10 ms (20 ms quando al suo posto va una richiesta STATUS) e il RT loop
 * segue ogni target nuovo in pochi ms (polso fino a 400 deg/s, base 600): il braccio si muoveva a gradini di
 * 10-20 ms. Qui il valore continuo (quaternione, B/S/G dell'ASSIST) scorre linearmente dal valore attuale a
 * quello del frame nuovo nell'intervallo fra gli ultimi due frame: traiettoria continua, ritardo <= 1 frame.
 * Cambio di modo, validita' B/S/G cambiata o frame fermi da piu' di SMOOTH_STALE_MS: si salta al valore del
 * frame (nessuna rincorsa da uno stato vecchio). */

#define SMOOTH_MIN_MS   5U
#define SMOOTH_MAX_MS   30U
#define SMOOTH_STALE_MS 100U

static struct {
    bool     init;
    uint8_t  mode;
    uint8_t  arm_valid;
    float    raw_q[4];       /* ultimo frame visto (per riconoscere il frame nuovo) */
    int16_t  raw_a[3];
    float    q0[4], q1[4];   /* da -> a */
    float    a0[3], a1[3];
    float    out_q[4];       /* ultimo valore uscito */
    float    out_a[3];
    uint32_t t_frame;        /* tick del frame nuovo */
    uint32_t dur;            /* durata della corsa verso il frame nuovo */
} sm;

void j5vr_smooth_between_frames(struct j5vr_state *s, uint32_t now_ms)
{
    if (s == NULL) { return; }
    const float rq[4] = { s->quat_w, s->quat_x, s->quat_y, s->quat_z };
    const bool smooth_mode = (s->mode == 3U || s->mode == 4U || s->mode == 5U);
    const bool fresh = !sm.init ||
        rq[0] != sm.raw_q[0] || rq[1] != sm.raw_q[1] || rq[2] != sm.raw_q[2] || rq[3] != sm.raw_q[3] ||
        s->mode5_arm_target_cdeg[0] != sm.raw_a[0] || s->mode5_arm_target_cdeg[1] != sm.raw_a[1] ||
        s->mode5_arm_target_cdeg[2] != sm.raw_a[2];
    const bool snap = !sm.init || !smooth_mode || s->mode != sm.mode || s->mode5_arm_valid != sm.arm_valid ||
                      (now_ms - sm.t_frame) > SMOOTH_STALE_MS;

    if (fresh || snap)
    {
        uint32_t dt = now_ms - sm.t_frame;
        sm.dur = (dt < SMOOTH_MIN_MS) ? SMOOTH_MIN_MS : (dt > SMOOTH_MAX_MS) ? SMOOTH_MAX_MS : dt;
        for (int i = 0; i < 4; i++)
        {
            sm.raw_q[i] = rq[i];
            sm.q1[i] = rq[i];
            sm.q0[i] = snap ? rq[i] : sm.out_q[i];
        }
        for (int i = 0; i < 3; i++)
        {
            sm.raw_a[i] = s->mode5_arm_target_cdeg[i];
            sm.a1[i] = (float)s->mode5_arm_target_cdeg[i];
            sm.a0[i] = snap ? sm.a1[i] : sm.out_a[i];
        }
        /* stesso emisfero: q e -q sono la stessa rotazione, interpolare sull'arco corto */
        if (sm.q0[0] * sm.q1[0] + sm.q0[1] * sm.q1[1] + sm.q0[2] * sm.q1[2] + sm.q0[3] * sm.q1[3] < 0.0f)
        {
            for (int i = 0; i < 4; i++) { sm.q0[i] = -sm.q0[i]; }
        }
        sm.t_frame   = now_ms;
        sm.mode      = s->mode;
        sm.arm_valid = s->mode5_arm_valid;
        sm.init      = true;
    }
    if (!smooth_mode)
    {
        return;
    }

    float u = (float)(now_ms - sm.t_frame) / (float)sm.dur;
    u = j5_clampf(u, 0.0f, 1.0f);
    float n = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        sm.out_q[i] = sm.q0[i] + (sm.q1[i] - sm.q0[i]) * u;
        n += sm.out_q[i] * sm.out_q[i];
    }
    n = (n > 1e-12f) ? sqrtf(n) : 1.0f;
    for (int i = 0; i < 4; i++) { sm.out_q[i] /= n; }
    s->quat_w = sm.out_q[0];
    s->quat_x = sm.out_q[1];
    s->quat_y = sm.out_q[2];
    s->quat_z = sm.out_q[3];
    for (int i = 0; i < 3; i++)
    {
        sm.out_a[i] = sm.a0[i] + (sm.a1[i] - sm.a0[i]) * u;
        if (s->mode5_arm_valid != 0U)
        {
            s->mode5_arm_target_cdeg[i] = (int16_t)lroundf(sm.out_a[i]);
        }
    }
}

/**
 * j5vr_set_joint_limits — aggiorna limiti per-giunto a runtime.
 * I valori vengono clampati ai limiti di sicurezza fisici globali.
 */
void j5vr_set_joint_limits(int joint, float lim_min, float lim_max)
{
    if (joint < 0 || joint >= SERVO_COUNT)   { return; }
    lim_min = j5_clampf(lim_min, SERVO_SAFETY_MIN_DEG, SERVO_SAFETY_MAX_DEG);
    lim_max = j5_clampf(lim_max, SERVO_SAFETY_MIN_DEG, SERVO_SAFETY_MAX_DEG);
    if (lim_min > lim_max)                   { return; }
    joint_min_deg[joint] = lim_min;
    joint_max_deg[joint] = lim_max;
}

/* =========================================================
 * MODE DISPATCH — MANUAL (mode 2)
 * ========================================================= */

/**
 * j5vr_apply_setpoint_incremental — MANUAL mode (1).
 *
 * Per ogni asse:
 *   - Se input è zero: snap desired alla posizione corrente (no coda).
 *   - Se input attivo: safe_increment con velocità proporzionale all'input.
 *
 * Chiamata da rt_loop.c nel case 1 del mode switch, dopo la validazione
 * con j5vr_to_servo_setpoint() e il controllo freeze/deadman.
 */
bool j5vr_apply_setpoint_incremental(const struct j5vr_state *j5vr, bool movement_allowed)
{
    if (j5vr == NULL) { return false; }

    g_j5vr_apply_incr_calls++;

#if J5VR_DEBUG_MINIMAL
    {
        static uint32_t log_counter_func = 0;
        if ((log_counter_func++ % 500) == 0)
        {
            LOG_INF("[J5VR] apply: mode=%u movement_allowed=%d",
                    j5vr->mode, movement_allowed ? 1 : 0);
        }
    }
    {
        static uint32_t dbg_first3 = 0;
        if (dbg_first3 < 3 &&
            (j5vr->joy_x != 0 || j5vr->joy_y != 0 || j5vr->pitch != 0 || j5vr->yaw != 0))
        {
            LOG_INF("[J5VR][DBG3] raw: joy_x=%d joy_y=%d pitch=%d yaw=%d"
                    " intensity=%u btnL=0x%04x btnR=0x%04x",
                    j5vr->joy_x, j5vr->joy_y, j5vr->pitch, j5vr->yaw,
                    j5vr->intensity, j5vr->buttons_left, j5vr->buttons_right);
            dbg_first3++;
        }
    }
#endif

    if (!movement_allowed)
    {
        const uint16_t bl_raw = j5vr->buttons_left;
        const uint16_t bl_sw = (uint16_t)((bl_raw >> 8) | (bl_raw << 8));
        const bool bx = ((bl_raw & BUTTON_X) != 0U) || ((bl_sw & BUTTON_X) != 0U) || ((j5vr->priority & BUTTON_X) != 0U);
        const bool by = ((bl_raw & BUTTON_Y) != 0U) || ((bl_sw & BUTTON_Y) != 0U) || ((j5vr->priority & BUTTON_Y) != 0U);
        const bool roll_req = bx || by;
        if (roll_req || j5vr->joy_y != 0) {
            LOG_DBG("[MANUAL_GATE] mode=%u joy_y=%d btnL=0x%04x",
                   (unsigned)j5vr->mode,
                   (int)j5vr->joy_y,
                   (unsigned)j5vr->buttons_left);
        }
        return false;
    }

    j5vr_actuation_update_max_vel_buttons(j5vr);

    /* --- Snap asse → posizione corrente quando input è zero (no coda al rilascio) --- */
    {
        const float jx = j5vr_int16_to_normalized_dz(j5vr->joy_x, JOYSTICK_DEADZONE);
        const float jy = j5vr_int16_to_normalized_dz(j5vr->joy_y, JOYSTICK_DEADZONE);
        const float yw = j5vr_int16_to_normalized_dz(j5vr->yaw,   JOYSTICK_DEADZONE);
        const float pt = j5vr_int16_to_normalized_dz(j5vr->pitch, JOYSTICK_DEADZONE);

        const bool trig_l = (j5vr->buttons_left  & BUTTON_TRIGGER) != 0;
        const bool trig_r = (j5vr->buttons_right & BUTTON_TRIGGER) != 0;
        const bool btn_x  = (j5vr->buttons_left & BUTTON_X) != 0U;
        const bool btn_y  = (j5vr->buttons_left & BUTTON_Y) != 0U;

        const bool base_has_input = (trig_l && !trig_r) || (trig_r && !trig_l);
        const bool roll_has_input = (btn_x  && !btn_y)  || (btn_y  && !btn_x);

        if (fabsf(jx) <= 0.01f) {
            desired_positions[SERVO_YAW]    = servo_get_angle_f(SERVO_YAW);
            step_accumulator[SERVO_YAW]     = 0.0f;
        }
        if (fabsf(jy) <= 0.01f) {
            desired_positions[SERVO_PITCH]  = servo_get_angle_f(SERVO_PITCH);
            step_accumulator[SERVO_PITCH]   = 0.0f;
        }
        if (!base_has_input) {
            desired_positions[SERVO_BASE]   = servo_get_angle_f(SERVO_BASE);
            step_accumulator[SERVO_BASE]    = 0.0f;
        }
        if (fabsf(yw) <= 0.01f) {
            desired_positions[SERVO_SPALLA] = servo_get_angle_f(SERVO_SPALLA);
            step_accumulator[SERVO_SPALLA]  = 0.0f;
        }
        if (fabsf(pt) <= 0.01f) {
            desired_positions[SERVO_GOMITO] = servo_get_angle_f(SERVO_GOMITO);
            step_accumulator[SERVO_GOMITO]  = 0.0f;
        }
        /* con l'Orizzonte il roll lo decide horizon_apply(): azzerarlo qui a ogni tick teneva l'accumulatore
         * sotto la soglia di passo e il roll non si muoveva mai */
        if (!roll_has_input && !hz.on) {
            desired_positions[SERVO_ROLL]   = servo_get_angle_f(SERVO_ROLL);
            step_accumulator[SERVO_ROLL]    = 0.0f;
        }
    }

    /* --- Parametri comuni di velocità --- */
    const float max_step_per_cycle = current_max_velocity_deg_per_sec * RT_LOOP_PERIOD_SEC;
    const float base_increment     = max_step_per_cycle * 2.0f;


    float intensity_scale = (float)j5vr->intensity / 255.0f;
    if (intensity_scale <= 0.0f) intensity_scale = 1.0f;

    /* --- Diagnostica UART: ultimi incrementi (milli-deg per ciclo) --- */
    {
        const float jx_n = j5vr_int16_to_normalized_dz(j5vr->joy_x, JOYSTICK_DEADZONE);
        const float jy_n = j5vr_int16_to_normalized_dz(j5vr->joy_y, JOYSTICK_DEADZONE);
        const float yw_n = j5vr_int16_to_normalized_dz(j5vr->yaw,   JOYSTICK_DEADZONE);
        const float pt_n = j5vr_int16_to_normalized_dz(j5vr->pitch, JOYSTICK_DEADZONE);

        int32_t iy = (int32_t)lroundf((jx_n * base_increment * intensity_scale) * 1000.0f);
        int32_t ip = (int32_t)lroundf((jy_n * base_increment * intensity_scale) * 1000.0f);
        int32_t is = (int32_t)lroundf((yw_n * base_increment * intensity_scale) * 1000.0f);
        int32_t ig = (int32_t)lroundf((pt_n * base_increment * intensity_scale) * 1000.0f);

        /* Clamp a int16 */
        iy = (iy < -32768) ? -32768 : (iy > 32767) ? 32767 : iy;
        ip = (ip < -32768) ? -32768 : (ip > 32767) ? 32767 : ip;
        is = (is < -32768) ? -32768 : (is > 32767) ? 32767 : is;
        ig = (ig < -32768) ? -32768 : (ig > 32767) ? 32767 : ig;

        g_j5vr_last_inc_mdeg_yaw    = (int16_t)iy;
        g_j5vr_last_inc_mdeg_pitch  = (int16_t)ip;
        g_j5vr_last_inc_mdeg_spalla = (int16_t)is;
        g_j5vr_last_inc_mdeg_gomito = (int16_t)ig;
    }

    /* --- Log incrementi rate-limited (printk diagnostica) --- */
    {
        static uint32_t incr_log = 0;
        if ((incr_log++ % 50U) == 0U)
        {
            const float jx_d = j5vr_int16_to_normalized_dz(j5vr->joy_x, JOYSTICK_DEADZONE);
            const float jy_d = j5vr_int16_to_normalized_dz(j5vr->joy_y, JOYSTICK_DEADZONE);
            const float yw_d = j5vr_int16_to_normalized_dz(j5vr->yaw,   JOYSTICK_DEADZONE);
            const float pt_d = j5vr_int16_to_normalized_dz(j5vr->pitch, JOYSTICK_DEADZONE);

            LOG_DBG("[INCR] jx=%.2f jy=%.2f p=%.2f y=%.2f int=%u",
                   (double)jx_d, (double)jy_d,
                   (double)pt_d, (double)yw_d,
                   (unsigned)j5vr->intensity);
        }
    }

    /* --- Applicazione incrementi per asse --- */

    /* Left stick X → YAW */
    const float joy_x_norm = j5vr_int16_to_normalized_dz(j5vr->joy_x, JOYSTICK_DEADZONE);
    if (fabsf(joy_x_norm) > 0.01f)
    {
        const float inc = joy_x_norm * base_increment * intensity_scale;
        safe_increment(SERVO_YAW, inc);
    }

    /* Left stick Y → PITCH */
    const float joy_y_norm = j5vr_int16_to_normalized_dz(j5vr->joy_y, JOYSTICK_DEADZONE);
    if (fabsf(joy_y_norm) > 0.01f)
    {
        const float inc = joy_y_norm * base_increment * intensity_scale;
        safe_increment(SERVO_PITCH, inc);
    }

    /* Trigger → BASE */
    const bool trigger_left  = (j5vr->buttons_left  & BUTTON_TRIGGER) != 0;
    const bool trigger_right = (j5vr->buttons_right & BUTTON_TRIGGER) != 0;
    if (trigger_left && !trigger_right)
    {
        const float inc = -base_increment * intensity_scale;
        safe_increment(SERVO_BASE, inc);
    }
    else if (trigger_right && !trigger_left)
    {
        const float inc = base_increment * intensity_scale;
        safe_increment(SERVO_BASE, inc);
    }

    /* Right stick X (yaw channel) → SPALLA */
    const float yaw_norm = j5vr_int16_to_normalized_dz(j5vr->yaw, JOYSTICK_DEADZONE);
    if (fabsf(yaw_norm) > 0.01f)
    {
        const float inc = yaw_norm * base_increment * intensity_scale;
        safe_increment(SERVO_SPALLA, inc);
    }

    /* Right stick Y (pitch channel) → GOMITO */
    const float pitch_norm = j5vr_int16_to_normalized_dz(j5vr->pitch, JOYSTICK_DEADZONE);
    if (fabsf(pitch_norm) > 0.01f)
    {
        const float inc = pitch_norm * base_increment * intensity_scale;
        safe_increment(SERVO_GOMITO, inc);
    }

    /* Pulsanti X/Y sinistro → ROLL */
    const bool button_x = (j5vr->buttons_left & BUTTON_X) != 0U;
    const bool button_y = (j5vr->buttons_left & BUTTON_Y) != 0U;
    if (button_x && !button_y)
    {
        const float inc = base_increment * intensity_scale;
        safe_increment(SERVO_ROLL, inc);
    }
    else if (button_y && !button_x)
    {
        const float inc = -base_increment * intensity_scale;
        safe_increment(SERVO_ROLL, inc);
    }

    j5vr_actuation_apply_desired(j5vr);
    return true;
}

void j5vr_apply_mode5_arm_head(const struct j5vr_state *s)
{
    if (s == NULL)
    {
        return;
    }

    j5vr_actuation_update_max_vel_buttons(s);

    if (s->mode == 5U && s->mode5_arm_valid != 0U)
    {
        for (int i = 0; i < 3; i++)
        {
            const float target_deg = ((float)s->mode5_arm_target_cdeg[i]) / 100.0f;
            desired_positions[i] = j5_clampf(target_deg, joint_min_deg[i], joint_max_deg[i]);
        }
    }

    j5vr_apply_head(s, false);
    j5vr_actuation_apply_desired(s);
}

/* =========================================================
 * MODE DISPATCH — HYBRID (mode 4)
 * ========================================================= */

/**
 * j5vr_apply_hybrid — HYBRID mode (4).
 * Combina orientamento testa (open-loop, dal quaternione visore)
 * con joystick per i giunti non controllati dalla testa.
 *
 * ATTENZIONE: NON validato in produzione. Usare solo in sessioni
 * di test dedicate. Il dispatch è in rt_loop.c, case 3.
 */
void j5vr_apply_hybrid(const struct j5vr_state *s)
{
    if (s == NULL) { return; }

    j5vr_actuation_update_max_vel_buttons(s);

    float intensity_scale = (float)s->intensity / 255.0f;
    if (intensity_scale <= 0.0f) intensity_scale = 1.0f;

    const float max_step_per_cycle = current_max_velocity_deg_per_sec * RT_LOOP_PERIOD_SEC;
    const float base_increment     = max_step_per_cycle * 2.0f;

    /* HYBRID = testa come HEAD (Y/P/R) + base/spalla/gomito come MANUAL. */

    /* Snap BASE/SPALLA/GOMITO quando input zero (stessa semantica MANUAL). */
    {
        const float yw = j5vr_int16_to_normalized_dz(s->yaw,   JOYSTICK_DEADZONE);
        const float pt = j5vr_int16_to_normalized_dz(s->pitch, JOYSTICK_DEADZONE);
        const bool trig_l = (s->buttons_left  & BUTTON_TRIGGER) != 0;
        const bool trig_r = (s->buttons_right & BUTTON_TRIGGER) != 0;
        const bool base_has_input = (trig_l && !trig_r) || (trig_r && !trig_l);

        if (!base_has_input) {
            desired_positions[SERVO_BASE] = servo_get_angle_f(SERVO_BASE);
            step_accumulator[SERVO_BASE]  = 0.0f;
        }
        if (fabsf(yw) <= 0.01f) {
            desired_positions[SERVO_SPALLA] = servo_get_angle_f(SERVO_SPALLA);
            step_accumulator[SERVO_SPALLA]  = 0.0f;
        }
        if (fabsf(pt) <= 0.01f) {
            desired_positions[SERVO_GOMITO] = servo_get_angle_f(SERVO_GOMITO);
            step_accumulator[SERVO_GOMITO]  = 0.0f;
        }
    }

    /* BASE: trigger sinistro/destro (MANUAL). */
    {
        const bool trigger_left  = (s->buttons_left  & BUTTON_TRIGGER) != 0;
        const bool trigger_right = (s->buttons_right & BUTTON_TRIGGER) != 0;
        if (trigger_left && !trigger_right)
            safe_increment(SERVO_BASE, -base_increment * intensity_scale);
        else if (trigger_right && !trigger_left)
            safe_increment(SERVO_BASE, base_increment * intensity_scale);
    }

    /* SPALLA/GOMITO: right stick X/Y (MANUAL). */
    {
        const float yaw_norm = j5vr_int16_to_normalized_dz(s->yaw, JOYSTICK_DEADZONE);
        if (fabsf(yaw_norm) > 0.01f)
            safe_increment(SERVO_SPALLA, yaw_norm * base_increment * intensity_scale);

        const float pitch_norm = j5vr_int16_to_normalized_dz(s->pitch, JOYSTICK_DEADZONE);
        if (fabsf(pitch_norm) > 0.01f)
            safe_increment(SERVO_GOMITO, pitch_norm * base_increment * intensity_scale);
    }

    /* Testa: YAW/PITCH/ROLL con la stessa pipeline del mode HEAD. */
    j5vr_apply_head(s, false);

    j5vr_actuation_apply_desired(s);
}
