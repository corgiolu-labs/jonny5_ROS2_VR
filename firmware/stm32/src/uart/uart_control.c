/*
 * uart_control.c — UART Control Plane
 *
 * Parser e dispatcher comandi ASCII su USART1 (PA9 TX / PA10 RX).
 *
 * Comandi supportati:
 *   STOP, SAFE, RESET, ENABLE, STATUS?
 *   IMUON, IMUOFF, VR?, HEADZERO
 *   HOME, PARK
 *   SETPOSE <B S G Y P R vel PLANNER>
 *   SETPOSE_T <B S G Y P R time_ms PLANNER>
 *   SET_OFFSETS <B S G Y P R>
 *   SET_JOINT_LIMITS <idx min_deg max_deg>
 *   SET_VR_PARAMS <16 float> [18 uint opzionali]
 *
 * Pipeline per ogni riga ricevuta:
 *   RX IRQ → ring buffer → uart_control_process() →
 *   uart_parse_seq_and_cmd() → uart_process_command() →
 *   uart_send_response_with_seq()
 *
 * Nota: DEMO è orchestrato lato Raspberry (sequenza SETPOSE da settings.json).
 * Sequenza per uscire da STOPPED: SAFE → ENABLE → IDLE.
 *
 * NOTE [Refactor-Phase1]:
 *   - Il parser/dispatcher principale (uart_control_process, uart_process_command,
 *     uart_parse_seq_and_cmd, uart_send_response_with_seq) è parte del control
 *     plane industriale e NON deve essere modificato nella logica.
 *   - Eventuali helper legacy o diagnostici possono essere solo etichettati
 *     via commenti come SUPERFLUA_LEGACY / TEST_ONLY, senza rimozioni.
 */

#include "uart/uart_control.h"
#include "core/state_machine.h"
#include "servo/j5vr_actuation.h"
#include "servo/j5vr_head.h"
#include "servo/servo_control.h"
#include "servo/pickplace.h"
#include "spi/j5_protocol.h"
#include "core/rt_loop.h"
#include "imu/imu.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/printk.h>

/* Dump diagnostico [VR_PARAMS] su SET_VR_PARAMS: printk SINCRONO in main
 * (priorita' 0) con 4 float -> puo' ritardare il RT loop di qualche ms quando
 * il Pi apre la sessione teleop. A 0 e' compilato fuori. NB: l'ack di protocollo
 * "OK SET_VR_PARAMS" (uart_send_response_with_seq, sotto) NON dipende da questo. */
#ifndef UART_VR_PARAMS_DEBUG
#define UART_VR_PARAMS_DEBUG 0
#endif
#include <zephyr/sys/ring_buffer.h>

/* =========================================================
 * RX ring buffer (ISR → process thread)
 * ========================================================= */

#define UART_RX_RING_SIZE 512
static uint8_t uart_rx_ring_storage[UART_RX_RING_SIZE];
static struct ring_buf uart_rx_ring;

/** Callback IRQ RX: svuota FIFO driver STM32 nel ring buffer. */
static void uart_rx_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev))
    {
        if (uart_fifo_read(dev, &c, 1) == 1)
        {
            ring_buf_put(&uart_rx_ring, &c, 1);
        }
    }
}

/* =========================================================
 * STATE
 * ========================================================= */

/** UART device: USART1 (PA9 TX, PA10 RX) */
static const struct device *uart_dev = NULL;

/** Flag di configurazione: abilita/disabilita la modalità HYBRID (mode 3). */
static bool hybrid_enabled = false;

#define UART_RX_BUF_SIZE 256
static char uart_rx_buffer[UART_RX_BUF_SIZE];   /* accumulo byte per riga */
static char uart_cmd_parsed[UART_RX_BUF_SIZE];  /* cmd senza prefisso seq  */
static int  uart_rx_idx = 0;

/* =========================================================
 * TX helpers
 * ========================================================= */

/**
 * uart_send_response_with_seq — invia risposta con eventuale prefisso "#<seq> ".
 * Se seq == 0 il prefisso viene omesso.
 */
static void uart_send_response_with_seq(uint32_t seq, const char *payload)
{
    if (uart_dev == NULL) { return; }

    if (seq > 0U)
    {
        char prefix[16];
        int n = snprintf(prefix, sizeof(prefix), "#%u ", (unsigned)seq);
        for (int i = 0; i < n; i++)
            uart_poll_out(uart_dev, prefix[i]);
    }
    for (const char *p = payload; *p != '\0'; p++)
        uart_poll_out(uart_dev, *p);
    uart_poll_out(uart_dev, '\n');
}

/** uart_send_unsolicited — invia "<msg>\n" senza seq number. */
void uart_send_unsolicited(const char *msg)
{
    if (uart_dev == NULL || msg == NULL) { return; }
    for (const char *p = msg; *p != '\0'; p++)
        uart_poll_out(uart_dev, *p);
    uart_poll_out(uart_dev, '\n');
}

bool uart_is_hybrid_enabled(void)
{
    return hybrid_enabled;
}

/* =========================================================
 * Parser primitives (no sscanf — comportamento deterministico)
 * ========================================================= */

/** Salta spazi e tab. */
static void skip_spaces(const char **p)
{
    while (**p == ' ' || **p == '\t')
        (*p)++;
}

/**
 * parse_uint — legge un intero decimale non-negativo da *p.
 * Avanza il puntatore oltre le cifre. Rifiuta numeri > 99999.
 */
static bool parse_uint(const char **p, unsigned int *out)
{
    skip_spaces(p);
    if (**p < '0' || **p > '9') { return false; }
    unsigned int val = 0U;
    while (**p >= '0' && **p <= '9')
    {
        if (val > 9999U) { return false; }
        val = val * 10U + (unsigned int)(**p - '0');
        (*p)++;
    }
    *out = val;
    return true;
}

/**
 * parse_float — legge un numero float con segno opzionale e parte decimale.
 * Senza libreria math: pura aritmetica intera + divisione.
 */
static bool parse_float(const char **p, float *out)
{
    skip_spaces(p);
    bool neg = false;
    if      (**p == '-') { neg = true; (*p)++; }
    else if (**p == '+') { (*p)++; }

    if ((**p < '0' || **p > '9') && **p != '.') { return false; }

    float val = 0.0f;
    while (**p >= '0' && **p <= '9')
    {
        val = val * 10.0f + (float)(**p - '0');
        (*p)++;
    }
    if (**p == '.')
    {
        (*p)++;
        float frac = 0.1f;
        while (**p >= '0' && **p <= '9')
        {
            val += (float)(**p - '0') * frac;
            frac *= 0.1f;
            (*p)++;
        }
    }
    *out = neg ? -val : val;
    return true;
}

/**
 * parse_alpha — legge un token alfanumerico + trattino (es. "RTR5", "B-C-B").
 * Accetta [A-Za-z0-9-], si ferma su spazio/fine stringa.
 */
static bool parse_alpha(const char **p, char *buf, size_t buf_len)
{
    skip_spaces(p);
    char c = **p;
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '-'))
    {
        return false;
    }
    size_t i = 0U;
    while (i < buf_len - 1U)
    {
        c = **p;
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-')
        {
            buf[i++] = c;
            (*p)++;
        }
        else { break; }
    }
    buf[i] = '\0';
    return (i > 0U);
}

/* =========================================================
 * Parsing pipeline helpers
 * ========================================================= */

/**
 * uart_parse_seq_and_cmd — estrae seq e comando da una riga grezza.
 * Se la riga inizia con '#': seq = numero, cmd = resto (dopo spazio).
 * Altrimenti: seq = 0, cmd = copia della riga.
 */
static void uart_parse_seq_and_cmd(const char *line, uint32_t *seq, char *cmd_buf)
{
    *seq = 0U;
    if (line[0] == '#')
    {
        unsigned int val = 0U;
        const char *p = line + 1;
        while (*p >= '0' && *p <= '9')
        {
            val = val * 10U + (unsigned int)(*p - '0');
            p++;
        }
        *seq = (uint32_t)val;
        while (*p == ' ' || *p == '\t') { p++; }
        size_t i = 0;
        while (*p != '\0' && i < (UART_RX_BUF_SIZE - 1))
            cmd_buf[i++] = *p++;
        cmd_buf[i] = '\0';
    }
    else
    {
        size_t len = strlen(line);
        if (len >= UART_RX_BUF_SIZE) { len = UART_RX_BUF_SIZE - 1; }
        memcpy(cmd_buf, line, len);
        cmd_buf[len] = '\0';
    }
}

/**
 * planner_from_string — converte stringa planner in j5_profile_t.
 * Ritorna false se la stringa non è riconosciuta (out invariato).
 */
static bool planner_from_string(const char *s, j5_profile_t *out)
{
    if (strcmp(s, "RTR3") == 0)                            { *out = J5_PROFILE_RTR3; return true; }
    if (strcmp(s, "RTR5") == 0)                            { *out = J5_PROFILE_RTR5; return true; }
    if (strcmp(s, "BB")   == 0 || strcmp(s, "B-B")   == 0){ *out = J5_PROFILE_BB;   return true; }
    if (strcmp(s, "BCB")  == 0 || strcmp(s, "B-C-B") == 0){ *out = J5_PROFILE_BCB;  return true; }
    return false;
}

/* =========================================================
 * Comando dispatcher
 * ========================================================= */

static void uart_process_command(uint32_t seq, const char *cmd)
{
    /* --- Controllo stato macchina --- */
    if (strncmp(cmd, "STOP", 4) == 0)
    {
        state_machine_set_stopped();
        pickplace_safe_off();   /* sicurezza: spegne valvola e motori vuoto */
        uart_send_response_with_seq(seq, "OK STOP");
    }
    else if (strcmp(cmd, "SAFE") == 0 || strcmp(cmd, "RESET") == 0)
    {
        (void)state_machine_set_safe();
        pickplace_safe_off();   /* sicurezza: spegne valvola e motori vuoto */
        uart_send_response_with_seq(seq, "OK SAFE");
    }
    else if (strncmp(cmd, "ENABLE", 6) == 0)
    {
        uart_send_response_with_seq(seq,
            state_machine_set_idle() ? "OK ENABLE" : "ERR ENABLE");
    }
    else if (strncmp(cmd, "STATUS?", 7) == 0)
    {
        const char *s;
        switch (state_machine_get_state())
        {
            case STATE_SAFE:    s = "STATUS:SAFE";    break;
            case STATE_IDLE:    s = "STATUS:IDLE";    break;
            case STATE_STOPPED: s = "STATUS:STOPPED"; break;
            default:            s = "STATUS:SAFE";    break; /* fallback difensivo: mai UNKNOWN verso Pi */
        }
        uart_send_response_with_seq(seq, s);
    }
    /* --- HYBRID mode control --- */
    else if (strcmp(cmd, "HYBRID ENABLE") == 0)
    {
        hybrid_enabled = true;
        uart_send_response_with_seq(seq, "OK HYBRID ENABLED");
    }
    else if (strcmp(cmd, "HYBRID DISABLE") == 0)
    {
        hybrid_enabled = false;
        uart_send_response_with_seq(seq, "OK HYBRID DISABLED");
    }
    /* --- IMU --- */
    else if (strcmp(cmd, "IMUON") == 0)
    {
        g_imu_reads_enabled = 1;
        uart_send_response_with_seq(seq, "OK IMUON");
    }
    else if (strcmp(cmd, "IMUOFF") == 0)
    {
        g_imu_reads_enabled = 0;
        imu_clear_orientation_state();
        uart_send_response_with_seq(seq, "OK IMUOFF");
    }
    /* --- Diagnostica VR --- */
    else if (strncmp(cmd, "VR?", 3) == 0)
    {
        struct j5vr_state s = g_j5vr_latest;
        const bool grip_l  = (s.buttons_left  & (1U << 1)) != 0U;
        const bool grip_r  = (s.buttons_right & (1U << 1)) != 0U;

        char msg[320];
        snprintf(msg, sizeof(msg),
                 "VR:state=%u ticks=%lu stage=%lu arm=%lu freeze=%lu guard=%lu"
                 " input_active=%lu imu=%lu/%lu fatal=%lu/%lu"
                 " mode=%u hb=%u deadman=%u allowed=%u"
                 " joy=%d,%d pitch=%d yaw=%d int=%u btnL=%04X btnR=%04X"
                 " calls=%lu inc_mdeg(y,p,s,g)=%d,%d,%d,%d"
                 " ang(S,P,Y)=%u,%u,%u pwm_us(S,P,Y)=%u,%u,%u\n",
                 (unsigned)state_machine_get_state(),
                 (unsigned long)g_rt_loop_ticks,
                 (unsigned long)(uint32_t)g_rt_loop_stage,
                 (unsigned long)(uint32_t)g_vr_armed,
                 (unsigned long)(uint32_t)g_vr_freeze_active,
                 (unsigned long)g_vr_guard_block_count,
                 (unsigned long)(uint32_t)g_vr_input_active,
                 (unsigned long)g_imu_thread_ticks,
                 (unsigned long)(uint32_t)g_imu_thread_stage,
                 (unsigned long)g_fatal_count,
                 (unsigned long)g_fatal_reason,
                 (unsigned)s.mode,
                 (unsigned)s.vr_heartbeat,
                 (unsigned)((grip_l && grip_r) ? 1U : 0U),
                 (unsigned)(state_machine_is_movement_allowed() ? 1U : 0U),
                 (int)s.joy_x, (int)s.joy_y, (int)s.pitch, (int)s.yaw,
                 (unsigned)s.intensity,
                 (unsigned)s.buttons_left, (unsigned)s.buttons_right,
                 (unsigned long)g_j5vr_apply_incr_calls,
                 (int)g_j5vr_last_inc_mdeg_yaw,   (int)g_j5vr_last_inc_mdeg_pitch,
                 (int)g_j5vr_last_inc_mdeg_spalla, (int)g_j5vr_last_inc_mdeg_gomito,
                 (unsigned)servo_get_angle(SERVO_SPALLA),
                 (unsigned)servo_get_angle(SERVO_PITCH),
                 (unsigned)servo_get_angle(SERVO_YAW),
                 (unsigned)servo_get_last_pulse_us(SERVO_SPALLA),
                 (unsigned)servo_get_last_pulse_us(SERVO_PITCH),
                 (unsigned)servo_get_last_pulse_us(SERVO_YAW));
        uart_send_response_with_seq(seq, msg);
    }
    /* --- Pose commands --- */
    else if (strcmp(cmd, "HOME") == 0)
    {
        if (state_machine_get_state() != STATE_IDLE)
        {
            uart_send_response_with_seq(seq, "ERR BUSY");
            return;
        }
        uart_send_response_with_seq(seq, "OK HOME");
        j5vr_center_all_servos();
    }
    else if (strcmp(cmd, "PARK") == 0)
    {
        if (state_machine_get_state() != STATE_IDLE)
        {
            uart_send_response_with_seq(seq, "ERR BUSY");
            return;
        }
        uart_send_response_with_seq(seq, "OK PARK");
        j5vr_go_teleop_pose();
    }
    else if (strcmp(cmd, "TELEOPPOSE") == 0)
    {
        if (state_machine_get_state() != STATE_IDLE)
        {
            uart_send_response_with_seq(seq, "ERR BUSY");
            return;
        }
        uart_send_response_with_seq(seq, "OK TELEOPPOSE");
        j5vr_go_teleop_pose();
    }
    else if (strcmp(cmd, "HEADZERO") == 0)
    {
        /* Reimposta offset/EMA/centro del loop HEAD (vale anche per HYBRID)
         * senza cambiare modalità corrente. */
        j5vr_reset_head_calib();
        uart_send_response_with_seq(seq, "OK HEADZERO");
    }
    else if (strcmp(cmd, "RELAX_DIGITAL") == 0)
    {
        /* Rilascia (PWM=0) i servo digitali PITCH e ROLL. Usato dal Pi dopo
         * SETPOSE_DONE di HOME per ridurre il surriscaldamento dei due giunti
         * più stressati. Gli altri servo restano ingaggiati. */
        servo_relax_digital();
        uart_send_response_with_seq(seq, "OK RELAX_DIGITAL");
    }
    /* --- Pick & Place: PP1/PP2 <duty 0..100>, PP? per stato ---
     * PP1 = elettrovalvola (PA0 TIM2_CH1).
     * PP2 = vacuum motors (PA1 TIM2_CH2).
     * Comandi config-only: NON gated dalla state machine (parità con SET_*).
     * Lo spegnimento di sicurezza è già nei rami STOP/SAFE/RESET. */
    else if ((strncmp(cmd, "PP1 ", 4) == 0) || (strncmp(cmd, "PP2 ", 4) == 0))
    {
        uint8_t channel = (cmd[2] == '1') ? PICKPLACE_CH_PP1 : PICKPLACE_CH_PP2;
        const char *p = cmd + 4;
        unsigned int duty = 0U;
        if (!parse_uint(&p, &duty)) {
            uart_send_response_with_seq(seq,
                (channel == PICKPLACE_CH_PP1) ? "ERR PP1_FMT" : "ERR PP2_FMT");
            return;
        }
        skip_spaces(&p);
        if (*p != '\0') {
            uart_send_response_with_seq(seq,
                (channel == PICKPLACE_CH_PP1) ? "ERR PP1_FMT" : "ERR PP2_FMT");
            return;
        }
        if (duty > 100U) {
            uart_send_response_with_seq(seq,
                (channel == PICKPLACE_CH_PP1) ? "ERR PP1_RANGE" : "ERR PP2_RANGE");
            return;
        }
        if (!pickplace_set_duty(channel, (uint8_t)duty)) {
            uart_send_response_with_seq(seq,
                (channel == PICKPLACE_CH_PP1) ? "ERR PP1_HW" : "ERR PP2_HW");
            return;
        }
        char ack[24];
        snprintf(ack, sizeof(ack), "OK PP%u %u",
                 (unsigned)(channel + 1U), (unsigned)duty);
        uart_send_response_with_seq(seq, ack);
    }
    else if (strcmp(cmd, "PP?") == 0)
    {
        char msg[32];
        snprintf(msg, sizeof(msg), "OK PP %u %u",
                 (unsigned)pickplace_get_duty(PICKPLACE_CH_PP1),
                 (unsigned)pickplace_get_duty(PICKPLACE_CH_PP2));
        uart_send_response_with_seq(seq, msg);
    }
    /* --- SETPOSE_T_HR <B*10 S*10 G*10 Y*10 P*10 R*10 time_ms PLANNER> ---
     * High-resolution: angoli moltiplicati per 10 (50–1750 = 5.0°–175.0°).
     * Permette setpoint sub-degree per movimenti fluidi su servo digitali. */
    else if (strncmp(cmd, "SETPOSE_T_HR ", 13) == 0)
    {
        if (state_machine_get_state() != STATE_IDLE)
        {
            uart_send_response_with_seq(seq, "ERR BUSY");
            return;
        }
        const char *p = cmd + 13;
        unsigned int args[7];
        bool fmt_ok = true;
        for (int i = 0; i < 7 && fmt_ok; i++)
        {
            fmt_ok = parse_uint(&p, &args[i]);
        }
        char planner_str[8] = {0};
        if (fmt_ok) { fmt_ok = parse_alpha(&p, planner_str, sizeof(planner_str)); }
        if (fmt_ok) { skip_spaces(&p); fmt_ok = (*p == '\0'); }
        if (!fmt_ok)
        {
            uart_send_response_with_seq(seq, "ERR SETPOSE_T_HR_FMT");
            return;
        }
        for (int i = 0; i < 6; i++)
        {
            if (args[i] < 50U || args[i] > 1750U)   /* 5.0..175.0° × 10 */
            {
                uart_send_response_with_seq(seq, "ERR SETPOSE_T_HR_RANGE");
                return;
            }
        }
        unsigned int time_ms = args[6];
        if (time_ms < 20U) { time_ms = 20U; }

        j5_profile_t prof;
        if (!planner_from_string(planner_str, &prof))
        {
            uart_send_response_with_seq(seq, "ERR SETPOSE_T_HR_PLN");
            return;
        }
        float q_target[6];
        for (int i = 0; i < 6; i++) { q_target[i] = (float)args[i] / 10.0f; }
        uart_send_response_with_seq(seq, "OK SETPOSE_T_HR");
        j5vr_go_setpose_time_f(q_target, 6, (uint32_t)time_ms, prof);
    }
    /* --- SETPOSE_T <B S G Y P R time_ms PLANNER> --- */
    else if (strncmp(cmd, "SETPOSE_T", 9) == 0 && cmd[9] == ' ')
    {
        if (state_machine_get_state() != STATE_IDLE)
        {
            uart_send_response_with_seq(seq, "ERR BUSY");
            return;
        }

        const char *p = cmd + 9;
        unsigned int args[7];
        bool fmt_ok = true;
        for (int i = 0; i < 7 && fmt_ok; i++)
        {
            fmt_ok = parse_uint(&p, &args[i]);
        }
        char planner_str[8] = {0};
        if (fmt_ok) { fmt_ok = parse_alpha(&p, planner_str, sizeof(planner_str)); }
        if (fmt_ok) { skip_spaces(&p); fmt_ok = (*p == '\0'); }

        if (!fmt_ok)
        {
            uart_send_response_with_seq(seq, "ERR SETPOSE_T_FMT");
            return;
        }
        for (int i = 0; i < 6; i++)
        {
            if (args[i] < 5U || args[i] > 175U)
            {
                uart_send_response_with_seq(seq, "ERR SETPOSE_T_RANGE");
                return;
            }
        }
        unsigned int time_ms = args[6];
        if (time_ms < 20U) { time_ms = 20U; }

        j5_profile_t prof;
        if (!planner_from_string(planner_str, &prof))
        {
            uart_send_response_with_seq(seq, "ERR SETPOSE_T_PLN");
            return;
        }
        uint32_t q_target[6];
        for (int i = 0; i < 6; i++) { q_target[i] = args[i]; }

        uart_send_response_with_seq(seq, "OK SETPOSE_T");
        j5vr_go_setpose_time(q_target, 6, (uint32_t)time_ms, prof);
    }
    /* --- SETPOSE <B S G Y P R vel PLANNER> --- */
    else if (strncmp(cmd, "SETPOSE", 7) == 0 && cmd[7] == ' ')
    {
        if (state_machine_get_state() != STATE_IDLE)
        {
            uart_send_response_with_seq(seq, "ERR BUSY");
            return;
        }

        const char *p = cmd + 7;
        unsigned int args[7];
        bool fmt_ok = true;
        for (int i = 0; i < 7 && fmt_ok; i++)
        {
            fmt_ok = parse_uint(&p, &args[i]);
        }
        char planner_str[8] = {0};
        if (fmt_ok) { fmt_ok = parse_alpha(&p, planner_str, sizeof(planner_str)); }
        if (fmt_ok) { skip_spaces(&p); fmt_ok = (*p == '\0'); }

        if (!fmt_ok)
        {
            uart_send_response_with_seq(seq, "ERR SETPOSE_FMT");
            return;
        }
        /* Range: giunti [5, 175], vel [1, 100] */
        for (int i = 0; i < 6; i++)
        {
            if (args[i] < 5U || args[i] > 175U)
            {
                uart_send_response_with_seq(seq, "ERR SETPOSE_RANGE");
                return;
            }
        }
        if (args[6] < 1U || args[6] > 100U)
        {
            uart_send_response_with_seq(seq, "ERR SETPOSE_RANGE");
            return;
        }

        j5_profile_t prof;
        if (!planner_from_string(planner_str, &prof))
        {
            uart_send_response_with_seq(seq, "ERR SETPOSE_PLANNER");
            return;
        }
        /* ACK prima di armare la traiettoria */
        uart_send_response_with_seq(seq, "OK SETPOSE");
        j5vr_go_setpose(
            (uint8_t)args[0], (uint8_t)args[1], (uint8_t)args[2],
            (uint8_t)args[3], (uint8_t)args[4], (uint8_t)args[5],
            (uint8_t)args[6], prof);
    }
    /* --- SET_OFFSETS <B S G Y P R> ---
     * Config-only command: writes _servo_offset_deg[]. Does NOT trigger motion,
     * does NOT change state, does not affect the currently-running trajectory.
     * Allowed in any state (SAFE/IDLE/STOPPED) for parity with the other
     * SET_* config commands (SET_VR_PARAMS, SET_JOINT_LIMITS, SET_PWM_CONFIG),
     * so Pi-side boot-time sync can succeed BEFORE the first ENABLE transitions
     * the state machine to IDLE. The motion commands that READ these offsets
     * (HOME/PARK/TELEOPPOSE/SETPOSE) keep their own STATE_IDLE gate. */
    else if (strncmp(cmd, "SET_OFFSETS", 11) == 0 && cmd[11] == ' ')
    {
        const char *p = cmd + 11;
        unsigned int args[6];
        bool fmt_ok = true;
        for (int i = 0; i < 6 && fmt_ok; i++)
        {
            fmt_ok = parse_uint(&p, &args[i]);
        }
        if (fmt_ok) { skip_spaces(&p); fmt_ok = (*p == '\0'); }

        if (!fmt_ok)
        {
            uart_send_response_with_seq(seq, "ERR SET_OFFSETS_FMT");
            return;
        }
        for (int i = 0; i < 6; i++)
        {
            if (args[i] > 180U)
            {
                uart_send_response_with_seq(seq, "ERR SET_OFFSETS_RANGE");
                return;
            }
        }
        uint8_t new_offsets[SERVO_COUNT];
        for (int i = 0; i < SERVO_COUNT; i++) { new_offsets[i] = (uint8_t)args[i]; }
        servo_offset_set(new_offsets);
        uart_send_response_with_seq(seq, "OK SET_OFFSETS");
    }
    /* --- SET_JOINT_LIMITS <idx min_deg max_deg> --- */
    else if (strncmp(cmd, "SET_JOINT_LIMITS", 16) == 0 && cmd[16] == ' ')
    {
        const char *p = cmd + 16;
        unsigned int idx = 0U;
        float lim_min = 0.0f, lim_max = 0.0f;
        bool ok = parse_uint(&p, &idx)
               && parse_float(&p, &lim_min)
               && parse_float(&p, &lim_max);
        if (!ok || idx >= (unsigned int)SERVO_COUNT)
        {
            uart_send_response_with_seq(seq, "ERR SET_JOINT_LIMITS_FMT");
            return;
        }
        j5vr_set_joint_limits((int)idx, lim_min, lim_max);
        uart_send_response_with_seq(seq, "OK SET_JOINT_LIMITS");
    }
    /* --- SET_VR_PARAMS <16 float> [21 uint opzionali] --- */
    else if (strncmp(cmd, "SET_VR_PARAMS", 13) == 0 && cmd[13] == ' ')
    {
        const char *p = cmd + 13;
        float fargs[16];
        bool ok = true;
        for (int i = 0; i < 16 && ok; i++)
        {
            ok = parse_float(&p, &fargs[i]);
        }
        if (!ok)
        {
            uart_send_response_with_seq(seq, "ERR SET_VR_PARAMS_FMT");
            return;
        }
        /* Parametri interi opzionali:
         * [0-2]  src_roll src_pitch src_yaw
         * [3-5]  en_roll en_pitch en_yaw
         * [6-8]  vel_base vel_spalla vel_gomito
         * [9-11] vel_yaw vel_pitch vel_roll (polso Manual VR)
         * [12-14] vel_yaw_head vel_pitch_head vel_roll_head (polso HEAD/HYBRID)
         * [15-17] vel_base_head vel_spalla_head vel_gomito_head (braccio HEAD/HYBRID)
         */
        unsigned int iargs[18] = { 2U, 1U, 0U, 1U, 1U, 1U, 0U, 0U, 0U,
                                   0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
        for (int i = 0; i < 18; i++)
        {
            if (!parse_uint(&p, &iargs[i])) { break; }
        }
        j5vr_set_vr_params(fargs[0], fargs[1], fargs[2],
                           fargs[3], fargs[4], fargs[5],
                           fargs[6], fargs[7], fargs[8],
                           fargs[9], fargs[10], fargs[11],
                           fargs[12],  /* head_sensitivity */
                           fargs[13], fargs[14], fargs[15],  /* sign_yaw, pitch, roll */
                           (int)iargs[0], (int)iargs[1], (int)iargs[2],   /* src_roll/pitch/yaw */
                           (int)iargs[3], (int)iargs[4], (int)iargs[5],   /* en_roll/pitch/yaw */
                           (int)iargs[6], (int)iargs[7], (int)iargs[8],   /* vel_base/spalla/gomito */
                           (int)iargs[9], (int)iargs[10], (int)iargs[11], /* vel_yaw/pitch/roll */
                           (int)iargs[12], (int)iargs[13], (int)iargs[14], /* vel_yaw/pitch/roll head */
                           (int)iargs[15], (int)iargs[16], (int)iargs[17]); /* vel_base/spalla/gomito head */
#if UART_VR_PARAMS_DEBUG
        printk("[VR_PARAMS] SET_VR_PARAMS ok: gy=%.2f gp=%.2f gr=%.2f sens=%.2f "
               "sign_ypr=%d %d %d src_rpy=%u %u %u (HEADZERO è solo calib, non tuning)\n",
               (double)fargs[0], (double)fargs[1], (double)fargs[2], (double)fargs[12],
               (int)fargs[13], (int)fargs[14], (int)fargs[15],
               (unsigned)iargs[0], (unsigned)iargs[1], (unsigned)iargs[2]);
#endif
        uart_send_response_with_seq(seq, "OK SET_VR_PARAMS");
    }
    /* --- SET_PWM_CONFIG <t8_hz> <t8_min_us> <t8_max_us> <t8_max_deg>
     *                     <t1_hz> <t1_min_us> <t1_max_us> <t1_max_deg> --- */
    else if (strncmp(cmd, "SET_PWM_CONFIG", 14) == 0 && cmd[14] == ' ')
    {
        const char *p = cmd + 14;
        unsigned int args[8] = {50U, 500U, 2500U, 180U, 50U, 500U, 2500U, 180U};
        bool ok = true;
        for (int i = 0; i < 8 && ok; i++)
        {
            ok = parse_uint(&p, &args[i]);
        }
        if (!ok)
        {
            uart_send_response_with_seq(seq, "ERR SET_PWM_CONFIG_FMT");
            return;
        }
        servo_set_pwm_config(args[0], args[1], args[2], args[3],
                             args[4], args[5], args[6], args[7]);
        uart_send_response_with_seq(seq, "OK SET_PWM_CONFIG");
    }
    else
    {
        uart_send_response_with_seq(seq, "ERR UNKNOWN");
    }
}

/* =========================================================
 * Public API
 * ========================================================= */

void uart_control_init(void)
{
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
    if (!device_is_ready(uart_dev))
    {
        printk("[UART] UART1 (control plane) non pronto\n");
        uart_dev = NULL;
        return;
    }
    ring_buf_init(&uart_rx_ring, UART_RX_RING_SIZE, uart_rx_ring_storage);
    uart_irq_callback_user_data_set(uart_dev, uart_rx_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    memset(uart_rx_buffer, 0, UART_RX_BUF_SIZE);
    uart_rx_idx = 0;
    printk("[UART] UART1 (PA9/PA10) control plane ok\n");
}

/** uart_control_process — legge il ring buffer e dispatcha i comandi completi (riga per riga). */
void uart_control_process(void)
{
    if (uart_dev == NULL) { return; }

    unsigned char c;
    while (ring_buf_get(&uart_rx_ring, &c, 1) == 1)
    {
        if (c == '\n' || c == '\r')
        {
            if (uart_rx_idx > 0)
            {
                uart_rx_buffer[uart_rx_idx] = '\0';
                uint32_t seq;
                uart_parse_seq_and_cmd(uart_rx_buffer, &seq, uart_cmd_parsed);
                uart_process_command(seq, uart_cmd_parsed);
                uart_rx_idx = 0;
                memset(uart_rx_buffer, 0, UART_RX_BUF_SIZE);
            }
        }
        else if (uart_rx_idx < (UART_RX_BUF_SIZE - 1))
        {
            uart_rx_buffer[uart_rx_idx++] = c;
        }
        else
        {
            /* Buffer overflow: scarta riga */
            uart_rx_idx = 0;
            memset(uart_rx_buffer, 0, UART_RX_BUF_SIZE);
        }
    }
}
