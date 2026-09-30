/*
 * j5_protocol.c — JONNY5 SPI Protocol
 *
 * Parsing payload J5VR, telemetria servo/IMU, costruzione frame TX.
 *
 * NOTE [Refactor-Phase1]:
 *   - Le funzioni che definiscono il layout on-wire (j5_build_frame,
 *     j5vr_parse_payload, helper di endianness) NON devono essere cambiate
 *     in firme o logica.
 *   - Eventuali estensioni diagnostiche devono preservare esattamente i
 *     layout dei frame esistenti; è consentito solo aggiungere commenti o
 *     documentare funzioni marcate come DUBBIO/DIAGNOSTIC_ONLY nei report.
 */

#include "spi/j5_protocol.h"
#include "imu/imu.h"
#include "servo/servo_control.h"
#include "core/rt_loop.h"
#include "core/state_machine.h"
#include "core/estop.h"
#include "spi/hal_spi_slave.h"
#include "servo/j5vr_setpose.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/spinlock.h>

/* Dump per-frame [VR_RX]/[IK_RX] su console: printk SINCRONI nel thread SPI
 * service (rate-limited 1 ogni 50 frame). Pura diagnostica: a 0 sono compilati
 * fuori (console pulita, zero costo sul path SPI). Mettere a 1 per riabilitarli. */
#ifndef J5_PROTO_RX_DEBUG
#define J5_PROTO_RX_DEBUG 0
#endif
#include <string.h>

/* =========================================================
 * STATE
 * ========================================================= */

/** Ultimo stato J5VR ricevuto (aggiornato a ogni frame J5_FRAME_TYPE_J5VR). */
struct j5vr_state g_j5vr_latest = {
    .mode           = 0,
    .joy_x          = 0,
    .joy_y          = 0,
    .pitch          = 0,
    .yaw            = 0,
    .intensity      = 0,
    .grip           = 0,
    .vr_heartbeat   = 0,
    .priority       = 0,
    .safe_mask      = 0,
    .quat_w         = 1.0f,
    .quat_x         = 0.0f,
    .quat_y         = 0.0f,
    .quat_z         = 0.0f,
    .buttons_left   = 0,
    .buttons_right  = 0,
    .mode5_arm_valid = 0,
    .mode5_control_flags = 0,
    .mode5_target_id = 0,
    .mode5_arm_target_cdeg = {0, 0, 0},
};

/* Protegge g_j5vr_latest: writer = thread SPI service, lettori = RT loop / UART. */
static struct k_spinlock g_j5vr_lock;

void j5vr_latest_snapshot(struct j5vr_state *out)
{
    if (out == NULL) { return; }
    k_spinlock_key_t key = k_spin_lock(&g_j5vr_lock);
    *out = g_j5vr_latest;
    k_spin_unlock(&g_j5vr_lock, key);
}

void j5vr_latest_set_buttons_xy(uint16_t xy_bits)
{
    const uint16_t mask = (uint16_t)((1U << 4) | (1U << 5));
    k_spinlock_key_t key = k_spin_lock(&g_j5vr_lock);
    g_j5vr_latest.buttons_left =
        (uint16_t)((g_j5vr_latest.buttons_left & ~mask) | (xy_bits & mask));
    k_spin_unlock(&g_j5vr_lock, key);
}

struct j5ik_state g_j5ik_latest = {
    .valid         = 0,
    .control_flags = 0,
    .target_id     = 0,
    .vr_heartbeat  = 0,
    .mode          = 0,
    .target_cdeg   = {0, 0, 0, 0, 0, 0},
};

volatile uint32_t g_j5ik_rx_counter = 0;
volatile uint16_t g_j5vr_last_rx_seq = 0;

/* Istante dell'ultimo J5IK (k_uptime_get_32). 0 = mai ricevuto. */
static atomic_t g_j5ik_last_rx_ms = ATOMIC_INIT(0);

void j5ik_latest_snapshot(struct j5ik_state *out)
{
    if (out == NULL) { return; }
    k_spinlock_key_t key = k_spin_lock(&g_j5vr_lock);
    *out = g_j5ik_latest;
    k_spin_unlock(&g_j5vr_lock, key);
}

uint32_t j5ik_last_rx_ms(void)
{
    return (uint32_t)atomic_get(&g_j5ik_last_rx_ms);
}

/* =========================================================
 * Codec helpers (endianness)
 * ========================================================= */

/** Serializza float32 in formato big-endian (IEEE 754). */
static void float_to_be32(float f, uint8_t *p)
{
    union { uint32_t u32; float f32; } u;
    u.f32 = f;
    p[0] = (uint8_t)(u.u32 >> 24);
    p[1] = (uint8_t)(u.u32 >> 16);
    p[2] = (uint8_t)(u.u32 >>  8);
    p[3] = (uint8_t)(u.u32);
}

/** Deserializza int16 big-endian. Cast a uint16_t prima dello shift per evitare UB. */
static int16_t be16_to_s16(const uint8_t *p)
{
    uint16_t u = (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
    return (int16_t)u;
}

/** Deserializza uint16 big-endian. */
static uint16_t be16_to_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/** Deserializza float32 big-endian (IEEE 754). */
static float be32_to_float(const uint8_t *p)
{
    union { uint32_t u32; float f32; } u;
    u.u32 = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
          | ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
    return u.f32;
}

/* =========================================================
 * TX telemetry (compact, payload offset 51-53)
 * Usata solo internamente da j5_build_frame per TELEMETRY.
 * ========================================================= */

static void fill_tx_telemetry_compact(uint8_t *payload)
{
    if (payload == NULL) { return; }
    payload[51] = 0;
    payload[52] = 0;
    payload[53] = 0;
}

/* =========================================================
 * RX parsing
 * ========================================================= */

/**
 * j5vr_parse_payload — deserializza 54 byte payload J5VR in g_j5vr_latest.
 *
 * Layout payload J5VR (54 byte):
 *   [0]     mode
 *   [1-2]   joy_x (BE s16)
 *   [3-4]   joy_y (BE s16)
 *   [5-6]   pitch (BE s16)
 *   [7-8]   yaw   (BE s16)
 *   [9]     intensity
 *   [10]    grip (legacy)
 *   [11-12] vr_heartbeat (BE u16)
 *   [13]    priority
 *   [14-15] safe_mask (BE u16)
 *   [16-31] quaternione W,X,Y,Z (4×float32 BE)
 *   [32-33] buttons_left  (BE u16)
 *   [34-35] buttons_right (BE u16)
 *   [36-45] estensione mode=5 arm-only (marker 'I') oppure payload legacy riservati
 */
void j5vr_parse_payload(const uint8_t *p)
{
    if (p == NULL) { return; }

    struct j5vr_state s;

    s.mode          = p[0];
    s.joy_x         = be16_to_s16(p +  1);
    s.joy_y         = be16_to_s16(p +  3);
    s.pitch         = be16_to_s16(p +  5);
    s.yaw           = be16_to_s16(p +  7);
    s.intensity     = p[9];
    s.grip          = p[10];
    s.vr_heartbeat  = be16_to_u16(p + 11);
    s.priority      = p[13];
    s.safe_mask     = be16_to_u16(p + 14);
    s.quat_w        = be32_to_float(p + 16);
    s.quat_x        = be32_to_float(p + 20);
    s.quat_y        = be32_to_float(p + 24);
    s.quat_z        = be32_to_float(p + 28);
    s.buttons_left  = be16_to_u16(p + 32);
    s.buttons_right = be16_to_u16(p + 34);
    s.mode5_arm_valid = 0U;
    s.mode5_control_flags = 0U;
    s.mode5_target_id = 0U;
    s.mode5_arm_target_cdeg[0] = 0;
    s.mode5_arm_target_cdeg[1] = 0;
    s.mode5_arm_target_cdeg[2] = 0;

    if (s.mode == 5U && p[36] == (uint8_t)'I')
    {
        s.mode5_control_flags = p[37];
        s.mode5_arm_valid = (uint8_t)((p[37] & (1U << 0)) != 0U);
        s.mode5_target_id = be16_to_u16(p + 38);
        s.mode5_arm_target_cdeg[0] = be16_to_s16(p + 40);
        s.mode5_arm_target_cdeg[1] = be16_to_s16(p + 42);
        s.mode5_arm_target_cdeg[2] = be16_to_s16(p + 44);
    }

    /* Pubblicazione atomica: il RT loop non vede mai uno stato a meta'. */
    k_spinlock_key_t key = k_spin_lock(&g_j5vr_lock);
    g_j5vr_latest = s;
    k_spin_unlock(&g_j5vr_lock, key);

#if J5_PROTO_RX_DEBUG
    /* Log rate-limited: conferma parse su STM32 (ogni 50 frame) */
    static uint32_t vr_rx_log = 0;
    if ((vr_rx_log++ % 50U) == 0U)
    {
        printk("[VR_RX] mode=%u buttons_L=%04X buttons_R=%04X"
               " joy_x=%d joy_y=%d pitch=%d yaw=%d intensity=%u hb=%u"
               " m5_valid=%u m5_id=%u m5_bsg=%d,%d,%d\n",
               (unsigned)g_j5vr_latest.mode,
               (unsigned)g_j5vr_latest.buttons_left,
               (unsigned)g_j5vr_latest.buttons_right,
               (int)g_j5vr_latest.joy_x,
               (int)g_j5vr_latest.joy_y,
               (int)g_j5vr_latest.pitch,
               (int)g_j5vr_latest.yaw,
               (unsigned)g_j5vr_latest.intensity,
               (unsigned)g_j5vr_latest.vr_heartbeat,
               (unsigned)g_j5vr_latest.mode5_arm_valid,
               (unsigned)g_j5vr_latest.mode5_target_id,
               (int)g_j5vr_latest.mode5_arm_target_cdeg[0],
               (int)g_j5vr_latest.mode5_arm_target_cdeg[1],
               (int)g_j5vr_latest.mode5_arm_target_cdeg[2]);
    }
#endif

}

void j5ik_parse_payload(const uint8_t *p)
{
    if (p == NULL) { return; }

    struct j5ik_state ik;
    ik.valid         = p[0];
    ik.control_flags = p[1];
    ik.target_id     = be16_to_u16(p + 2);
    ik.vr_heartbeat  = be16_to_u16(p + 4);
    ik.mode          = p[6];
    for (int i = 0; i < 6; i++)
    {
        ik.target_cdeg[i] = be16_to_s16(p + 8 + (i * 2));
    }

    /* Pubblicazione atomica (RT loop legge via j5ik_latest_snapshot) e
     * mode/hb coerenti anche nei diagnostici legacy. */
    {
        k_spinlock_key_t key = k_spin_lock(&g_j5vr_lock);
        g_j5ik_latest = ik;
        g_j5vr_latest.mode = ik.mode;
        g_j5vr_latest.vr_heartbeat = ik.vr_heartbeat;
        k_spin_unlock(&g_j5vr_lock, key);
    }
    g_j5ik_rx_counter++;
    atomic_set(&g_j5ik_last_rx_ms, (atomic_val_t)k_uptime_get_32());

#if J5_PROTO_RX_DEBUG
    static uint32_t ik_rx_log = 0;
    if ((ik_rx_log++ % 50U) == 0U)
    {
        printk("[IK_RX] valid=%u flags=0x%02X id=%u hb=%u mode=%u"
               " q(B,S,G,Y,P,R)cdeg=%d,%d,%d,%d,%d,%d\n",
               (unsigned)g_j5ik_latest.valid,
               (unsigned)g_j5ik_latest.control_flags,
               (unsigned)g_j5ik_latest.target_id,
               (unsigned)g_j5ik_latest.vr_heartbeat,
               (unsigned)g_j5ik_latest.mode,
               (int)g_j5ik_latest.target_cdeg[0],
               (int)g_j5ik_latest.target_cdeg[1],
               (int)g_j5ik_latest.target_cdeg[2],
               (int)g_j5ik_latest.target_cdeg[3],
               (int)g_j5ik_latest.target_cdeg[4],
               (int)g_j5ik_latest.target_cdeg[5]);
    }
#endif
}

/* =========================================================
 * TX telemetry (pubblico — per frame STATUS da hal_spi_slave)
 * ========================================================= */

/**
 * j5vr_fill_tx_telemetry — scrive diagnostica nei byte 46-53 del payload TX.
 *
 * Layout offset 46-53:
 *   46-47: vr_heartbeat (BE)
 *   48:    mode
 *   49:    0x00 (riservato)
 *   50-51: diag_mask BE (bit0=deadman, 1=input_active, 2=armed, 3=freeze, 4=guard_seen)
 *   52-53: 0x00 (riservato)
 */
void j5vr_fill_tx_telemetry(uint8_t *payload)
{
    if (payload == NULL) { return; }

    payload[46] = (uint8_t)(g_j5vr_latest.vr_heartbeat >> 8);
    payload[47] = (uint8_t)(g_j5vr_latest.vr_heartbeat & 0xFF);
    payload[48] = g_j5vr_latest.mode;
    payload[49] = 0x00;

    {
        const bool grip_l  = (g_j5vr_latest.buttons_left  & (1U << 1)) != 0U;
        const bool grip_r  = (g_j5vr_latest.buttons_right & (1U << 1)) != 0U;
        const bool deadman = grip_l && grip_r;
        uint16_t diag = 0;
        if (deadman)                      { diag |= (1U << 0); }
        if (g_vr_input_active)            { diag |= (1U << 1); }
        if (g_vr_armed)                   { diag |= (1U << 2); }
        if (g_vr_freeze_active)           { diag |= (1U << 3); }
        if (g_vr_guard_block_count != 0U) { diag |= (1U << 4); }
        payload[50] = (uint8_t)(diag >> 8);
        payload[51] = (uint8_t)(diag & 0xFF);
    }
    payload[52] = 0;
    payload[53] = 0;
}

/* =========================================================
 * TX frame builder
 * ========================================================= */

/**
 * j5_build_frame — costruisce un frame TX valido di 64 byte.
 *
 * Per J5_FRAME_TYPE_TELEMETRY riempie il payload con:
 *   [0-27]  IMU raw (accel_x/y/z, gyro_x/y/z, temp — 7×float32 BE)
 *   [28]    imu_valid (1 se snapshot ok e orientamento valido, 0 altrimenti)
 *   [29-44] quaternione orientamento IMU (W,X,Y,Z — 4×float32 BE)
 *   [45-50] angoli servo in gradi uint8 (B, S, G, Y, P, R)
 *   [51-53] IMU sample_counter LSB24 (debug instrumentation)
 *
 * Per tutti gli altri tipi il payload è zero; il chiamante può sovrascrivere
 * i byte necessari dopo la chiamata.
 */
void j5_build_frame(j5_frame_t *frame, j5_frame_type_t type, uint16_t seq)
{
    bool have_imu_sample_counter = false;
    uint32_t imu_sample_counter = 0;

    memset(frame, 0, sizeof(j5_frame_t));
    frame->header[0]        = 'J';
    frame->header[1]        = '5';
    frame->protocol_version = 1;
    frame->frame_type       = (uint8_t)type;
    frame->sequence_counter = __builtin_bswap16(seq);
    frame->payload_len      = J5_PROTOCOL_FRAME_SIZE;
    frame->flags            = 0;

    if (type != J5_FRAME_TYPE_TELEMETRY)
    {
        /* Payload zero: il chiamante aggiunge i propri campi se necessario */
        return;
    }

    /* --- TELEMETRY payload --- */

    if (!g_imu_reads_enabled)
    {
        /* IMUOFF: imu_valid=0, quaternione identità */
        frame->payload[28] = 0u;
        float_to_be32(1.0f, frame->payload + 29);
        /* quat x/y/z restano 0 per memset */
    }
    else if (imu_is_available())
    {
        imu_snapshot_t snap;
        const bool snap_ok = imu_get_snapshot(&snap);

        if (snap_ok)
        {
            float_to_be32(snap.accel_x, frame->payload +  0);
            float_to_be32(snap.accel_y, frame->payload +  4);
            float_to_be32(snap.accel_z, frame->payload +  8);
            float_to_be32(snap.gyro_x,  frame->payload + 12);
            float_to_be32(snap.gyro_y,  frame->payload + 16);
            float_to_be32(snap.gyro_z,  frame->payload + 20);
            float_to_be32(snap.temp,    frame->payload + 24);
        }

        /* imu_valid = 1 solo se snapshot presente e quaternione BNO085 valido */
        frame->payload[28] = (snap_ok && imu_is_orientation_valid()) ? 1u : 0u;

        if (snap_ok)
        {
            float_to_be32(snap.quat_w, frame->payload + 29);
            float_to_be32(snap.quat_x, frame->payload + 33);
            float_to_be32(snap.quat_y, frame->payload + 37);
            float_to_be32(snap.quat_z, frame->payload + 41);
            have_imu_sample_counter = true;
            imu_sample_counter = snap.sample_counter;
        }
    }

    /* Angoli servo (0-180°) per debug UI */
    frame->payload[45] = servo_get_angle(SERVO_BASE);
    frame->payload[46] = servo_get_angle(SERVO_SPALLA);
    frame->payload[47] = servo_get_angle(SERVO_GOMITO);
    frame->payload[48] = servo_get_angle(SERVO_YAW);
    frame->payload[49] = servo_get_angle(SERVO_PITCH);
    frame->payload[50] = servo_get_angle(SERVO_ROLL);

    /* Diagnostica compatta: byte 51-53 */
    fill_tx_telemetry_compact(frame->payload);
    if (have_imu_sample_counter) {
        /* IMU instrumentation (debug): sample counter LSB24 in payload[51..53]. */
        frame->payload[51] = (uint8_t)((imu_sample_counter >> 16) & 0xFFu);
        frame->payload[52] = (uint8_t)((imu_sample_counter >> 8) & 0xFFu);
        frame->payload[53] = (uint8_t)(imu_sample_counter & 0xFFu);
    }

    /* Periodo runtime del RT loop in microsecondi (EWMA) nei 2 byte reserved.
     * Layout BE uint16 in frame->reserved[0..1] (offset assoluto 62-63).
     * Il Pi calcola loop_hz = 1e6 / loop_period_us. Valore 0 = warm-up.
     * Backward compat: parser legacy ignoravano reserved (sempre 0). */
    {
        uint16_t period = g_rt_loop_period_us;
        frame->reserved[0] = (uint8_t)((period >> 8) & 0xFFu);
        frame->reserved[1] = (uint8_t)(period & 0xFFu);
    }
}

/* =========================================================
 * Protocollo v2: CRC + TELEMETRY_V2
 * ========================================================= */

static void u16_to_be(uint16_t v, uint8_t *p)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFU);
}

static void u32_to_be(uint32_t v, uint8_t *p)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v & 0xFFU);
}

/** float -> int16 con scala e saturazione. */
static int16_t f_to_i16_scaled(float v, float scale)
{
    float x = v * scale;
    if (!(x == x)) { return 0; }           /* NaN */
    if (x > 32767.0f)  { return 32767; }
    if (x < -32768.0f) { return -32768; }
    return (int16_t)((x >= 0.0f) ? (x + 0.5f) : (x - 0.5f));
}

static uint16_t sat_u16(uint32_t v)
{
    return (v > 0xFFFFU) ? 0xFFFFU : (uint16_t)v;
}

uint16_t j5_crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    if (data == NULL) { return crc; }
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++)
        {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

bool j5_frame_v2_crc_ok(const uint8_t *frame64)
{
    if (frame64 == NULL) { return false; }
    const uint16_t want = j5_crc16_ccitt(frame64, J5_PROTOCOL_FRAME_SIZE - 2U);
    const uint16_t got  = be16_to_u16(frame64 + (J5_PROTOCOL_FRAME_SIZE - 2U));
    return want == got;
}

void j5_frame_seal_v2(j5_frame_t *frame)
{
    if (frame == NULL) { return; }
    frame->protocol_version = (uint8_t)J5_PROTOCOL_VERSION_V2;
    frame->flags            = (uint8_t)J5_FLAG_CRC16;
    const uint16_t crc = j5_crc16_ccitt((const uint8_t *)frame, J5_PROTOCOL_FRAME_SIZE - 2U);
    u16_to_be(crc, frame->reserved);
}

/**
 * j5_build_telemetry_v2 — layout payload (BE, 54 byte):
 *   [0-3]   stm_time_ms (k_uptime_get_32)
 *   [4-5]   stm_tx_seq (contatore risposte TELEMETRY_V2, rileva risposte ripetute)
 *   [6-7]   rx_seq_gaps (frame Pi persi, saturato)
 *   [8]     fsm_state (0=SAFE, 1=IDLE, 2=STOPPED)
 *   [9]     status_flags (J5_TLM2_ST_*)
 *   [10]    mode attivo
 *   [11]    diag_flags (J5_TLM2_DG_*)
 *   [12-23] 6 x int16 posizione comandata fisica, centi-gradi (B S G Y P R)
 *   [24-31] quaternione IMU W X Y Z, int16 Q15
 *   [32-37] gyro X Y Z int16, mrad/s
 *   [38-43] accel X Y Z int16, cm/s^2
 *   [44-45] imu_sample_counter LSB16
 *   [46-47] rt_loop_period_us
 *   [48-49] rt_step_us
 *   [50-51] rt_overruns (saturato)
 *   [52-53] crc_errors RX (saturato)
 */
void j5_build_telemetry_v2(j5_frame_t *frame, uint16_t seq)
{
    static uint16_t stm_tx_seq = 0U;   /* solo thread SPI service */

    if (frame == NULL) { return; }
    memset(frame, 0, sizeof(j5_frame_t));
    frame->header[0]        = 'J';
    frame->header[1]        = '5';
    frame->frame_type       = (uint8_t)J5_FRAME_TYPE_TELEMETRY_V2;
    frame->sequence_counter = __builtin_bswap16(seq);
    frame->payload_len      = J5_PROTOCOL_FRAME_SIZE;

    uint8_t *p = frame->payload;
    stm_tx_seq++;
    u32_to_be(k_uptime_get_32(), p + 0);
    u16_to_be(stm_tx_seq, p + 4);
    u16_to_be(hal_spi_rx_seq_gaps(), p + 6);

    p[8] = (uint8_t)state_machine_get_state();

    struct j5vr_state vr;
    j5vr_latest_snapshot(&vr);
    const bool deadman = ((vr.buttons_left & (1U << 1)) != 0U) &&
                         ((vr.buttons_right & (1U << 1)) != 0U);
    uint8_t st = 0U;
    if (estop_is_active())                   { st |= J5_TLM2_ST_ESTOP; }
    if (state_machine_is_movement_allowed()) { st |= J5_TLM2_ST_MOVE_ALLOWED; }
    if (deadman)                             { st |= J5_TLM2_ST_DEADMAN; }
    if (g_vr_input_active)                   { st |= J5_TLM2_ST_INPUT; }
    if (g_vr_armed)                          { st |= J5_TLM2_ST_ARMED; }
    if (g_vr_freeze_active)                  { st |= J5_TLM2_ST_FREEZE; }
    if (j5vr_setpose_is_active())            { st |= J5_TLM2_ST_SETPOSE; }
    p[10] = vr.mode;

    uint8_t dg = 0U;
    if (g_vr_guard_block_count != 0U) { dg |= J5_TLM2_DG_GUARD_SEEN; }
    if (g_imu_reads_enabled)          { dg |= J5_TLM2_DG_IMU_ENABLED; }
    if (g_j5ik_stream_active)         { dg |= J5_TLM2_DG_STREAM_LIVE; }
    if (j5vr_setpose_pose_known())    { dg |= J5_TLM2_DG_POSE_KNOWN; }

    for (int i = 0; i < 6; i++)
    {
        const int16_t cdeg = (int16_t)((int16_t)servo_get_angle((servo_joint_t)i) * 100);
        u16_to_be((uint16_t)cdeg, p + 12 + (i * 2));
    }

    /* Quaternione identita' di default */
    u16_to_be((uint16_t)32767, p + 24);
    if (g_imu_reads_enabled && imu_is_available())
    {
        dg |= J5_TLM2_DG_IMU_PRESENT;
        imu_snapshot_t snap;
        if (imu_get_snapshot(&snap))
        {
            u16_to_be((uint16_t)f_to_i16_scaled(snap.quat_w, 32767.0f), p + 24);
            u16_to_be((uint16_t)f_to_i16_scaled(snap.quat_x, 32767.0f), p + 26);
            u16_to_be((uint16_t)f_to_i16_scaled(snap.quat_y, 32767.0f), p + 28);
            u16_to_be((uint16_t)f_to_i16_scaled(snap.quat_z, 32767.0f), p + 30);
            u16_to_be((uint16_t)f_to_i16_scaled(snap.gyro_x, 1000.0f), p + 32);
            u16_to_be((uint16_t)f_to_i16_scaled(snap.gyro_y, 1000.0f), p + 34);
            u16_to_be((uint16_t)f_to_i16_scaled(snap.gyro_z, 1000.0f), p + 36);
            u16_to_be((uint16_t)f_to_i16_scaled(snap.accel_x, 100.0f), p + 38);
            u16_to_be((uint16_t)f_to_i16_scaled(snap.accel_y, 100.0f), p + 40);
            u16_to_be((uint16_t)f_to_i16_scaled(snap.accel_z, 100.0f), p + 42);
            u16_to_be((uint16_t)(snap.sample_counter & 0xFFFFU), p + 44);
            if (imu_is_orientation_valid()) { st |= J5_TLM2_ST_IMU_VALID; }
        }
    }
    p[9]  = st;
    p[11] = dg;

    u16_to_be(g_rt_loop_period_us, p + 46);
    u16_to_be(g_rt_step_us, p + 48);
    u16_to_be(sat_u16(rt_loop_get_overruns()), p + 50);
    u16_to_be(hal_spi_rx_crc_errors(), p + 52);

    j5_frame_seal_v2(frame);
}
