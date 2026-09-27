/*
 * j5_protocol.h — JONNY5 SPI Protocol
 *
 * Protocollo SPI deterministico a frame fisso 64 byte.
 * Frame atomici con header riconoscibile ('J','5') e sequence counter
 * big-endian monotono.
 *
 * Layout frame (j5_frame_t, 64 byte packed):
 *   [0-1]  header: 'J' '5'
 *   [2]    protocol_version = 1
 *   [3]    frame_type (j5_frame_type_t)
 *   [4-5]  sequence_counter (BE)
 *   [6]    payload_len = 64
 *   [7]    flags = 0
 *   [8-61] payload[54]
 *   [62-63] reserved = 0
 *
 * Protocollo v2 (stesso frame 64 byte, vedi ros2_ws/docs/SPI_PROTOCOL_V2.md):
 *   [2]     protocol_version = 2
 *   [7]     flags = J5_FLAG_CRC16
 *   [62-63] CRC-16/CCITT-FALSE (BE) sui byte 0..61
 * Il firmware accetta v1 e v2 e risponde con la stessa versione della
 * richiesta. In v2 le richieste J5VR/J5IK/TELEMETRY ricevono TELEMETRY_V2.
 */

#ifndef J5_PROTOCOL_H
#define J5_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 * Costanti
 * ========================================================= */

#define J5_PROTOCOL_FRAME_SIZE 64   /**< Dimensione frame in byte (fissa) */
#define J5VR_PAYLOAD_LEN       54   /**< Byte payload disponibili         */

#define J5_PROTOCOL_VERSION_V1 1U
#define J5_PROTOCOL_VERSION_V2 2U
#define J5_FLAG_CRC16          0x01U /**< v2: CRC-16 nei byte 62-63        */

/** Mode J5IK: streaming di target articolari dal Pi (ROS 2 / ros2_control). */
#define J5_MODE_JOINT_STREAM   6U
/** J5IK control_flags bit7: consenso esplicito al movimento in JOINT_STREAM. */
#define J5IK_FLAG_STREAM_ENABLE (1U << 7)
/** Oltre questo intervallo senza frame J5IK il braccio resta fermo (hold). */
#define J5IK_STREAM_TIMEOUT_MS 100U
/** Velocita' massima [deg/s] nel mode JOINT_STREAM (indipendente dallo stato VR),
 *  ulteriormente limitata dai cap per-giunto joint_max_vel_deg_s (polso 35). */
#define J5_STREAM_MAX_VEL_DEG_S 60.0f

/* TELEMETRY_V2 status_flags (payload[9]) */
#define J5_TLM2_ST_ESTOP        (1U << 0)
#define J5_TLM2_ST_MOVE_ALLOWED (1U << 1)
#define J5_TLM2_ST_DEADMAN      (1U << 2)
#define J5_TLM2_ST_INPUT        (1U << 3)
#define J5_TLM2_ST_ARMED        (1U << 4)
#define J5_TLM2_ST_FREEZE       (1U << 5)
#define J5_TLM2_ST_SETPOSE      (1U << 6)
#define J5_TLM2_ST_IMU_VALID    (1U << 7)

/* TELEMETRY_V2 diag_flags (payload[11]) */
#define J5_TLM2_DG_GUARD_SEEN   (1U << 0)
#define J5_TLM2_DG_IMU_PRESENT  (1U << 1)
#define J5_TLM2_DG_IMU_ENABLED  (1U << 2)
#define J5_TLM2_DG_STREAM_LIVE  (1U << 3)

/* =========================================================
 * Tipi
 * ========================================================= */

/** Tipo frame SPI. I valori numerici fanno parte del protocollo su filo. */
typedef enum {
    J5_FRAME_TYPE_TELEMETRY = 0x01, /**< Telemetria IMU stimata + stato servo software-side */
    J5_FRAME_TYPE_TEST_ECHO = 0x02, /**< Echo diagnostico (loopback)     */
    J5_FRAME_TYPE_STATUS    = 0x03, /**< Status / ack generico            */
    J5_FRAME_TYPE_J5VR      = 0x04, /**< Payload comandi VR dal master   */
    J5_FRAME_TYPE_J5IK      = 0x05, /**< Payload target IK diretti       */
    J5_FRAME_TYPE_ASSIST_V2_CONTROL   = 0x06, /**< ASSIST v2 CONTROL (RAW / WIRE v1) */
    J5_FRAME_TYPE_ASSIST_V2_TELEMETRY = 0x07, /**< ASSIST v2 TELEMETRY echo          */
    J5_FRAME_TYPE_TELEMETRY_V2 = 0x08  /**< v2: telemetria compatta STM -> Pi (solo TX) */
} j5_frame_type_t;

/** Stato J5VR ricevuto: parsing payload → struttura C. Solo lettura dati, nessuna attuazione. */
struct j5vr_state {
    uint8_t  mode;
    int16_t  joy_x;
    int16_t  joy_y;
    int16_t  pitch;
    int16_t  yaw;
    uint8_t  intensity;
    uint8_t  grip;          /**< Legacy — parsato ma non usato; mantenuto per compatibilità strutturale */
    uint16_t vr_heartbeat;
    uint8_t  priority;
    uint16_t safe_mask;
    /* Quaternioni orientamento visore VR (offset 16-31, 4 × float32 BE) */
    float    quat_w;
    float    quat_x;
    float    quat_y;
    float    quat_z;
    /* Pulsanti joystick (offset 32-35, 2 × uint16 BE) */
    uint16_t buttons_left;  /**< bit0=trigger, 1=grip, 3=thumbstick, 4=X, 5=Y */
    uint16_t buttons_right; /**< bit0=trigger, 1=grip, 3=thumbstick, 4=A, 5=B */
    /* Estensione mode=5 nei byte riservati 36-45 del frame J5VR:
     *   [36]    marker 'I'
     *   [37]    control_flags (bit0=valid, bit1=grip_active, bit2=hold_active)
     *   [38-39] target_id (BE u16)
     *   [40-41] base   (BE s16, centi-gradi fisici)
     *   [42-43] spalla (BE s16, centi-gradi fisici)
     *   [44-45] gomito (BE s16, centi-gradi fisici)
     */
    uint8_t  mode5_arm_valid;
    uint8_t  mode5_control_flags;
    uint16_t mode5_target_id;
    int16_t  mode5_arm_target_cdeg[3];
};

/** Stato J5IK ricevuto: target 6-DOF già risolti lato Raspberry in angoli fisici centideg. */
struct j5ik_state {
    uint8_t  valid;
    uint8_t  control_flags;   /**< bit0=grip attivo, bit1=hold */
    uint16_t target_id;
    uint16_t vr_heartbeat;
    uint8_t  mode;
    int16_t  target_cdeg[6];  /**< Ordine B S G Y P R, centi-gradi fisici */
};

/** Ultimo frame J5VR ricevuto (aggiornato da j5vr_parse_payload). */
extern struct j5vr_state g_j5vr_latest;
extern struct j5ik_state g_j5ik_latest;
extern volatile uint32_t g_j5ik_rx_counter;
extern volatile uint16_t g_j5vr_last_rx_seq;

/** Frame strutturato 64 byte (packed). */
typedef struct __attribute__((packed)) {
    uint8_t  header[2];          /**< 'J' '5' (0x4A 0x35)   */
    uint8_t  protocol_version;   /**< = 1                    */
    uint8_t  frame_type;         /**< j5_frame_type_t        */
    uint16_t sequence_counter;   /**< Big endian             */
    uint8_t  payload_len;        /**< = 64                   */
    uint8_t  flags;              /**< = 0                    */
    uint8_t  payload[54];
    uint8_t  reserved[2];        /**< = 0                    */
} j5_frame_t;

_Static_assert(sizeof(j5_frame_t) == J5_PROTOCOL_FRAME_SIZE,
               "j5_frame_t must be exactly 64 bytes");

/* =========================================================
 * API
 * ========================================================= */

/**
 * j5_build_frame — costruisce un frame TX valido.
 * Per J5_FRAME_TYPE_TELEMETRY riempie il payload con snapshot IMU e angoli servo.
 * Per tutti gli altri tipi il payload viene azzerato (da riempire dal chiamante se necessario).
 */
/* Nota di contratto: i campi compatibili con la UI pubblicati come `servo_deg_*`
 * rappresentano command state/stato interno comandato del firmware, non misure
 * encoder fisiche del robot. */
void j5_build_frame(j5_frame_t *frame, j5_frame_type_t type, uint16_t seq);

/**
 * j5vr_parse_payload — parsing 54 byte payload J5VR; aggiorna g_j5vr_latest.
 * Non esegue alcuna attuazione.
 */
void j5vr_parse_payload(const uint8_t *p);
void j5ik_parse_payload(const uint8_t *p);

/**
 * j5vr_latest_snapshot — copia coerente di g_j5vr_latest.
 * Il writer (thread SPI service, prio 7) puo' essere prelazionato dal RT loop
 * (prio 4) a meta' aggiornamento: i lettori multi-campo devono usare questa
 * funzione invece di copiare direttamente la struct.
 */
void j5vr_latest_snapshot(struct j5vr_state *out);

/** j5vr_latest_set_buttons_xy — aggiorna in modo atomico i bit X/Y (4,5) di buttons_left. */
void j5vr_latest_set_buttons_xy(uint16_t xy_bits);

/** j5ik_latest_snapshot — copia coerente di g_j5ik_latest (stesso lock di g_j5vr_latest). */
void j5ik_latest_snapshot(struct j5ik_state *out);

/** Istante (k_uptime_get_32) dell'ultimo frame J5IK ricevuto; 0 = mai. */
uint32_t j5ik_last_rx_ms(void);

/* =========================================================
 * Protocollo v2
 * ========================================================= */

/** CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, xorout 0. */
uint16_t j5_crc16_ccitt(const uint8_t *data, uint32_t len);

/** true se il frame (64 byte) ha CRC v2 valido nei byte 62-63. */
bool j5_frame_v2_crc_ok(const uint8_t *frame64);

/** Converte un frame gia' costruito in v2: version=2, flags=CRC16, CRC in 62-63. */
void j5_frame_seal_v2(j5_frame_t *frame);

/**
 * j5_build_telemetry_v2 — frame TELEMETRY_V2 (0x08) sigillato con CRC.
 * Layout payload in ros2_ws/docs/SPI_PROTOCOL_V2.md.
 */
void j5_build_telemetry_v2(j5_frame_t *frame, uint16_t seq);

/**
 * j5vr_fill_tx_telemetry — scrive diagnostica nei byte 46-53 del payload TX.
 * Usato da hal_spi_slave per il frame STATUS.
 *
 * Layout offset 46-53:
 *   46-47: vr_heartbeat (BE)
 *   48:    mode
 *   49:    0x00 (riservato)
 *   50-51: diag_mask BE (bit0=deadman, 1=input_active, 2=armed, 3=freeze, 4=guard_seen)
 *   52-53: 0x00 (riservato)
 */
void j5vr_fill_tx_telemetry(uint8_t *payload);

#endif /* J5_PROTOCOL_H */
