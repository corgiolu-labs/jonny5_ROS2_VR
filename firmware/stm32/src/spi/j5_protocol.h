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
    J5_FRAME_TYPE_ASSIST_V2_TELEMETRY = 0x07  /**< ASSIST v2 TELEMETRY echo          */
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
