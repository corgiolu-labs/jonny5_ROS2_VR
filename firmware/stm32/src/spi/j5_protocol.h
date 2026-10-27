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
 * (Il protocollo v2 con CRC e il frame J5IK per ROS 2 sono stati tolti il
 * 26/10/2026, branch fw/minimal: sono nella storia git.)
 */

#ifndef J5_PROTOCOL_H
#define J5_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 * Costanti
 * ========================================================= */

#define J5_PROTOCOL_FRAME_SIZE 64   /**< Dimensione frame in byte (fissa) */

#define J5_PROTOCOL_VERSION_V1 1U

/* =========================================================
 * Tipi
 * ========================================================= */

/** Tipo frame SPI. I valori numerici fanno parte del protocollo su filo. */
typedef enum {
    J5_FRAME_TYPE_TELEMETRY = 0x01, /**< Telemetria IMU stimata + stato servo software-side */
    J5_FRAME_TYPE_TEST_ECHO = 0x02, /**< Echo diagnostico (loopback)     */
    J5_FRAME_TYPE_STATUS    = 0x03, /**< Status / ack generico            */
    J5_FRAME_TYPE_J5VR      = 0x04  /**< Payload comandi VR dal master   */
} j5_frame_type_t;

/** Stato J5VR ricevuto: parsing payload → struttura C. Solo lettura dati, nessuna attuazione. */
struct j5vr_state {
    uint8_t  mode;
    int16_t  joy_x;
    int16_t  joy_y;
    int16_t  pitch;
    int16_t  yaw;
    uint8_t  intensity;
    uint16_t vr_heartbeat;
    uint8_t  priority;
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
     *   [38-39] target_id (BE u16, ignorato)
     *   [40-41] base   (BE s16, centi-gradi fisici)
     *   [42-43] spalla (BE s16, centi-gradi fisici)
     *   [44-45] gomito (BE s16, centi-gradi fisici)
     */
    uint8_t  mode5_arm_valid;
    uint8_t  mode5_control_flags;
    int16_t  mode5_arm_target_cdeg[3];
};

/** Ultimo frame J5VR ricevuto (aggiornato da j5vr_parse_payload). */
extern struct j5vr_state g_j5vr_latest;

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

/**
 * j5vr_latest_snapshot — copia coerente di g_j5vr_latest.
 * Il writer (thread SPI service, prio 7) puo' essere prelazionato dal RT loop
 * (prio 4) a meta' aggiornamento: i lettori multi-campo devono usare questa
 * funzione invece di copiare direttamente la struct.
 */
void j5vr_latest_snapshot(struct j5vr_state *out);


/**
 * j5vr_fill_tx_telemetry — scrive diagnostica nei byte 46-53 del payload TX.
 * Usato da hal_spi_slave per il frame STATUS.
 *
 * Layout offset 46-53:
 *   46-47: vr_heartbeat (BE)
 *   48:    mode
 *   49:    0x00 (riservato)
 *   50-51: diag_mask BE (bit0=deadman, 1=input_active, 2=armed, 3=freeze, 4=guard_seen,
 *          5=estop_line_active, 6=estop_bypass_active)
 *   52-53: 0x00 (riservato)
 */
void j5vr_fill_tx_telemetry(uint8_t *payload);

#endif /* J5_PROTOCOL_H */
