/*
 * assist_v2_raw.h — ASSIST v2 RAW validation (WIRE REGISTER v1 / v1.1 / v3 telemetry).
 *
 * NO actuation: parse CONTROL 0x06, classify VALID/REJECTED/IGNORED, emit TELEMETRY 0x07.
 */

#ifndef ASSIST_V2_RAW_H
#define ASSIST_V2_RAW_H

#include <stdint.h>

#define J5_ASSIST_V2_FRAME_SIZE 128U
#define J5_ASSIST_V2_ASSIST_REGION_LEN 120U

#define ASSIST_V2_PROTO_VERSION     0x01U
#define ASSIST_V2_PROTO_VERSION_V11 0x02U
#define ASSIST_V2_PROTO_VERSION_V3  0x03U
#define ASSIST_V2_MODE_ASSIST_V2    0x01U

#define ASSIST_V2_EXT_FLAG_HAS_LEFT_QUAT  0x01U
#define ASSIST_V2_EXT_FLAG_HAS_RIGHT_QUAT 0x02U

#define ASSIST_V2_RX_CLASS_NONE     0U
#define ASSIST_V2_RX_CLASS_VALID    1U
#define ASSIST_V2_RX_CLASS_REJECTED 2U
#define ASSIST_V2_RX_CLASS_IGNORED  3U

#define ASSIST_V2_FAULT_NONE               0U
#define ASSIST_V2_FAULT_PROTO_MISMATCH     1U
#define ASSIST_V2_FAULT_CRC                2U
#define ASSIST_V2_FAULT_FIELD_RANGE        3U
#define ASSIST_V2_FAULT_RESERVED_FLAGS     4U
#define ASSIST_V2_FAULT_NAN_INF            6U
#define ASSIST_V2_FAULT_MODE_MISMATCH           7U
#define ASSIST_V2_FAULT_EXT_FLAGS_INVALID       8U
#define ASSIST_V2_FAULT_CONTROLLER_QUAT_INVALID 9U

/* TELEMETRY 0x07 v3 layout (assist region a[0..119], CRC su a[0..115]) */
#define ASSIST_V3_TLM_OFF_PROTO_VERSION      0U
#define ASSIST_V3_TLM_OFF_LAYOUT_MINOR       1U
#define ASSIST_V3_TLM_OFF_Q_ACT_CDEG         2U   /* 6 x i16 LE, B/S/G/Y/P/R */
#define ASSIST_V3_TLM_OFF_Q_DES_APPLIED_CDEG 14U  /* 6 x i16 LE, B/S/G/Y/P/R */
#define ASSIST_V3_TLM_OFF_TARGET_SEQ_ECHO    26U  /* u32 LE */
#define ASSIST_V3_TLM_OFF_TARGET_TS_ECHO_MS  30U  /* u64 LE */
#define ASSIST_V3_TLM_OFF_CONTROL_AGE_MS     38U  /* u32 LE */
#define ASSIST_V3_TLM_OFF_ASSIST_STATE       42U  /* u8 */
#define ASSIST_V3_TLM_OFF_DEADMAN            43U  /* u8 */
#define ASSIST_V3_TLM_OFF_MOVEMENT_ALLOWED   44U  /* u8 */
#define ASSIST_V3_TLM_OFF_IMU_VALID          45U  /* u8 */
#define ASSIST_V3_TLM_OFF_FLAGS_ECHO         46U  /* u32 LE */
#define ASSIST_V3_TLM_OFF_FAULT_CODE         50U  /* u16 LE */
#define ASSIST_V3_TLM_OFF_RX_CLASS           52U  /* u8 */
#define ASSIST_V3_TLM_OFF_TLM_FLAGS          53U  /* u8 */
#define ASSIST_V3_TLM_OFF_IMU_ACCEL_X        54U  /* f32 LE (g) */
#define ASSIST_V3_TLM_OFF_IMU_ACCEL_Y        58U
#define ASSIST_V3_TLM_OFF_IMU_ACCEL_Z        62U
#define ASSIST_V3_TLM_OFF_IMU_GYRO_X         66U  /* f32 LE (rad/s) */
#define ASSIST_V3_TLM_OFF_IMU_GYRO_Y         70U
#define ASSIST_V3_TLM_OFF_IMU_GYRO_Z         74U
#define ASSIST_V3_TLM_OFF_IMU_TEMP_C         78U  /* f32 LE */
#define ASSIST_V3_TLM_OFF_IMU_Q_W            82U  /* f32 LE */
#define ASSIST_V3_TLM_OFF_IMU_Q_X            86U
#define ASSIST_V3_TLM_OFF_IMU_Q_Y            90U
#define ASSIST_V3_TLM_OFF_IMU_Q_Z            94U
#define ASSIST_V3_TLM_OFF_IMU_SAMPLE_COUNTER 98U  /* u32 LE */
#define ASSIST_V3_TLM_OFF_RESERVED_START     102U /* 9DOF future: mag/heading/quality */
#define ASSIST_V3_TLM_OFF_RESERVED_END       115U
#define ASSIST_V3_TLM_OFF_CRC32              116U /* u32 LE */

void assist_v2_raw_handle_control_and_build_telemetry(const uint8_t *rx128,
						      uint8_t *out_tx128);
void assist_v2_raw_build_telemetry_only(const uint8_t *rx128, uint8_t *out_tx128);

#endif /* ASSIST_V2_RAW_H */
