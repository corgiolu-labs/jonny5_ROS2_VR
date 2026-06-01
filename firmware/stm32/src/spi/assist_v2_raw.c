/*
 * assist_v2_raw.c — ASSIST v2 RAW: validate CONTROL, echo TELEMETRY. Zero actuation.
 *
 * WIRE REGISTER v1 / v1.1 — assist region little-endian, CRC32 IEEE on bytes 0..115.
 * v1.1: extension block 48–83 (flags + quat); byte 84–115 non validati (tail).
 */

#include "spi/assist_v2_raw.h"
#include "spi/j5_protocol.h"
#include "imu/imu.h"
#include "servo/servo_control.h"
#include "core/state_machine.h"

#include <zephyr/kernel.h>
#include <string.h>
#include <math.h>

#if IS_ENABLED(CONFIG_ASSIST_V2_RAW_MODE)

/* ---- Shadow state (NOT read by servo path) ---- */
static uint32_t s_last_accepted_target_seq;
static uint64_t s_last_accepted_target_timestamp_ms;
static uint8_t  s_last_rx_classification = ASSIST_V2_RX_CLASS_NONE;
static uint16_t s_last_fault_code;
static uint32_t s_last_flags_echo;
static uint8_t  s_last_assist_state_echo;
static float    s_last_q_des_valid[6];
static int64_t  s_last_valid_rx_local_ms;
static uint8_t  s_last_assist_proto_echo = ASSIST_V2_PROTO_VERSION;

static uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
	uint32_t crc = 0xFFFFFFFFu;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint32_t)data[i];
		for (int b = 0; b < 8; b++) {
			if (crc & 1u) {
				crc = (crc >> 1) ^ 0xEDB88320u;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc ^ 0xFFFFFFFFu;
}

static inline uint32_t rd_u32_le(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

static inline uint64_t rd_u64_le(const uint8_t *p)
{
	uint64_t lo = rd_u32_le(p);
	uint64_t hi = rd_u32_le(p + 4);

	return lo | (hi << 32);
}

static inline void wr_u32_le(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
	p[2] = (uint8_t)((v >> 16) & 0xFF);
	p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void wr_u64_le(uint8_t *p, uint64_t v)
{
	wr_u32_le(p, (uint32_t)(v & 0xFFFFFFFFu));
	wr_u32_le(p + 4, (uint32_t)(v >> 32));
}

static inline void wr_u16_le(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void wr_i16_le(uint8_t *p, int16_t v)
{
	wr_u16_le(p, (uint16_t)v);
}

static inline float rd_f32_le(const uint8_t *p)
{
	union {
		uint32_t u;
		float f;
	} u;

	u.u = rd_u32_le(p);
	return u.f;
}

static inline void wr_f32_le(uint8_t *p, float f)
{
	union {
		uint32_t u;
		float fl;
	} u;

	u.fl = f;
	wr_u32_le(p, u.u);
}

static int16_t deg_to_cdeg_i16(float deg)
{
	float v = deg;
	if (isnan(v) || isinf(v)) {
		v = 0.0f;
	}
	if (v > 327.67f) {
		v = 327.67f;
	}
	if (v < -327.68f) {
		v = -327.68f;
	}
	float scaled = v * 100.0f;
	if (scaled >= 0.0f) {
		scaled += 0.5f;
	} else {
		scaled -= 0.5f;
	}
	return (int16_t)scaled;
}

static bool finite_float(float x)
{
	return !isnan(x) && !isinf(x);
}

static bool q_in_range(float q)
{
	return finite_float(q) && q >= 0.0f && q <= 180.0f;
}

static bool bytes_all_zero(const uint8_t *p, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		if (p[i] != 0U) {
			return false;
		}
	}
	return true;
}

/*
 * WIRE v1.1 extension: flags + optional controller quats. Quat bytes never feed actuation.
 * reserved_tail 84–115 intentionally not checked.
 */
static bool validate_assist_v11_extension(const uint8_t *a, uint16_t *fault_out)
{
	uint32_t ef = rd_u32_le(a + 48);

	if ((ef & ~0x3u) != 0U) {
		*fault_out = ASSIST_V2_FAULT_EXT_FLAGS_INVALID;
		return false;
	}

	unsigned int has_l = (ef & ASSIST_V2_EXT_FLAG_HAS_LEFT_QUAT) != 0U;
	unsigned int has_r = (ef & ASSIST_V2_EXT_FLAG_HAS_RIGHT_QUAT) != 0U;

	if (has_l) {
		for (int k = 0; k < 4; k++) {
			float q = rd_f32_le(a + 52 + k * 4);

			if (!finite_float(q)) {
				*fault_out = ASSIST_V2_FAULT_CONTROLLER_QUAT_INVALID;
				return false;
			}
		}
	} else if (!bytes_all_zero(a + 52, 16U)) {
		*fault_out = ASSIST_V2_FAULT_CONTROLLER_QUAT_INVALID;
		return false;
	}

	if (has_r) {
		for (int k = 0; k < 4; k++) {
			float q = rd_f32_le(a + 68 + k * 4);

			if (!finite_float(q)) {
				*fault_out = ASSIST_V2_FAULT_CONTROLLER_QUAT_INVALID;
				return false;
			}
		}
	} else if (!bytes_all_zero(a + 68, 16U)) {
		*fault_out = ASSIST_V2_FAULT_CONTROLLER_QUAT_INVALID;
		return false;
	}

	return true;
}

static void fill_servo_q_act(float out[6])
{
	out[0] = (float)servo_get_angle(SERVO_BASE);
	out[1] = (float)servo_get_angle(SERVO_SPALLA);
	out[2] = (float)servo_get_angle(SERVO_GOMITO);
	out[3] = (float)servo_get_angle(SERVO_YAW);
	out[4] = (float)servo_get_angle(SERVO_PITCH);
	out[5] = (float)servo_get_angle(SERVO_ROLL);
}

static void deadman_and_movement(uint8_t *deadman_out, uint8_t *move_out)
{
	const bool grip_l = (g_j5vr_latest.buttons_left & (1U << 1)) != 0U;
	const bool grip_r = (g_j5vr_latest.buttons_right & (1U << 1)) != 0U;
	const bool deadman = grip_l && grip_r;

	*deadman_out = deadman ? 1U : 0U;
	*move_out = (state_machine_is_movement_allowed() && deadman) ? 1U : 0U;
}

static void build_telemetry_payload(uint8_t *ar, const uint8_t *rx128)
{
	(void)rx128;
	(void)memset(ar, 0, J5_ASSIST_V2_ASSIST_REGION_LEN);
	ar[ASSIST_V3_TLM_OFF_PROTO_VERSION] = ASSIST_V2_PROTO_VERSION_V3;
	ar[ASSIST_V3_TLM_OFF_LAYOUT_MINOR] = 0x00U;

	float q_act[6];
	fill_servo_q_act(q_act);
	for (int i = 0; i < 6; i++) {
		wr_i16_le(ar + ASSIST_V3_TLM_OFF_Q_ACT_CDEG + (i * 2), deg_to_cdeg_i16(q_act[i]));
		wr_i16_le(
			ar + ASSIST_V3_TLM_OFF_Q_DES_APPLIED_CDEG + (i * 2),
			deg_to_cdeg_i16(s_last_q_des_valid[i])
		);
	}

	wr_u32_le(ar + ASSIST_V3_TLM_OFF_TARGET_SEQ_ECHO, s_last_accepted_target_seq);
	wr_u64_le(ar + ASSIST_V3_TLM_OFF_TARGET_TS_ECHO_MS, s_last_accepted_target_timestamp_ms);

	uint32_t age = 0U;
	if (s_last_accepted_target_seq > 0U) {
		int64_t now = k_uptime_get();
		int64_t dt = now - s_last_valid_rx_local_ms;
		if (dt > 0 && dt < (int64_t)0xFFFFFFFFLL) {
			age = (uint32_t)dt;
		}
	}
	wr_u32_le(ar + ASSIST_V3_TLM_OFF_CONTROL_AGE_MS, age);

	ar[ASSIST_V3_TLM_OFF_ASSIST_STATE] = s_last_assist_state_echo;
	uint8_t dm = 0, mv = 0;
	deadman_and_movement(&dm, &mv);
	ar[ASSIST_V3_TLM_OFF_DEADMAN] = dm;
	ar[ASSIST_V3_TLM_OFF_MOVEMENT_ALLOWED] = mv;

	imu_snapshot_t snap;
	bool snap_ok = imu_get_snapshot(&snap);
	bool imu_valid = false;
	uint8_t tlm_flags = 0U;
	if (snap_ok) {
		imu_valid = imu_is_orientation_valid();
		tlm_flags |= (1U << 0); /* IMU sample present */
		if (imu_valid) {
			tlm_flags |= (1U << 1); /* IMU orientation valid */
		}
	}
	ar[ASSIST_V3_TLM_OFF_IMU_VALID] = imu_valid ? 1U : 0U;

	wr_u32_le(ar + ASSIST_V3_TLM_OFF_FLAGS_ECHO, s_last_flags_echo);
	wr_u16_le(ar + ASSIST_V3_TLM_OFF_FAULT_CODE, s_last_fault_code);
	ar[ASSIST_V3_TLM_OFF_RX_CLASS] = s_last_rx_classification;
	ar[ASSIST_V3_TLM_OFF_TLM_FLAGS] = tlm_flags;

	if (snap_ok) {
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_ACCEL_X, snap.accel_x);
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_ACCEL_Y, snap.accel_y);
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_ACCEL_Z, snap.accel_z);
		/* Gyro standard 0x07 v3: rad/s (coerente con imu_snapshot_t). */
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_GYRO_X, snap.gyro_x);
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_GYRO_Y, snap.gyro_y);
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_GYRO_Z, snap.gyro_z);
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_TEMP_C, snap.temp);
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_Q_W, snap.quat_w);
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_Q_X, snap.quat_x);
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_Q_Y, snap.quat_y);
		wr_f32_le(ar + ASSIST_V3_TLM_OFF_IMU_Q_Z, snap.quat_z);
		wr_u32_le(ar + ASSIST_V3_TLM_OFF_IMU_SAMPLE_COUNTER, snap.sample_counter);
	}
	/* a[102..115] reserved per futura IMU 9DOF (magnetometro/heading/quality). */

	wr_u32_le(ar + ASSIST_V3_TLM_OFF_CRC32, crc32_ieee(ar, 116));
}

static void write_header_tx(uint8_t *tx128, const uint8_t *rx128, uint8_t frame_type)
{
	tx128[0] = 'J';
	tx128[1] = '5';
	tx128[2] = 0x01;
	tx128[3] = frame_type;
	tx128[4] = rx128[4];
	tx128[5] = rx128[5];
	tx128[6] = J5_ASSIST_V2_FRAME_SIZE;
	tx128[7] = 0x00;
}

void assist_v2_raw_handle_control_and_build_telemetry(const uint8_t *rx128, uint8_t *out_tx128)
{
	const uint8_t *a = rx128 + 8;
	uint32_t crc_expect = rd_u32_le(a + 116);
	uint32_t crc_calc = crc32_ieee(a, 116);
	uint8_t cls = ASSIST_V2_RX_CLASS_REJECTED;
	uint16_t fault = ASSIST_V2_FAULT_NONE;

	if (crc_calc != crc_expect) {
		fault = ASSIST_V2_FAULT_CRC;
		goto done;
	}

	const uint8_t proto = a[0];

	if (proto != ASSIST_V2_PROTO_VERSION && proto != ASSIST_V2_PROTO_VERSION_V11) {
		fault = ASSIST_V2_FAULT_PROTO_MISMATCH;
		goto done;
	}
	if (a[1] != ASSIST_V2_MODE_ASSIST_V2) {
		fault = ASSIST_V2_FAULT_MODE_MISMATCH;
		goto done;
	}

	uint32_t target_seq = rd_u32_le(a + 2);
	uint64_t target_ts = rd_u64_le(a + 6);
	uint32_t validity = rd_u32_le(a + 14);

	if (target_seq == 0U) {
		fault = ASSIST_V2_FAULT_FIELD_RANGE;
		goto done;
	}
	if (target_ts == 0ULL) {
		fault = ASSIST_V2_FAULT_FIELD_RANGE;
		goto done;
	}
	if (validity == 0U) {
		fault = ASSIST_V2_FAULT_FIELD_RANGE;
		goto done;
	}

	if (proto == ASSIST_V2_PROTO_VERSION) {
		for (int i = 48; i <= 115; i++) {
			if (a[i] != 0U) {
				fault = ASSIST_V2_FAULT_RESERVED_FLAGS;
				goto done;
			}
		}
	} else {
		if (!validate_assist_v11_extension(a, &fault)) {
			goto done;
		}
	}

	uint8_t assist_st = a[42];

	if (a[43] != 0U) {
		fault = ASSIST_V2_FAULT_RESERVED_FLAGS;
		goto done;
	}

	uint32_t flags = rd_u32_le(a + 44);

	if ((flags & 0xFFFFFFC0u) != 0U) {
		fault = ASSIST_V2_FAULT_RESERVED_FLAGS;
		goto done;
	}

	if (assist_st > 6U) {
		fault = ASSIST_V2_FAULT_FIELD_RANGE;
		goto done;
	}

	for (int j = 0; j < 6; j++) {
		float q = rd_f32_le(a + 18 + j * 4);

		if (!q_in_range(q)) {
			fault = ASSIST_V2_FAULT_NAN_INF;
			goto done;
		}
	}

	if (s_last_accepted_target_seq > 0U && target_seq <= s_last_accepted_target_seq) {
		cls = ASSIST_V2_RX_CLASS_IGNORED;
		fault = ASSIST_V2_FAULT_NONE;
		goto done;
	}

	cls = ASSIST_V2_RX_CLASS_VALID;
	s_last_accepted_target_seq = target_seq;
	s_last_accepted_target_timestamp_ms = target_ts;
	s_last_valid_rx_local_ms = k_uptime_get();
	s_last_flags_echo = flags;
	s_last_assist_state_echo = assist_st;
	s_last_assist_proto_echo = proto;
	for (int j = 0; j < 6; j++) {
		s_last_q_des_valid[j] = rd_f32_le(a + 18 + j * 4);
	}
	/* Mirror X/Y dal canale ASSIST v2 flags (bit4/bit5) verso stato J5VR
	 * usato da MANUAL ROLL. Manteniamo invariati gli altri bit di buttons_left. */
	{
		uint16_t bl = g_j5vr_latest.buttons_left;
		bl = (uint16_t)(bl & ~(uint16_t)((1U << 4) | (1U << 5)));
		if ((flags & (1U << 4)) != 0U) { bl = (uint16_t)(bl | (1U << 4)); }
		if ((flags & (1U << 5)) != 0U) { bl = (uint16_t)(bl | (1U << 5)); }
		g_j5vr_latest.buttons_left = bl;
	}
	fault = ASSIST_V2_FAULT_NONE;

done:
	s_last_rx_classification = cls;
	s_last_fault_code = fault;

	write_header_tx(out_tx128, rx128, (uint8_t)J5_FRAME_TYPE_ASSIST_V2_TELEMETRY);
	build_telemetry_payload(out_tx128 + 8, rx128);
}

void assist_v2_raw_build_telemetry_only(const uint8_t *rx128, uint8_t *out_tx128)
{
	write_header_tx(out_tx128, rx128, (uint8_t)J5_FRAME_TYPE_ASSIST_V2_TELEMETRY);
	build_telemetry_payload(out_tx128 + 8, rx128);
}

#else /* !CONFIG_ASSIST_V2_RAW_MODE */

void assist_v2_raw_handle_control_and_build_telemetry(const uint8_t *rx128, uint8_t *out_tx128)
{
	(void)rx128;
	(void)out_tx128;
}

void assist_v2_raw_build_telemetry_only(const uint8_t *rx128, uint8_t *out_tx128)
{
	(void)rx128;
	(void)out_tx128;
}

#endif /* CONFIG_ASSIST_V2_RAW_MODE */
