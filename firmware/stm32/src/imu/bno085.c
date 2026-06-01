/*
 * JONNY5 - Minimal BNO085 driver (Phase 2)
 *
 * Scope:
 *   - init: drain post-reset queue, send Set-Feature for Rotation Vector.
 *   - poll: one I2C read per call, parse SHTP header, extract quaternion when
 *     a Rotation Vector input report is present on channel 3.
 *
 * Deliberately minimal: no DFU, no calibration save/load, no other features,
 * no interrupt handling. Single-threaded contract — caller must not invoke
 * bno085_* concurrently from two threads on the shared I2C bus.
 *
 * SHTP reference: CEVA SH-2 Reference Manual, section "SHTP over I2C".
 * BNO085 sensor reports: section "Sensor Reports (Input Reports)" → Rotation
 * Vector = sensor ID 0x05.
 */

#include "imu/bno085.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(bno085, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */
/* SHTP / SH-2 protocol constants                                             */
/* -------------------------------------------------------------------------- */

/* SHTP channels (see SH-2 Reference Manual §5.2) */
#define SHTP_CH_COMMAND       0   /* SHTP command / advertisement */
#define SHTP_CH_EXECUTABLE    1   /* reset / on / sleep */
#define SHTP_CH_CONTROL       2   /* sensor hub control: Set-Feature, etc. */
#define SHTP_CH_REPORTS       3   /* input sensor reports */
/* Other channels exist but are not used here. */

/* SH-2 command IDs on channel 2 */
#define SH2_CMD_SET_FEATURE   0xFD
#define SH2_CMD_GET_FEATURE_RESP 0xFC

/* Sensor IDs (subset) */
#define SH2_SID_ROTATION_VECTOR 0x05

/* Offsets inside an SHTP packet */
#define SHTP_OFF_LEN_LSB    0
#define SHTP_OFF_LEN_MSB    1
#define SHTP_OFF_CHANNEL    2
#define SHTP_OFF_SEQ        3
#define SHTP_HEADER_SIZE    4

/* Offsets inside a channel-3 Input Sensor Report prefix (5-byte timestamp base)
 * followed by the sensor-specific report. See SH-2 §7.2. */
#define INPUT_REPORT_TIMEBASE_SIZE 5

/* Rotation Vector report (inside the sensor-specific section): 14 bytes */
#define RV_OFF_ID           0   /* must be 0x05 */
#define RV_OFF_SEQ          1
#define RV_OFF_STATUS       2
#define RV_OFF_DELAY        3
#define RV_OFF_I_LSB        4
#define RV_OFF_I_MSB        5
#define RV_OFF_J_LSB        6
#define RV_OFF_J_MSB        7
#define RV_OFF_K_LSB        8
#define RV_OFF_K_MSB        9
#define RV_OFF_REAL_LSB     10
#define RV_OFF_REAL_MSB     11
#define RV_OFF_ACC_LSB      12
#define RV_OFF_ACC_MSB      13
#define RV_REPORT_SIZE      14

/* Fixed-point scale for rotation-vector quaternion components: Q14. */
#define Q14_SCALE           (1.0f / 16384.0f)

/* Read size: one 32-byte transaction comfortably fits a Rotation Vector report
 * (4 header + 5 timebase + 14 report = 23 bytes). Larger queued packets are
 * truncated: we read the 4-byte header, compute total length, and if bigger
 * than 32 B we drain the remainder in follow-up 32-byte reads. */
#define BNO085_READ_CHUNK   32

/* -------------------------------------------------------------------------- */
/* Module state                                                               */
/* -------------------------------------------------------------------------- */

static const struct device *s_bus = NULL;
static bool s_ready = false;
static uint8_t s_seq_ch2 = 0;   /* our outgoing sequence on channel 2 */

/* -------------------------------------------------------------------------- */
/* Low-level SHTP helpers                                                     */
/* -------------------------------------------------------------------------- */

/* Read exactly `len` bytes from the BNO085 into `buf`.
 * Returns 0 on success, negative errno otherwise. */
static int bno_read(uint8_t *buf, size_t len)
{
	if (s_bus == NULL) {
		return -ENODEV;
	}
	return i2c_read(s_bus, buf, len, BNO085_I2C_ADDR);
}

/* Write exactly `len` bytes to the BNO085 from `buf`.
 * Returns 0 on success, negative errno otherwise. */
static int bno_write(const uint8_t *buf, size_t len)
{
	if (s_bus == NULL) {
		return -ENODEV;
	}
	return i2c_write(s_bus, buf, len, BNO085_I2C_ADDR);
}

/* Parse the 16-bit SHTP length field. Bit 15 is the "continuation" flag and is
 * masked out. Length includes the 4-byte header. 0 means no packet available;
 * 0xFFFF is returned by some modes to indicate an error. */
static uint16_t shtp_parse_len(const uint8_t *hdr)
{
	return (uint16_t)hdr[SHTP_OFF_LEN_LSB] |
	       ((uint16_t)(hdr[SHTP_OFF_LEN_MSB] & 0x7F) << 8);
}

/* Drain up to `max_packets` pending packets. Used at init time to clear the
 * advertisement + reset-complete traffic the BNO085 emits after power-up. */
static int bno_drain(int max_packets)
{
	uint8_t buf[BNO085_READ_CHUNK];
	for (int i = 0; i < max_packets; i++) {
		int r = bno_read(buf, BNO085_READ_CHUNK);
		if (r != 0) {
			return r;
		}
		uint16_t len = shtp_parse_len(buf);
		if (len == 0 || len == 0xFFFF) {
			/* Queue is empty — we're done. */
			return 0;
		}
		/* If the packet is longer than our chunk, read and discard the
		 * rest so the next read sees the following packet. */
		if (len > BNO085_READ_CHUNK) {
			size_t remaining = len - BNO085_READ_CHUNK;
			while (remaining > 0) {
				size_t step = remaining > BNO085_READ_CHUNK ?
					      BNO085_READ_CHUNK : remaining;
				r = bno_read(buf, step);
				if (r != 0) {
					return r;
				}
				remaining -= step;
			}
		}
	}
	return 0;
}

/* -------------------------------------------------------------------------- */
/* Set-Feature command (channel 2, cmd 0xFD)                                   */
/* -------------------------------------------------------------------------- */

/* Build and send a Set-Feature command for the given sensor ID at the given
 * report period (microseconds). Other fields are set to 0 (no batching, no
 * sensor-specific config). Packet size = 4 header + 17 payload = 21 bytes.
 */
static int bno_set_feature(uint8_t sensor_id, uint32_t period_us)
{
	uint8_t pkt[21];
	const uint16_t len = sizeof(pkt);

	pkt[SHTP_OFF_LEN_LSB]   = (uint8_t)(len & 0xFF);
	pkt[SHTP_OFF_LEN_MSB]   = (uint8_t)((len >> 8) & 0x7F);
	pkt[SHTP_OFF_CHANNEL]   = SHTP_CH_CONTROL;
	pkt[SHTP_OFF_SEQ]       = s_seq_ch2++;

	pkt[4]  = SH2_CMD_SET_FEATURE;
	pkt[5]  = sensor_id;
	pkt[6]  = 0;    /* feature flags */
	pkt[7]  = 0;    /* change sensitivity LSB */
	pkt[8]  = 0;    /* change sensitivity MSB */
	/* Report interval (µs), little-endian */
	pkt[9]  = (uint8_t)(period_us & 0xFF);
	pkt[10] = (uint8_t)((period_us >> 8) & 0xFF);
	pkt[11] = (uint8_t)((period_us >> 16) & 0xFF);
	pkt[12] = (uint8_t)((period_us >> 24) & 0xFF);
	/* Batch interval (µs): 0 = no batching */
	pkt[13] = 0; pkt[14] = 0; pkt[15] = 0; pkt[16] = 0;
	/* Sensor-specific config: 0 */
	pkt[17] = 0; pkt[18] = 0; pkt[19] = 0; pkt[20] = 0;

	return bno_write(pkt, sizeof(pkt));
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

int bno085_init(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(i2c1))
	s_bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));
#else
	s_bus = NULL;
#endif
	s_ready = false;
	s_seq_ch2 = 0;

	if (s_bus == NULL || !device_is_ready(s_bus)) {
		LOG_ERR("I2C1 not ready");
		return -ENODEV;
	}

	/* Probe: a 1-byte read must ACK before we talk SHTP. */
	{
		uint8_t probe = 0;
		int r = bno_read(&probe, 1);
		if (r != 0) {
			LOG_ERR("probe read failed at 0x%02X (r=%d)",
				BNO085_I2C_ADDR, r);
			return r;
		}
	}

	/* Let the BNO085 finish any pending boot-time housekeeping. */
	k_msleep(100);

	/* Drain any queued packets (advertisement, reset-complete, etc.). */
	int rc = bno_drain(8);
	if (rc != 0) {
		LOG_WRN("drain failed rc=%d — continuing anyway", rc);
	}

	/* Enable Rotation Vector at 100 Hz (10000 µs period). */
	rc = bno_set_feature(SH2_SID_ROTATION_VECTOR, 10000u);
	if (rc != 0) {
		LOG_ERR("Set-Feature(RotationVector) failed rc=%d", rc);
		return rc;
	}

	/* Small settle delay so the first report is ready the next poll. */
	k_msleep(20);

	LOG_INF("init ok: addr=0x%02X, Rotation Vector @ 100 Hz",
		BNO085_I2C_ADDR);
	return 0;
}

int bno085_poll_quat(float *q_w, float *q_x, float *q_y, float *q_z,
		     uint8_t *accuracy)
{
	if (s_bus == NULL) {
		return -ENODEV;
	}

	uint8_t buf[BNO085_READ_CHUNK];
	int r = bno_read(buf, BNO085_READ_CHUNK);
	if (r != 0) {
		return r;
	}

	uint16_t len = shtp_parse_len(buf);
	if (len == 0 || len == 0xFFFF) {
		/* No data available. */
		return 0;
	}

	/* Only channel-3 (input reports) interests us for now. */
	if (buf[SHTP_OFF_CHANNEL] != SHTP_CH_REPORTS) {
		/* Drain any tail that didn't fit in the first chunk so the
		 * next poll sees the next packet cleanly. */
		if (len > BNO085_READ_CHUNK) {
			size_t remaining = len - BNO085_READ_CHUNK;
			uint8_t dump[BNO085_READ_CHUNK];
			while (remaining > 0) {
				size_t step = remaining > BNO085_READ_CHUNK ?
					      BNO085_READ_CHUNK : remaining;
				int rr = bno_read(dump, step);
				if (rr != 0) {
					return rr;
				}
				remaining -= step;
			}
		}
		return 0;
	}

	/* Layout: [SHTP hdr: 4][timebase: 5][report…].
	 * For Rotation Vector the report starts with id=0x05 and is 14 bytes.
	 */
	const size_t report_off = SHTP_HEADER_SIZE + INPUT_REPORT_TIMEBASE_SIZE;
	if (len < report_off + RV_REPORT_SIZE) {
		/* Shorter than expected — not a rotation vector. */
		return 0;
	}
	if (buf[report_off + RV_OFF_ID] != SH2_SID_ROTATION_VECTOR) {
		/* Some other sensor report we didn't enable — ignore. */
		return 0;
	}

	const uint8_t *p = &buf[report_off];

	/* Signed 16-bit little-endian assembly. */
	const int16_t i_raw  = (int16_t)((uint16_t)p[RV_OFF_I_LSB]    | ((uint16_t)p[RV_OFF_I_MSB]    << 8));
	const int16_t j_raw  = (int16_t)((uint16_t)p[RV_OFF_J_LSB]    | ((uint16_t)p[RV_OFF_J_MSB]    << 8));
	const int16_t k_raw  = (int16_t)((uint16_t)p[RV_OFF_K_LSB]    | ((uint16_t)p[RV_OFF_K_MSB]    << 8));
	const int16_t w_raw  = (int16_t)((uint16_t)p[RV_OFF_REAL_LSB] | ((uint16_t)p[RV_OFF_REAL_MSB] << 8));

	if (q_x) { *q_x = (float)i_raw * Q14_SCALE; }
	if (q_y) { *q_y = (float)j_raw * Q14_SCALE; }
	if (q_z) { *q_z = (float)k_raw * Q14_SCALE; }
	if (q_w) { *q_w = (float)w_raw * Q14_SCALE; }

	if (accuracy) {
		*accuracy = p[RV_OFF_STATUS] & 0x03;   /* bits 0..1 */
	}

	s_ready = true;
	return 1;
}

bool bno085_is_ready(void)
{
	return s_ready;
}
