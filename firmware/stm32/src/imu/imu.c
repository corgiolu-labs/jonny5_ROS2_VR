/*
 * JONNY5 - IMU Module (BNO085)
 * Orientamento via BNO085 SHTP Rotation Vector (fusione sul sensore).
 * Backend unico: il quaternione arriva gia' fuso da bno085.c via I2C1.
 *
 * NOTE:
 *   - Critical path (imu_update_orientation, imu_get_snapshot,
 *     imu_get_quaternion) NON va modificato nei refactor.
 *   - imu_i2c_scan_bus rileva il BNO085 (0x4A/0x4B) e abilita il backend:
 *     non e' diagnostica opzionale, fa parte dell'init.
 */

#include "imu.h"
#include "imu/bno085.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_DBG);

/* Serve per poter richiamare PINCTRL_DT_DEV_CONFIG_GET(i2c1) in questo TU */
#if DT_NODE_EXISTS(DT_NODELABEL(i2c1))
PINCTRL_DT_DEFINE(DT_NODELABEL(i2c1));
#endif

/* Stato quaternione IMU (BNO085) — inizialmente identità */
static struct imu_quat g_imu_quat = {
	.w = 1.0f,
	.x = 0.0f,
	.y = 0.0f,
	.z = 0.0f,
};

/* True quando l'orientamento (quaternione BNO085) e' valido (dati freschi). */
static bool g_imu_orientation_valid = false;

/*
 * Double-buffer atomico lock-free per snapshot IMU.
 *
 * Schema writer/reader senza mutex, senza seqlock, senza retry:
 *   Writer (thread IMU, 400 Hz):
 *     1. Legge l'indice attivo corrente (0 o 1).
 *     2. Scrive il nuovo snapshot nel buffer inattivo (indice ^ 1).
 *     3. Esegue atomic_set() per pubblicare il nuovo indice attivo.
 *        Da questo punto tutti i nuovi reader vedono il buffer aggiornato.
 *   Reader (thread SPI, 60–100 Hz):
 *     1. atomic_get() dell'indice attivo — una sola lettura atomica.
 *     2. Copia strutturale del buffer indicato — nessun loop, nessuna
 *        dipendenza dal writer.
 *
 * Sostituisce il vecchio seqlock (g_imu_seq pari/dispari + __DMB()),
 * introdotto per eliminare imu_valid=False sporadici a SPI > 60 Hz
 * causati dalla contesa writer/reader sul seqlock dispari.
 * Con il double-buffer imu_valid=100% è garantito fino a 100 Hz SPI.
 */
static imu_snapshot_t g_imu_buffers[2];
static atomic_t       g_imu_active_idx       = ATOMIC_INIT(0);
static bool           g_imu_has_valid_snapshot = false;
static uint32_t       g_imu_sample_counter = 0;

static bool imu_available = false;

void imu_i2c_bus_recovery(void)
{
	/* AN4559/ST: I2C bus recovery tramite 9 clock manuali su SCL */
	LOG_WRN("[IMU] I2C bus recovery start");

	/* Nucleo-F446RE: I2C1 su PB8(SCL) / PB9(SDA) come da overlay */
#if DT_NODE_EXISTS(DT_NODELABEL(gpiob)) && DT_NODE_EXISTS(DT_NODELABEL(i2c1))
	const struct device *gpio_b = DEVICE_DT_GET(DT_NODELABEL(gpiob));
	if (!device_is_ready(gpio_b))
	{
		LOG_ERR("[IMU] GPIOB not ready (cannot recover)");
		return;
	}

	/* Configura temporaneamente PB8/PB9 come GPIO open-drain */
	(void)gpio_pin_configure(gpio_b, 8, GPIO_OUTPUT | GPIO_OPEN_DRAIN);
	(void)gpio_pin_configure(gpio_b, 9, GPIO_OUTPUT | GPIO_OPEN_DRAIN);
	/* Rilascia SDA (open-drain high = high-Z) */
	gpio_pin_set(gpio_b, 9, 1);

	/* Clock pulses — k_usleep invece di k_busy_wait: cede la CPU agli altri thread */
	for (int i = 0; i < 9; i++)
	{
		gpio_pin_set(gpio_b, 8, 1);
		k_usleep(10);
		gpio_pin_set(gpio_b, 8, 0);
		k_usleep(10);
	}

	/* Termina con SCL alto */
	gpio_pin_set(gpio_b, 8, 1);
	k_usleep(10);

	/* Verifica SDA dopo i 9 clock */
	int sda = gpio_pin_get(gpio_b, 9);
	if (sda == 0)
	{
		LOG_ERR("[IMU] I2C bus still locked after recovery");
	}

	/* Sequenza START+STOP manuale per resettare lo state-machine del
	 * device I2C (NXP/ST AN4509). I soli 9 clock non bastano se lo slave
	 * sta ancora "aspettando il prossimo byte": serve una transizione
	 * START (SDA 1->0 con SCL=1) seguita da STOP (SDA 0->1 con SCL=1) per
	 * riportare entrambi i lati in idle. */
	gpio_pin_set(gpio_b, 9, 1);
	gpio_pin_set(gpio_b, 8, 1);
	k_usleep(10);
	gpio_pin_set(gpio_b, 9, 0);      /* START: SDA 1->0 mentre SCL=1 */
	k_usleep(10);
	gpio_pin_set(gpio_b, 8, 0);      /* abbassa SCL dopo START */
	k_usleep(10);
	gpio_pin_set(gpio_b, 8, 1);      /* risale SCL con SDA basso */
	k_usleep(10);
	gpio_pin_set(gpio_b, 9, 1);      /* STOP: SDA 0->1 mentre SCL=1 */
	k_usleep(10);

	/* Ripristina pinctrl I2C1 default */
	(void)pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(i2c1)), PINCTRL_STATE_DEFAULT);
#else
	LOG_WRN("[IMU] I2C bus recovery skipped (missing DT nodes)");
#endif

	LOG_WRN("[IMU] I2C bus recovery done");
}

/* Set to true by the bus scan when a BNO085 (0x4A or 0x4B) is found on I2C1.
 * Consumed by imu_init() to select the BNO085 backend (only supported sensor). */
static bool s_bno085_detected = false;

/* Scan the I2C bus and log every address that ACKs (also sets s_bno085_detected).
 * Useful when the sensor on the bus is unknown or has been swapped (e.g. BNO085
 * at 0x4A/0x4B in place of MPU6050 at 0x68). Uses a 1-byte raw i2c_read which
 * is protocol-agnostic: any device present must ACK its address.
 */
static void imu_i2c_scan_bus(const struct device *bus, const char *name)
{
	if (bus == NULL || !device_is_ready(bus)) {
		LOG_WRN("[IMU] %s SCAN skipped: bus not ready", name);
		return;
	}
	char line[160];
	int n = snprintf(line, sizeof(line), "[IMU] %s SCAN 0x08-0x77:", name);
	int hits = 0;
	for (uint16_t addr = 0x08; addr <= 0x77; addr++) {
		uint8_t b = 0;
		int r = i2c_read(bus, &b, 1, addr);
		if (r == 0) {
			if (n < (int)sizeof(line) - 8) {
				n += snprintf(line + n, sizeof(line) - n,
					      " 0x%02X", (unsigned)addr);
			}
			hits++;
			if (addr == 0x4A || addr == 0x4B) {
				s_bno085_detected = true;
			}
		}
	}
	if (hits == 0) {
		LOG_WRN("[IMU] %s SCAN: no devices ACKed", name);
	} else {
		LOG_INF("%s  (%d device(s))", line, hits);
	}
}

int imu_init(void)
{
	/* Reset internal quat + sample counter regardless of which backend we
	 * end up using. Double-buffer remains stale until first publish. */
	g_imu_orientation_valid = false;
	g_imu_sample_counter = 0;
	g_imu_quat.w = 1.0f;
	g_imu_quat.x = 0.0f;
	g_imu_quat.y = 0.0f;
	g_imu_quat.z = 0.0f;

	/* Bus recovery: good hygiene for any I2C device after reset/flash. */
	imu_i2c_bus_recovery();

	/* Diagnostic bus scan. Sets s_bno085_detected when 0x4A/0x4B ACKs.
	 * Logs every address that responds — valuable when the sensor on the
	 * bus changes (MPU6050 at 0x68, BNO085 at 0x4A/0x4B, etc.). */
#if DT_NODE_EXISTS(DT_NODELABEL(i2c1))
	imu_i2c_scan_bus(DEVICE_DT_GET(DT_NODELABEL(i2c1)), "I2C1");
#endif

	/* ============================================================ *
	 * Production backend: BNO085 (fused rotation vector via SHTP).
	 * ============================================================ */
	if (s_bno085_detected) {
		int rc = bno085_init();
		if (rc != 0) {
			imu_available = false;
			LOG_ERR("[IMU] bno085_init failed rc=%d (imu_available=0)", rc);
			return rc;
		}
		imu_available = true;
		LOG_INF("[IMU] init ok (backend=BNO085 @ 0x%02X) — quat feed from Rotation Vector",
			BNO085_I2C_ADDR);
		return 0;
	}

	/* Nessun BNO085 sul bus: IMU non disponibile (backend unico BNO085). */
	LOG_ERR("[IMU] no BNO085 detected on I2C1 (expected 0x4A/0x4B) - imu_available=0");
	imu_available = false;
	return -ENODEV;
}

bool imu_is_available(void)
{
	return imu_available;
}

bool imu_is_orientation_valid(void)
{
	return g_imu_orientation_valid;
}

/**
 * Azzera lo stato di orientamento (quaternione + snapshot) quando IMU è disabilitata (IMUOFF).
 * Evita che la telemetria o altri consumer riutilizzino l'ultimo quat valido.
 */
void imu_clear_orientation_state(void)
{
	g_imu_orientation_valid = false;
	g_imu_quat.w = 1.0f;
	g_imu_quat.x = 0.0f;
	g_imu_quat.y = 0.0f;
	g_imu_quat.z = 0.0f;

	/* Scrivi snapshot di reset su entrambi i buffer per coerenza */
	imu_snapshot_t reset = {
		.quat_w = 1.0f, .quat_x = 0.0f, .quat_y = 0.0f, .quat_z = 0.0f,
		.zworld_x = 0.0f, .zworld_y = 0.0f, .zworld_z = 1.0f,
		.wrist_roll_rate = 0.0f, .wrist_pitch_rate = 0.0f, .wrist_yaw_rate = 0.0f,
	};
	g_imu_buffers[0] = reset;
	g_imu_buffers[1] = reset;
	atomic_set(&g_imu_active_idx, 0);
	g_imu_has_valid_snapshot = false;
	g_imu_sample_counter = 0;
}

/* ------------------------------------------------------------------------- *
 * imu_update_orientation — production body (BNO085 backend).
 *
 * Called at 400 Hz by the IMU thread. BNO085 emits Rotation Vector reports
 * natively at ~100 Hz, so roughly 3 of every 4 calls return no new data —
 * those early-exit without re-publishing, and the snapshot stays valid from
 * the previous publish (downstream continues reading the last fresh quat).
 *
 * imu_snapshot_t mapping in BNO085 v1:
 *   quat_*        = Rotation Vector (unit quaternion, from SH-2 sensor 0x05)
 *   zworld_*      = derived from quat (same formula as legacy)
 *   sample_counter/timestamp_us = advanced on each new packet
 *   accel_*, gyro_*, temp, wrist_*_rate = zeroed (see BNO085-v1 note below)
 *
 * BNO085-v1 temporary limitation: only Rotation Vector is enabled on the
 * sensor; raw accel/gyro/temp and wrist rates are not yet plumbed. Consumers
 * that only read quat_* + zworld_* + sample_counter + timestamp_us are
 * unaffected (this is the SPI telemetry, ws_handlers_imu, dashboard IMU view,
 * VR head-tracking path — verified from current repo grep). If a consumer
 * starts using accel/gyro, Phase 6 will enable Calibrated Accel (sensor 0x01)
 * + Calibrated Gyro (sensor 0x02) on the BNO085 and populate those fields.
 * ------------------------------------------------------------------------- */
void imu_update_orientation(float dt_s)
{
	ARG_UNUSED(dt_s); /* BNO085 is fused internally; Madgwick's dt no longer used */

	if (!imu_available) {
		g_imu_orientation_valid = false;
		return;
	}

	float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
	uint8_t accuracy = 0;
	const int r = bno085_poll_quat(&qw, &qx, &qy, &qz, &accuracy);
	if (r <= 0) {
		/* No new Rotation Vector report this tick (r==0 = empty queue;
		 * r<0 = transient I/O error). Keep the last published snapshot
		 * valid: returning without touching g_imu_orientation_valid
		 * preserves it if it was already true, and returning without
		 * re-publishing keeps the existing active buffer in place. */
		return;
	}

	/* Sync the legacy g_imu_quat (imu_get_quaternion reads it for consumers
	 * that don't use imu_get_snapshot). */
	g_imu_quat.w = qw;
	g_imu_quat.x = qx;
	g_imu_quat.y = qy;
	g_imu_quat.z = qz;
	g_imu_orientation_valid = true;

	/* Double-buffer writer: fill the inactive buffer, then atomic flip. */
	uint32_t cur = (uint32_t)atomic_get(&g_imu_active_idx);
	uint32_t next = cur ^ 1U;

	imu_snapshot_t *dst = &g_imu_buffers[next];
	const uint32_t sample_counter = ++g_imu_sample_counter;
	const uint32_t timestamp_us = (uint32_t)k_cyc_to_us_floor32(k_cycle_get_32());

	/* BNO085-v1: raw IMU fields not yet enabled on the sensor. */
	dst->accel_x = 0.0f;
	dst->accel_y = 0.0f;
	dst->accel_z = 0.0f;
	dst->gyro_x  = 0.0f;
	dst->gyro_y  = 0.0f;
	dst->gyro_z  = 0.0f;
	dst->temp    = 0.0f;

	dst->quat_w  = qw;
	dst->quat_x  = qx;
	dst->quat_y  = qy;
	dst->quat_z  = qz;
	dst->zworld_x = 2.0f * (qw * qy + qx * qz);
	dst->zworld_y = 2.0f * (qy * qz - qw * qx);
	dst->zworld_z = qw * qw - qx * qx - qy * qy + qz * qz;

	/* BNO085-v1: wrist rates derived from raw gyro — zeroed until Phase 6
	 * enables Calibrated Gyro on the sensor. */
	dst->wrist_roll_rate  = 0.0f;
	dst->wrist_pitch_rate = 0.0f;
	dst->wrist_yaw_rate   = 0.0f;

	dst->sample_counter = sample_counter;
	dst->timestamp_us = timestamp_us;

	atomic_set(&g_imu_active_idx, (atomic_val_t)next);
	g_imu_has_valid_snapshot = true;

	/* Observability: log BNO085 Rotation-Vector accuracy (0..3) on change so
	 * magnetometer-calibration state is visible on COM3. No SPI/WS effect. */
	{
		static uint8_t s_last_bno085_acc = 0xFF;
		if (accuracy != s_last_bno085_acc) {
			LOG_INF("[IMU] BNO085 accuracy=%u", (unsigned)accuracy);
			s_last_bno085_acc = accuracy;
		}
	}
}

void imu_get_quaternion(struct imu_quat *out)
{
	if (out == NULL)
	{
		return;
	}

	/* Double-buffer reader: atomic_get garantisce visibilità, nessun retry */
	uint32_t idx = (uint32_t)atomic_get(&g_imu_active_idx);
	out->w = g_imu_buffers[idx].quat_w;
	out->x = g_imu_buffers[idx].quat_x;
	out->y = g_imu_buffers[idx].quat_y;
	out->z = g_imu_buffers[idx].quat_z;
}

bool imu_get_snapshot(imu_snapshot_t *out)
{
	if (out == NULL || !g_imu_has_valid_snapshot)
	{
		return false;
	}

	uint32_t idx = (uint32_t)atomic_get(&g_imu_active_idx);
	*out = g_imu_buffers[idx];
	return true;
}

