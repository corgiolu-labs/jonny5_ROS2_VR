/*
 * JONNY5 - IMU Module (BNO085)
 * Orientamento BNO085: quaternione fuso (Rotation Vector) via I2C1
 */

#ifndef IMU_H
#define IMU_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdbool.h>
#include <stdint.h>

/* Quaternione IMU stimato (BNO085) */
struct imu_quat {
	float w;
	float x;
	float y;
	float z;
};

/**
 * Snapshot IMU coerente — aggiornato dal thread IMU (400 Hz) via double-buffer atomico.
 * Tutti i consumer (SPI TELEMETRY, monitor) leggono tramite imu_get_snapshot();
 * nessun consumer legge i registri raw direttamente.
 */
struct imu_snapshot {
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float temp;
	float quat_w;
	float quat_x;
	float quat_y;
	float quat_z;
	float zworld_x;
	float zworld_y;
	float zworld_z;
	float wrist_roll_rate;
	float wrist_pitch_rate;
	float wrist_yaw_rate;
	uint32_t sample_counter; /* Contatore snapshot IMU validi (monotono, wrap uint32) */
	uint32_t timestamp_us;   /* Timestamp monotono di produzione snapshot (microsecondi, wrap uint32) */
};

typedef struct imu_snapshot imu_snapshot_t;

/* Inizializzazione IMU (chiamata dal thread IMU dopo deferred startup) */
int imu_init(void);

/* Aggiorna l'orientamento (chiamata a dt fisso dal thread IMU, 400 Hz) */
void imu_update_orientation(float dt_s);

/* Restituisce l'ultimo quaternione stimato — lettura dal double-buffer attivo */
void imu_get_quaternion(struct imu_quat *out);

/**
 * Lettura snapshot IMU completo dal double-buffer atomico.
 * Lock-free: singolo atomic_get + copia strutturale, zero retry.
 * Ritorna true se esiste almeno uno snapshot valido, false se IMU non
 * ha ancora prodotto almeno uno snapshot valido.
 */
bool imu_get_snapshot(imu_snapshot_t *out);


/* I2C bus recovery (AN4559): sblocca SDA/SCL dopo reset/flash */
void imu_i2c_bus_recovery(void);

/* True se l'orientamento IMU (quaternione BNO085) è basato su campioni validi */
bool imu_is_orientation_valid(void);

/** Azzera orientamento e snapshot quando IMU è disabilitata (IMUOFF). Da chiamare da UART su IMUOFF. */
void imu_clear_orientation_state(void);

/* Verifica se IMU è disponibile */
bool imu_is_available(void);

#endif /* IMU_H */
