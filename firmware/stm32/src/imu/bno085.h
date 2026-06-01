/*
 * JONNY5 - Minimal BNO085 driver (SHTP Rotation Vector su I2C1)
 *
 * Talks to a Hillcrest/CEVA BNO085 over I2C using the SHTP transport protocol.
 * Exposes only what Phase 3 needs to validate the sensor: init + rotation-vector
 * polling + readiness flag.
 *
 * Reversibility: this is a self-contained module. Nothing in imu.h / imu.c calls
 * into it. Until wired into the main IMU pipeline, it can be removed from the
 * build by simply not linking bno085.c.
 */

#ifndef BNO085_H
#define BNO085_H

#include <stdbool.h>
#include <stdint.h>

/* Default I2C address on this board (confirmed by i2c scan: ADR tied high). */
#define BNO085_I2C_ADDR       0x4B

/* Initialize the BNO085:
 *   1. Verify the I2C bus is ready.
 *   2. Drain the post-reset advertisement packets already in the device queue.
 *   3. Send Set-Feature for Rotation Vector at 100 Hz on channel 2.
 *
 * Must be called from thread context (uses k_sleep).
 * Returns 0 on success, negative errno on failure.
 */
int bno085_init(void);

/* Poll the device for a new Rotation Vector sensor report.
 *
 * Does one 32-byte I2C read and parses the SHTP packet if present.
 * If the packet contains a Rotation Vector input report, populates the outputs
 * with a unit quaternion and the sensor's accuracy status.
 *
 * Returns:
 *    1  if a new rotation-vector quaternion was parsed into the outputs
 *    0  if no rotation-vector packet was present (buffer empty, or different channel)
 *   <0  negative errno on I2C error
 *
 * q_w/q_x/q_y/q_z: output unit quaternion (real, i, j, k components).
 * accuracy:        SH-2 status bits 0..1 (0=unreliable … 3=high accuracy).
 */
int bno085_poll_quat(float *q_w, float *q_x, float *q_y, float *q_z,
		     uint8_t *accuracy);

/* Returns true after at least one Rotation Vector packet has been successfully
 * parsed since init. */
bool bno085_is_ready(void);

#endif /* BNO085_H */
