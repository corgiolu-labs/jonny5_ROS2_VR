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

