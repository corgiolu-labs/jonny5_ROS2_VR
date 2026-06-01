/*
 * RT Loop - Header
 *
 * Loop realtime deterministico a 1 kHz.
 * Refresh watchdog, gestione state machine.
 */

#ifndef RT_LOOP_H
#define RT_LOOP_H

#include <stdbool.h>
#include <stdint.h>

/* RT loop period — single definition used by all modules */
#define RT_LOOP_PERIOD_MS  1

void rt_loop_init(void);
void rt_loop_start(void);

/* Diagnostica: tick counter del RT loop (incrementa a 1kHz quando vivo) */
extern volatile uint32_t g_rt_loop_ticks;
extern volatile uint8_t g_rt_loop_stage;

/* Periodo del RT loop misurato runtime, in microsecondi (EWMA su ~16 iter).
 * Pubblicato in TELEMETRY reserved bytes [62-63] BE uint16. 0 = non ancora
 * misurato (primi tick dopo il boot). Target di design: 1000 us = 1 kHz. */
extern volatile uint16_t g_rt_loop_period_us;

/* Tempo di esecuzione puro di rt_loop_step() in microsecondi (EWMA ~16 iter).
 * 0 = non ancora misurato. Confronto con g_rt_loop_period_us per distinguere
 * costo di calcolo vs costo di scheduling/quantizzazione tick. */
extern volatile uint16_t g_rt_step_us;

/* Diagnostica: fatal error count/reason (popolato da fatal_diag.c) */
extern volatile uint32_t g_fatal_count;
extern volatile uint32_t g_fatal_reason;

/* Diagnostica teleop VR gating */
extern volatile uint8_t g_vr_armed;
extern volatile uint8_t g_vr_freeze_active;
extern volatile uint32_t g_vr_guard_block_count;
extern volatile uint8_t g_vr_input_active;

/* Diagnostica IMU thread liveness */
extern volatile uint32_t g_imu_thread_ticks;
extern volatile uint8_t g_imu_thread_stage;
extern volatile uint8_t g_imu_reads_enabled;

#endif /* RT_LOOP_H */
