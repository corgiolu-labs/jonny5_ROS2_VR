/*
 * sim_rt.c — the real core/rt_loop.c, compiled into the simulator.
 *
 * rt_loop_step() is static in rt_loop.c: including the file gives the simulator the
 * exact same step function the STM32 runs at 1 kHz. The thread / TIM6 / IMU-thread
 * code is compiled but never started (k_thread_create is a no-op here).
 */
#include "../../src/core/rt_loop.c"

/* One iteration of rt_thread_fn(): E-STOP sampled first, then the control step. */
void sim_rt_tick_once(void)
{
    g_rt_loop_period_us = 1000U;   /* the TIM6 tick is exact in the simulator */
    estop_poll_tick();
    rt_loop_step();
}
