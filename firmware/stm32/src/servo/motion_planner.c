/*
 * Motion Planner - Minimal stub
 *
 * stop_all() is called by rt_loop on deadman release and mode guard.
 * The original trajectory_state array was never activated — this is
 * now a clean no-op. Actual trajectory control lives in j5vr_setpose.c.
 */

#include "servo/motion_planner.h"

void motion_planner_stop_all(void)
{
    /* No-op: trajectory state managed by j5vr_setpose_tick() */
}
