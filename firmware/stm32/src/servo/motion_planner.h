/*
 * Motion Planner - Header (minimal)
 *
 * Provides stop_all() to halt active trajectories on deadman release.
 */

#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include "core/rt_loop.h"  /* RT_LOOP_PERIOD_MS */

void motion_planner_stop_all(void);

#endif /* MOTION_PLANNER_H */
