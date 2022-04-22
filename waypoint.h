#pragma once
/*
 * Declarations for waypoints.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "config.h"

/*
 * void waypoint_run_sm_enter(void);
 * void waypoint_init(void);
 * void waypoint_set(void);
 * void waypoint_reset(void);
 * void waypoint_goto_next(void);
 * void waypoint_get_arrived(void);
 */

void waypoint_print(config_waypoint_t waypoint);
