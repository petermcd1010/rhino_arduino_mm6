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

typedef enum {
    WAYPOINT_COMMAND_MOVE_AT              = 'A',
    WAYPOINT_COMMAND_MOVE_BESIDE,
    WAYPOINT_COMMAND_MOVE_CLOSE,
    WAYPOINT_COMMAND_MOVE_APPROACHING,
    WAYPOINT_COMMAND_GOTO_STEP            = 'G',
    WAYPOINT_COMMAND_GOTO_STEP_IF_IO      = 'J',
    WAYPOINT_COMMAND_INTERROGATE_SWITCHES = 'I',
    WAYPOINT_COMMAND_WAIT_MILLIS          = 'W',
} waypoint_command_t;

void waypoint_print(config_waypoint_t waypoint);