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
    WAYPOINT_COMMAND_IF_IO_PIN_GOTO_STEP  = 'J',
    WAYPOINT_COMMAND_WAIT_IO_PIN          = 'K',
    WAYPOINT_COMMAND_INTERROGATE_SWITCHES = 'I',
    WAYPOINT_COMMAND_WAIT_MILLIS          = 'W',
} waypoint_command_t;


typedef struct __attribute__((packed)) {
    uint32_t crc;
    char command;  // -1 if entry has not been set or has been deleted.
    union {
        struct {
            int pin;  // Pin to potentially check or wait for trigger.
            int step;  // Step to potentially goto.
        } io_goto;
        int wait_millis;  // For wait command.
        struct {
            float a;
            float b;
            float c;
            float d;
            float e;
            float f;
        } motor;
    };
} waypoint_t;

int waypoint_get_max_count(void);  // Returns max number of waypoints that can be stored.
waypoint_t waypoint_get(int index);
void waypoint_set(int index, waypoint_t waypoint);
void waypoint_delete(int index);
void waypoint_print(int index);
