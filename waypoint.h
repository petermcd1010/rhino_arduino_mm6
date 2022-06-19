#pragma once
/*
 * Declarations for waypoints.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "config.h"

typedef enum {
    WAYPOINT_COMMAND_MOVE_AT                            = 'A',
    WAYPOINT_COMMAND_MOVE_BESIDE                        = 'B',
    WAYPOINT_COMMAND_MOVE_CLOSE                         = 'C',
    WAYPOINT_COMMAND_MOVE_APPROACHING                   = 'D',
    WAYPOINT_COMMAND_SET_ENABLED_MOTORS                 = 'E',
    WAYPOINT_COMMAND_GOTO_STEP                          = 'G',
    WAYPOINT_COMMAND_IF_IO_PIN_GOTO_STEP                = 'J',
    WAYPOINT_COMMAND_WAIT_IO_PIN                        = 'K',
    WAYPOINT_COMMAND_CALIBRATE_HOME_SWITCHES_AND_LIMITS = 'L',
    WAYPOINT_COMMAND_CALIBRATE_HOME_SWITCHES            = 'O',
    WAYPOINT_COMMAND_WAIT_MILLIS                        = 'W',
} waypoint_command_t;

typedef struct __attribute__((packed)) {
    uint32_t crc;
    char command;  // -1 if entry has not been set or has been deleted.
    union {
        int           enabled_motors_mask; // Mask of motors to enable.
        struct {
            int pin;  // Pin to potentially check or wait for trigger.
            int step;  // Step to potentially goto.
        } io_goto;
        unsigned long wait_millis;  // For wait command.
        float         motor[MOTOR_ID_COUNT];
    };
} waypoint_t;

int waypoint_get_max_count(void);  // Returns max number of waypoints that can be stored.
int waypoint_get_used_count(void);
waypoint_t waypoint_get(int index);
void waypoint_set(int index, waypoint_t waypoint);
void waypoint_insert_before(int index, waypoint_t waypoint);
void waypoint_append(waypoint_t waypoint);
void waypoint_delete(int index);
void waypoint_print(int index);
void waypoint_print_all_used(void);
void waypoint_run(int start_index, int count);  // If count == -1, then run until done.
