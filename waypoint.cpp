/*
 * Implementation for waypoints.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <limits.h>
#include "config.h"
#include "crc32c.h"
#include "hardware.h"
#include "log.h"
#include "motor.h"
#include "sm.h"
#include "waypoint.h"

static sm_state_func exit_to_state = NULL;  // Transition to this state when done running waypoints.

static int current_index = 0;
static waypoint_t current_waypoint = { 0 };
static int steps_remaining = 0;

static void break_handler(void);
static void start(void);
static void run(void);
static void stop(void);

int waypoint_get_max_count(void)
{
    int start_address = 0;
    int nbytes = 0;

    config_get_waypoint_eeprom_region(&start_address, &nbytes);

    static const int max_num_waypoints = nbytes / sizeof(waypoint_t);

    return max_num_waypoints;
}

static int get_eeprom_address(int index)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count());

    int start_address = 0;
    int nbytes = 0;

    config_get_waypoint_eeprom_region(&start_address, &nbytes);

    return start_address + index * sizeof(waypoint_t);
}

waypoint_t waypoint_get(int index)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count());

    waypoint_t waypoint;

    EEPROM.get(get_eeprom_address(index), waypoint);

    uint32_t saved_crc = waypoint.crc;

    waypoint.crc = 0;  // Set to 0, as the crc is part of the crc calculation.
    waypoint.crc = crc32c_calculate(&waypoint, sizeof(waypoint_t));

    if (waypoint.crc != saved_crc) {
        // Bad CRC. Likely reading a waypoint that was never written, but zero it anyway and set step == -1.
        memset(&waypoint, 0, sizeof(waypoint_t));
        waypoint.command = -1;
    }

    return waypoint;
}

void waypoint_set(int index, waypoint_t waypoint)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count());

    waypoint.crc = 0;  // Set to zero to make the CRC reproducible.
    waypoint.crc = crc32c_calculate(&waypoint, sizeof(waypoint_t));

    EEPROM.put(get_eeprom_address(index), waypoint);
}

void waypoint_delete(int index)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count());

    waypoint_t waypoint = { 0 };

    EEPROM.put(get_eeprom_address(index), waypoint);
}

static void print_move(int index, char *to_string, waypoint_t waypoint)
{
    log_writeln(F("%d: Move %s a:%d b:%d c:%d d:%d e:%d f:%d."),
                index,
                to_string,
                (int)waypoint.motor[0],
                (int)waypoint.motor[1],
                (int)waypoint.motor[2],
                (int)waypoint.motor[3],
                (int)waypoint.motor[4],
                (int)waypoint.motor[5]);
}

void waypoint_print(int index)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count());

    waypoint_t waypoint = waypoint_get(index);

    switch (waypoint.command) {
    case WAYPOINT_COMMAND_MOVE_AT:  // Fallthrough.
        print_move(index, "to", waypoint);
        break;
    case WAYPOINT_COMMAND_MOVE_BESIDE:  // Fallthough.
        print_move(index, "beside", waypoint);
        break;
    case WAYPOINT_COMMAND_MOVE_CLOSE:  // Fallthrough.
        print_move(index, "close to", waypoint);
        break;
    case WAYPOINT_COMMAND_MOVE_APPROACHING:
        print_move(index, "approaching", waypoint);
        break;
    case WAYPOINT_COMMAND_GOTO_STEP:
        log_writeln(F("%d: Goto %d."),
                    index,
                    waypoint.io_goto.step);
        break;
    case WAYPOINT_COMMAND_IF_IO_PIN_GOTO_STEP:
        log_writeln(F("%d: If i/o pin %d then goto %d."),
                    index,
                    waypoint.io_goto.pin,
                    waypoint.io_goto.step);
        break;
    case WAYPOINT_COMMAND_WAIT_IO_PIN:
        log_writeln(F("%d: Wait i/o pin %d."),
                    index,
                    waypoint.io_goto.pin);
        break;
    case WAYPOINT_COMMAND_INTERROGATE_SWITCHES:
        log_writeln(F("%d: Interrogate switches."), index);
        // TODO.
        break;
    case WAYPOINT_COMMAND_WAIT_MILLIS:
        log_writeln(F("%d: Wait %ld milliseconds."),
                    index,
                    waypoint.wait_millis);
        break;
    default:
        log_writeln(F("waypoint.command == %d"), waypoint.command);
        assert(false);
        break;
    }
}

void waypoint_run(int start_index, int count)
{
    assert(start_index >= 0);
    assert(start_index < waypoint_get_max_count());

    log_writeln(F("waypoint_run starting at step %d count %d"), start_index, count);

    current_index = start_index;
    steps_remaining = count;

    exit_to_state = sm_get_state();
    sm_set_next_state(start, break_handler);
}

static void break_handler(void)
{
    log_writeln(F("Break detected. Stopping waypoint sequence."));
    motor_disable_all();
    sm_set_next_state(stop, NULL);
}

void start(void)
{
    assert(current_index >= 0);
    assert(current_index < waypoint_get_max_count());

    for (int i = MOTOR_ID_A; i <= MOTOR_ID_LAST; i++) {
        motor_set_enabled((motor_id_t)i, true);
    }

    log_writeln(F("Execuing waypoint sequence. Press <CTRL+C> to stop."));
    sm_set_next_state(run, break_handler);
}

static bool check_progress(waypoint_t waypoint)
{
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        switch (waypoint.command) {
        case WAYPOINT_COMMAND_MOVE_AT:
            if (motor_get_encoder((motor_id_t)i) != waypoint.motor[i])
                return false;
            break;
        case WAYPOINT_COMMAND_MOVE_BESIDE:
            if (abs(motor_get_encoder((motor_id_t)i) - waypoint.motor[i]) > 1)
                return false;
            break;
        case WAYPOINT_COMMAND_MOVE_CLOSE:
            if (abs(motor_get_encoder((motor_id_t)i) - waypoint.motor[i]) > 30)
                return false;
            break;
        case WAYPOINT_COMMAND_MOVE_APPROACHING:
            if (abs(motor_get_encoder((motor_id_t)i) - waypoint.motor[i]) > 200)
                return false;
            break;
        default:
            assert(false);
            break;
        }
    }

    return true;
}

static bool set_target_waypoint(waypoint_t waypoint)
{
    assert((waypoint.command == WAYPOINT_COMMAND_MOVE_AT) ||
           (waypoint.command == WAYPOINT_COMMAND_MOVE_BESIDE) ||
           (waypoint.command == WAYPOINT_COMMAND_MOVE_CLOSE) ||
           (waypoint.command == WAYPOINT_COMMAND_MOVE_APPROACHING));

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        motor_set_target_encoder((motor_id_t)i, waypoint.motor[i]);
    }
}

static void run(void)
{
    assert(current_index >= 0);
    assert(current_index < waypoint_get_max_count());

    static waypoint_t waypoint = { 0 };

    static bool wait_millis_run = false;
    static unsigned long wait_millis_start = -1;  // millis() returns unsigned_long.

    static int prev_waypoint_index = -1;

    if ((current_index >= waypoint_get_max_count()) || (steps_remaining == 0)) {
        sm_set_next_state(stop, NULL);
        return;
    }

    if (prev_waypoint_index != current_index) {
        waypoint = waypoint_get(current_index);
        if (waypoint.command != -1)
            waypoint_print(current_index);
        prev_waypoint_index = current_index;
        if ((waypoint.command != -1) && (steps_remaining > 0))
            steps_remaining--;
    }

    switch (waypoint.command) {
    case -1:
        current_index++;
        break;
    case WAYPOINT_COMMAND_MOVE_AT:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_BESIDE:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_CLOSE:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_APPROACHING:
        if (check_progress(waypoint))
            current_index++;
        else
            set_target_waypoint(waypoint);  // TODO: Only set when index changed.
        break;
    case WAYPOINT_COMMAND_GOTO_STEP:
        current_index = waypoint.io_goto.step;
        break;
    case WAYPOINT_COMMAND_IF_IO_PIN_GOTO_STEP:
        if (hardware_get_header_pin_pressed(waypoint.io_goto.pin))
            current_index = waypoint.io_goto.step;
        else
            current_index++;
        break;
    case WAYPOINT_COMMAND_WAIT_IO_PIN:
        if (hardware_get_header_pin_pressed(waypoint.io_goto.pin)) {
            log_writeln(F("Pin %d triggered."), waypoint.io_goto.pin);
            current_index = waypoint.io_goto.step;
        }
        break;
    case WAYPOINT_COMMAND_INTERROGATE_SWITCHES:
        // TODO.
        current_index++;
        break;
    case WAYPOINT_COMMAND_WAIT_MILLIS:
        if (wait_millis_run == false) {
            wait_millis_start = millis();
            wait_millis_run = true;
        }

        if (millis() - wait_millis_start >= waypoint.wait_millis) {
            wait_millis_run = false;;
            current_index++;
        }
        break;
    default:
        log_writeln(F("Unexpected command %d '%c'"), waypoint.command, waypoint.command);
        assert(false);
        break;
    }
}

static void stop(void)
{
    current_index = -1;

    motor_disable_all();
    memset(&current_waypoint, 0, sizeof(waypoint_t));
    log_writeln(F("Done running waypoint sequence."));

    sm_set_next_state(exit_to_state, NULL);
}
