/*
 * Implementation for waypoints.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <limits.h>
#include "calibrate.h"
#include "config.h"
#include "crc32c.h"
#include "hardware.h"
#include "log.h"
#include "motor.h"
#include "sm.h"
#include "waypoint.h"

static sm_state_t exit_to_state = { 0 };  // Transition to this state when done running waypoints.

static int current_index = 0;
static waypoint_t current_waypoint = { 0 };
static int steps_remaining = 0;

static void break_handler(void);
static void start(void);
static void run(void);
static void stop(void);

static const char state_waypoint_start_name[] PROGMEM = "waypoint_start";
static const char state_waypoint_run_name[] PROGMEM = "waypoint_stop";
static const char state_waypoint_stop_name[] PROGMEM = "waypoint_run";

static const sm_state_t state_waypoint_start = { .run = start, .break_handler = break_handler, .process_break_only = true, .name = state_waypoint_start_name };
static const sm_state_t state_waypoint_run = { .run = run, .break_handler = break_handler, .process_break_only = true, .name = state_waypoint_run_name };
static const sm_state_t state_waypoint_stop = { .run = stop, .break_handler = break_handler, .process_break_only = true, .name = state_waypoint_stop_name };


int waypoint_get_max_count(void)
{
    int start_address = 0;
    int nbytes = 0;

    config_get_waypoint_eeprom_region(&start_address, &nbytes);

    static const int max_num_waypoints = nbytes / sizeof(waypoint_t);

    return max_num_waypoints;
}

int waypoint_get_used_count(void)
{
    int max = waypoint_get_max_count();
    int count = 0;

    for (int i = 0; i < max; i++) {
        if (waypoint_get(i).command != -1)
            count++;
    }

    return count;
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

    int addr = get_eeprom_address(index);

    EEPROM.put(addr, waypoint);
}

void waypoint_insert_before(int index, waypoint_t waypoint)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count() - 1);

    // Move all waypoints from index to end of list toward end of list by 1, adjust goto targets.
    for (int i = waypoint_get_max_count() - 2; i >= index; i--) {
        waypoint_t copy = waypoint_get(i);
        if (((copy.command == WAYPOINT_COMMAND_GOTO_STEP) ||
             (copy.command == WAYPOINT_COMMAND_IF_IO_PIN_GOTO_STEP)) &&
            copy.io_goto.step >= index)
            copy.io_goto.step++;
        waypoint_set(i + 1, copy);
    }

    // Set our waypoint.
    waypoint_set(index, waypoint);

    // Start from the waypoint just inserted toward start of list and adjust all goto targets.
    for (int i = index; i >= 0; i--) {
        waypoint_t copy = waypoint_get(i);
        if (((copy.command == WAYPOINT_COMMAND_GOTO_STEP) ||
             (copy.command == WAYPOINT_COMMAND_IF_IO_PIN_GOTO_STEP)) &&
            copy.io_goto.step >= index) {
            copy.io_goto.step++;
            waypoint_set(i, copy);
        }
    }
}

void waypoint_append(waypoint_t waypoint)
{
    int last_index = -1;

    for (int i = 0; i < waypoint_get_max_count(); i++) {
        waypoint_t waypoint2 = waypoint_get(i);
        if (waypoint2.command != -1)
            last_index = i;
    }

    waypoint_set(last_index + 1, waypoint);
}

void waypoint_delete(int index)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count());

    waypoint_t waypoint = { 0 };

    EEPROM.put(get_eeprom_address(index), waypoint);
}

static void print_move(char *to_string, waypoint_t waypoint)
{
    log_writeln(F("Move %s a:%d b:%d c:%d d:%d e:%d f:%d."),
                to_string,
                (int)waypoint.motor[0],
                (int)waypoint.motor[1],
                (int)waypoint.motor[2],
                (int)waypoint.motor[3],
                (int)waypoint.motor[4],
                (int)waypoint.motor[5]);
}

static void print_enabled_motors(int mask)
{
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (mask & (1 << i))
            log_write(F("%c"), 'A' + i);
    }
}

void waypoint_print(int index)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count());

    waypoint_t waypoint = waypoint_get(index);

    log_write(F("%d: "), index);

    switch (waypoint.command) {
    case WAYPOINT_COMMAND_MOVE_AT:     // Fallthrough.
        print_move("to", waypoint);
        break;
    case WAYPOINT_COMMAND_MOVE_BESIDE:     // Fallthough.
        print_move("beside", waypoint);
        break;
    case WAYPOINT_COMMAND_MOVE_CLOSE:     // Fallthrough.
        print_move("close to", waypoint);
        break;
    case WAYPOINT_COMMAND_MOVE_APPROACHING:
        print_move("approaching", waypoint);
        break;
    case WAYPOINT_COMMAND_SET_ENABLED_MOTORS:
        if (waypoint.enabled_motors_mask == 0) {
            log_writeln(F("Disable all motors."));
        } else {
            log_write(F("Enable motors "));
            print_enabled_motors(waypoint.enabled_motors_mask);
            log_writeln(F("."));
        }
        break;
    case WAYPOINT_COMMAND_GOTO_STEP:
        log_writeln(F("Goto %d."),
                    waypoint.io_goto.step);
        break;
    case WAYPOINT_COMMAND_IF_IO_PIN_GOTO_STEP:
        log_writeln(F("If I/O pin %d then goto %d."),
                    waypoint.io_goto.pin,
                    waypoint.io_goto.step);
        break;
    case WAYPOINT_COMMAND_WAIT_IO_PIN:
        log_writeln(F("Wait I/O pin %d."),
                    waypoint.io_goto.pin);
        break;
    case WAYPOINT_COMMAND_CALIBRATE_HOME_SWITCHES_AND_LIMITS:
        if (waypoint.enabled_motors_mask == 0) {
            log_writeln(F("Calibrate home switches and limits for all motors."));
        } else {
            log_write(F("Calibrate home switches and limits for motors "));
            print_enabled_motors(waypoint.enabled_motors_mask);
            log_writeln(F("."));
        }
        break;
    case WAYPOINT_COMMAND_CALIBRATE_HOME_SWITCHES:
        if (waypoint.enabled_motors_mask == 0) {
            log_writeln(F("Calibrate home switches."));
        } else {
            log_write(F("Calibrate home switches for motors "));
            print_enabled_motors(waypoint.enabled_motors_mask);
            log_writeln(F("."));
        }
        break;
    case WAYPOINT_COMMAND_WAIT_MILLIS:
        log_writeln(F("Wait %ld milliseconds."),
                    waypoint.wait_millis);
        break;
    default:
        log_writeln(F("waypoint.command == %d"), waypoint.command);
        assert(false);
        break;
    }
}

void waypoint_print_all_used(void)
{
    for (int i = 0; i < waypoint_get_max_count(); i++) {
        waypoint_t waypoint = waypoint_get(i);
        if (waypoint.command != -1)
            waypoint_print(i);
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

    sm_set_next_state(state_waypoint_start);
}

static void break_handler(void)
{
    log_writeln(F("Break detected. Stopping waypoint sequence."));
    motor_disable_all();

    sm_set_next_state(state_waypoint_stop);
}

void start(void)
{
    assert(current_index >= 0);
    assert(current_index < waypoint_get_max_count());

    for (int i = MOTOR_ID_A; i < MOTOR_ID_COUNT; i++) {
        motor_set_enabled((motor_id_t)i, true);
    }

    log_writeln(F("Execuing waypoint sequence. Press <CTRL+C> to stop."));

    sm_set_next_state(state_waypoint_run);
}

static bool check_progress(waypoint_t waypoint)
{
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (!motor_get_enabled((motor_id_t)i))
            continue;

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
        if (!motor_get_enabled((motor_id_t)i))
            continue;
        motor_set_target_encoder((motor_id_t)i, waypoint.motor[i]);
    }
}

static void run(void)
{
    assert(current_index >= 0);

    static waypoint_t waypoint = { 0 };

    static bool wait_millis_run = false;
    static unsigned long wait_millis_start = -1;     // millis() returns unsigned_long.
    static int prev_waypoint_index = -1;

    if ((current_index >= waypoint_get_max_count()) || (steps_remaining == 0)) {
        prev_waypoint_index = -1;
        sm_set_next_state(state_waypoint_stop);
        return;
    }

    if (prev_waypoint_index != current_index) {
        waypoint = waypoint_get(current_index);
        if (waypoint.command != -1) {
            waypoint_print(current_index);
            if ((prev_waypoint_index != -1) && (steps_remaining > 0))
                steps_remaining--;
        }
        prev_waypoint_index = current_index;
    }

    switch (waypoint.command) {
    case -1:
        current_index++;
        break;
    case WAYPOINT_COMMAND_MOVE_AT:     // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_BESIDE:     // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_CLOSE:     // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_APPROACHING:
        if (check_progress(waypoint))
            current_index++;
        else
            set_target_waypoint(waypoint);  // TODO: Only set when index changed.
        break;
    case WAYPOINT_COMMAND_GOTO_STEP:
        current_index = waypoint.io_goto.step;
        break;
    case WAYPOINT_COMMAND_SET_ENABLED_MOTORS:
        motor_set_enabled_mask(waypoint.enabled_motors_mask);
        current_index++;
        break;
    case WAYPOINT_COMMAND_IF_IO_PIN_GOTO_STEP:
        if (hardware_get_header_pin_pressed(waypoint.io_goto.pin)) {
            log_writeln(F("I/O pin %d triggered."), waypoint.io_goto.pin);
            current_index = waypoint.io_goto.step;
        }
        break;
    case WAYPOINT_COMMAND_WAIT_IO_PIN:
        if (hardware_get_header_pin_pressed(waypoint.io_goto.pin)) {
            log_writeln(F("Pin %d triggered."), waypoint.io_goto.pin);
            current_index++;
        }
        break;
    case WAYPOINT_COMMAND_CALIBRATE_HOME_SWITCHES_AND_LIMITS:
        // Sets up state machine to start calibrating.
        calibrate_home_switch_and_limits(waypoint.enabled_motors_mask, 100);
        current_index++;
        break;
    case WAYPOINT_COMMAND_CALIBRATE_HOME_SWITCHES:
        // Sets up state machine to start calibrating.
        calibrate_home_switch(waypoint.enabled_motors_mask, 100);
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

    sm_set_next_state(exit_to_state);
}
