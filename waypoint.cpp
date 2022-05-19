/*
 * Implementation for waypoints.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "crc32c.h"
#include "hardware.h"
#include "log.h"
#include "motor.h"
#include "sm.h"
#include "waypoint.h"

typedef enum {
    PROGRESS_AT = 0,  // Exactly at waypoint position.
    PROGRESS_BESIDE,  // Within 1 click of waypoint position.
    PROGRESS_CLOSE,  // Between 2 and 30 clicks of waypoint position.
    PROGRESS_APPROACHING,  // Between 30 and 200 clicks of waypoint position.
    PROGRESS_ON_WAY,  // More than 200 clicks away from waypoint position.
} progress_t;

static sm_state_func exit_to_state = NULL;  // Transition to this state when done running waypoints.

static int motion_status[MOTOR_ID_COUNT] = { 0 };  // Motion Status:
static int tracking = 0;  // TODO: Change to bool or enum.
static int track_report_encoder_value[MOTOR_ID_COUNT] = { 0 };  // Last value logged while tracking.
static int limit_prev[MOTOR_ID_COUNT] = { 0 };  // Limit/Home switch Previous Value.

static int current_waypoint_index = 0;
static waypoint_t current_waypoint = { 0 };


static void break_handler(void);
static void start(void);
static void run(void);
static void stop(void);

#if 0
static void interrogate_limit_switches()
{
}
#endif

static void sm_execute_next_step(void)
{
    log_writeln(F("waypoint sm_run"));
    sm_set_state_name(F("waypoint sm_run"));
#if 0
    assert(current_waypoint.step < waypoint_get_max_count());
    current_waypoint = waypoint_get(current_waypoint.step);

    if (current_waypoint.step == -1)
        sm_set_next_state(sm_exit, NULL);

    // TurnOnPID();
    switch (current_waypoint.command) {
    case WAYPOINT_COMMAND_MOVE_AT:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_BESIDE:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_CLOSE:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_APPROACHING:
        log_write(F("Waypoint %d: sm_run move_to "), current_waypoint.step);
        waypoint_print(current_waypoint);
        move_to_waypoint_angle(current_waypoint);
        tracking = 2;  // Track angles.
        sm_set_next_state(sm_track_move_to, NULL);
        break;
#endif
}

static void track_report(void)
{
    if (tracking > 0) {
        for (int motor_id = MOTOR_ID_A; motor_id <= MOTOR_ID_F; motor_id++) {
            int encoder_value = motor_get_encoder(motor_id);
            if (track_report_encoder_value[motor_id] != encoder_value) {
                log_write(F("@"));
                if (tracking == 1)
                    log_write(F("%c %d"), char(motor_id + 'A'), motor_get_encoder(motor_id));
                else if (tracking == 2)
                    log_write(F("%c %d"), char(motor_id + 'a'), (int)(motor_get_angle(motor_id)));
                log_writeln(F(":HS %d:"), limit_prev[motor_id]);
                track_report_encoder_value[motor_id] = encoder_value;
            }
        }
    }
}

static bool check_progress(progress_t progress)
{
    // Returns true if status <= progress.
    return
        (motion_status[MOTOR_ID_C] <= progress) ||
        (motion_status[MOTOR_ID_D] <= progress) ||
        (motion_status[MOTOR_ID_E] <= progress) ||
        (motion_status[MOTOR_ID_F] <= progress);
}

static void sm_track_move_to(void)
{
    log_writeln(F("waypoint sm_track_move_to"));
    sm_set_state_name(F("waypoint sm_track_move_to"));

    // TODO: Look into millis() rollover and handle appropriately.
    static int track_millis = -1;

    if ((track_millis < 0) || ((millis() - track_millis) >= 50)) {
        track_millis = millis();
        track_report();
    }

    if (((current_waypoint.command == 'A') && check_progress(PROGRESS_AT)) ||
        ((current_waypoint.command == 'B') && check_progress(PROGRESS_BESIDE)) ||
        ((current_waypoint.command == 'C') && check_progress(PROGRESS_CLOSE)) ||
        ((current_waypoint.command == 'D') && check_progress(PROGRESS_APPROACHING)))
        // current_waypoint.step++;
        track_millis = -1;
}

static void sm_exit(void)
{
    log_writeln(F("waypoint sm_exit"));
    sm_set_state_name(F("waypoint sm_exit"));

    motor_disable_all();
    memset(&current_waypoint, 0, sizeof(waypoint_t));
    log_writeln(F("Done running waypoint sequence."));
}

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

    waypoint.crc = 0;
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

void waypoint_print(int index)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count());

    waypoint_t waypoint = waypoint_get(index);

    switch (waypoint.command) {
    case WAYPOINT_COMMAND_MOVE_AT:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_BESIDE:  // Fallthough.
    case WAYPOINT_COMMAND_MOVE_CLOSE:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_APPROACHING:
        log_writeln(F("%d: Move to a:%d b:%d c:%d d:%d e:%d f:%d."),
                    index,
                    (int)waypoint.motor[0],
                    (int)waypoint.motor[1],
                    (int)waypoint.motor[2],
                    (int)waypoint.motor[3],
                    (int)waypoint.motor[4],
                    (int)waypoint.motor[5]);
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
        log_writeln(F("waypoint.command=%d"), waypoint.command);
        assert(false);
        break;
    }
}

void waypoint_run(int start_index, int count)
{
    assert(start_index >= 0);
    assert(start_index < waypoint_get_max_count());

    log_writeln(F("waypoint_run at %d count %d"), start_index, count);  // TODO: Use count.

    current_waypoint_index = start_index;

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
    assert(current_waypoint_index >= 0);
    assert(current_waypoint_index < waypoint_get_max_count());

    for (int i = MOTOR_ID_A; i <= MOTOR_ID_LAST; i++) {
        motor_set_enabled((motor_id_t)i, true);
    }

    log_writeln(F("Execuing waypoint sequence. Press <CTRL+C> to stop."));
    sm_set_next_state(run, break_handler);
}

static bool at_waypoint(waypoint_t waypoint)
{
    assert(waypoint.command == WAYPOINT_COMMAND_MOVE_AT);

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (motor_get_encoder((motor_id_t)i) != waypoint.motor[i])
            return false;
    }

    return true;
}

static bool set_target_waypoint(waypoint_t waypoint)
{
    assert(waypoint.command == WAYPOINT_COMMAND_MOVE_AT);

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        motor_set_target_encoder((motor_id_t)i, waypoint.motor[i]);
    }
}

static void run(void)
{
    assert(current_waypoint_index >= 0);
    assert(current_waypoint_index < waypoint_get_max_count());

    waypoint_t waypoint = waypoint_get(current_waypoint_index);  // TODO: Only get when index changes.

    static bool wait_millis_run = false;
    static unsigned long wait_millis_start = -1;  // millis() returns unsigned_long.

    static int prev_waypoint_index = -1;

    if ((prev_waypoint_index != current_waypoint_index) && (waypoint.command != -1)) {
        waypoint_print(current_waypoint_index);
        prev_waypoint_index = current_waypoint_index;
    }

    switch (waypoint.command) {
    case -1:
        current_waypoint_index++;
        break;
    case WAYPOINT_COMMAND_MOVE_AT:
        if (at_waypoint(waypoint))
            current_waypoint_index++;
        else
            set_target_waypoint(waypoint);  // TODO: Only set when index changed.
        break;
    case WAYPOINT_COMMAND_GOTO_STEP:
        current_waypoint_index = waypoint.io_goto.step;
        break;
    case WAYPOINT_COMMAND_IF_IO_PIN_GOTO_STEP:
        if (hardware_get_header_pin_pressed(waypoint.io_goto.pin))
            current_waypoint_index = waypoint.io_goto.step;
        else
            current_waypoint_index++;
        break;
    case WAYPOINT_COMMAND_WAIT_IO_PIN:
        if (hardware_get_header_pin_pressed(waypoint.io_goto.pin)) {
            log_writeln(F("Pin %d triggered."), waypoint.io_goto.pin);
            current_waypoint_index = waypoint.io_goto.step;
        }
        break;
    case WAYPOINT_COMMAND_INTERROGATE_SWITCHES:
        current_waypoint_index++;
        break;
    case WAYPOINT_COMMAND_WAIT_MILLIS:
        if (wait_millis_run == false) {
            wait_millis_start = millis();
            wait_millis_run = true;
        }

        if (millis() - wait_millis_start >= waypoint.wait_millis) {
            wait_millis_run = false;;
            current_waypoint_index++;
        }
        break;
    default:
        log_writeln(F("Unexpected command %d '%c'"), waypoint.command, waypoint.command);
        assert(false);
        break;
    }

    if (current_waypoint_index >= waypoint_get_max_count())
        sm_set_next_state(stop, NULL);
}

static void stop(void)
{
    current_waypoint_index = -1;

    motor_disable_all();
    memset(&current_waypoint, 0, sizeof(waypoint_t));
    log_writeln(F("Done running waypoint sequence."));

    sm_set_next_state(exit_to_state, NULL);
}
