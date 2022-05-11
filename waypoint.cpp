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

static void (*sm_exit_to_state)(void) = NULL;
static int motion_status[MOTOR_ID_COUNT] = { 0 };  // Motion Status:
static int tracking = 0;  // TODO: Change to bool or enum.
static int track_report_encoder_value[MOTOR_ID_COUNT] = { 0 };  // Last value logged while tracking.
static int limit_prev[MOTOR_ID_COUNT] = { 0 };  // Limit/Home switch Previous Value

static void move_to_a_waypoint_angle();
static void move_to_waypoint_angle(waypoint_t waypoint);

static waypoint_t current_waypoint = { 0 };

static void sm_execute_next_step(void);
static void sm_track_move_to(void);
static void sm_interrogate_limit_switches(void);
static void sm_wait_millis(void);
static void sm_exit(void);

static void interrogate_limit_switches()
{
}

void waypoint_sm_enter(void)
{
    log_writeln(F("waypoint_sm_enter"));
    sm_set_state_name(F("waypoint_sm_enter"));

    sm_exit_to_state = sm_get_state();  // State to transition to when waypoints done running.
    sm_set_next_state(sm_execute_next_step);

    current_waypoint.step = 0;
}

static void sm_execute_next_step(void)
{
    log_writeln(F("waypoint sm_run"));
    sm_set_state_name(F("waypoint sm_run"));

    log_write(F("Waypoint: Reading step %d."), current_waypoint.step);

    assert(current_waypoint.step < waypoint_get_max_count());
    current_waypoint = waypoint_get(current_waypoint.step);

    if (current_waypoint.step == -1)
        sm_set_next_state(sm_exit);

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
        sm_set_next_state(sm_track_move_to);
        break;
    case WAYPOINT_COMMAND_GOTO_STEP:
        log_writeln(F("Waypoint %d: goto step %d."), current_waypoint.step);
        current_waypoint.step = current_waypoint.goto_step;
        break;
    case WAYPOINT_COMMAND_GOTO_STEP_IF_IO:
        // pin = waypoint.b;
        log_writeln(F("Waypoint %d: goto step %d if button pressed."), current_waypoint.goto_step);
        if (hardware_get_button_pressed()) // TODO: Support configurable expansion IO line.
            current_waypoint.step = current_waypoint.goto_step;
        else
            current_waypoint.step++;
        break;
    case WAYPOINT_COMMAND_INTERROGATE_SWITCHES:
        log_writeln(F("Waypoint %d: interrogate limit switches."), current_waypoint.goto_step);
        sm_set_next_state(sm_interrogate_limit_switches);
        break;
    case WAYPOINT_COMMAND_WAIT_MILLIS:
        log_writeln(F("Waypoint %d: wait %d milliseconds."), current_waypoint.wait_millis);
        sm_set_next_state(sm_wait_millis);
        break;
    }
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
        ((current_waypoint.command == 'D') && check_progress(PROGRESS_APPROACHING))) {
        current_waypoint.step++;
        track_millis = -1;
    }
}

static void sm_interrogate_limit_switches(void)
{
    log_writeln(F(""));
    sm_set_state_name(F("waypoint_sm_interrogate_limit_switches"));
}

static void sm_wait_millis(void)
{
    assert(current_waypoint.wait_millis >= 0);
    static int wait_start_millis = -1;

    log_writeln(F("waypoint sm_wait_millis"));
    sm_set_state_name(F("waypoint sm_wait_millis"));

    if (wait_start_millis == -1)
        wait_start_millis = millis();

    if ((millis() - wait_start_millis) > current_waypoint.wait_millis) {
        // TODO: Look into millis() rollover and handle appropriately.
        current_waypoint.wait_millis = wait_start_millis = -1;
        sm_set_next_state(sm_execute_next_step);
    }
}

static void sm_exit(void)
{
    log_writeln(F("waypoint sm_exit"));
    sm_set_state_name(F("waypoint sm_exit"));

    motor_disable_all();
    memset(&current_waypoint, 0, sizeof(waypoint_t));
    sm_set_next_state(sm_exit_to_state);
    log_writeln(F("Done running waypoint sequence."));
}



static String in_buffer = "";
static String string_splits[8];  //Used to store waypoints
static String get_string_part_at_specific_index(String StringToSplit, char SplitChar, int StringPartIndex);

static void split_waypoint(String waypoint)
{
    for (int i = 0; i < 8; i++) {
        string_splits[i] = get_string_part_at_specific_index(waypoint, '!', i);
    }
}

static String get_string_part_at_specific_index(String StringToSplit, char SplitChar, int StringPartIndex)
{
    String originallyString = StringToSplit;
    String outString = "";

    for (int i1 = 0; i1 <= StringPartIndex; i1++) {
        outString = "";                //if the for loop starts again reset the outString (in this case other part of the String is needed to take out)
        int SplitIndex = StringToSplit.indexOf(SplitChar);  //set the SplitIndex with the position of the SplitChar in StringToSplit

        if (SplitIndex == -1)          //is true, if no Char is found at the given Index

            //outString += "Error in get_string_part_at_specific_index: No SplitChar found at String '" + originallyString + "' since StringPart '" + (i1-1) + "'";    //just to find Errors
            return outString;
        for (int i2 = 0; i2 < SplitIndex; i2++) {
            outString += StringToSplit.charAt(i2);  //write the char at Position 0 of StringToSplit to outString
        }
        StringToSplit = StringToSplit.substring(StringToSplit.indexOf(SplitChar) + 1);  //change the String to the Substring starting at the position+1 where last SplitChar found
    }
    return outString;
}

static void set_waypoint_angle(waypoint_t waypoint)
{
    in_buffer.setCharAt(0, ' ');
    int step = in_buffer.toInt();

    split_waypoint(in_buffer);
    waypoint.step = step;
    char command = string_splits[1][0];  // TODO: validate command.

    waypoint.command = command;
    waypoint.motor.a = string_splits[2].toFloat();
    waypoint.motor.b = string_splits[3].toFloat();
    waypoint.motor.c = string_splits[4].toFloat();
    waypoint.motor.d = string_splits[5].toFloat();
    waypoint.motor.e = string_splits[6].toFloat();
    waypoint.motor.f = string_splits[7].toFloat();

    waypoint_set(step, waypoint);
    waypoint_print(waypoint);

    log_writeln(F("set."));
}

static void get_waypoint_angle(void)
{
    in_buffer.setCharAt(0, ' ');
    int index = in_buffer.toInt();
    waypoint_t waypoint = waypoint_get(index);

    waypoint_print(waypoint);
}

static void move_to_a_waypoint_angle(void)
{
    in_buffer.setCharAt(0, ' ');
    int step = in_buffer.toInt();

    log_write(F("Move to "));
    waypoint_t waypoint = waypoint_get(step);

    waypoint_print(waypoint);
    move_to_waypoint_angle(waypoint);
}

static void move_to_waypoint_angle(waypoint_t waypoint)
{
    motor_set_target_angle(MOTOR_ID_A, waypoint.motor.a);
    motor_set_target_angle(MOTOR_ID_B, waypoint.motor.b);
    motor_set_target_angle(MOTOR_ID_C, waypoint.motor.c);
    motor_set_target_angle(MOTOR_ID_D, waypoint.motor.d);
    motor_set_target_angle(MOTOR_ID_E, waypoint.motor.e);
    motor_set_target_angle(MOTOR_ID_F, waypoint.motor.f);
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
        waypoint.step = -1;
    }

    return waypoint;
}

void waypoint_set(int index, waypoint_t waypoint)
{
    assert(index >= 0);
    assert(index < waypoint_get_max_count());
    assert(waypoint.step == index);

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

void waypoint_print(waypoint_t waypoint)
{
    assert(waypoint.step != -1);

    switch (waypoint.command) {
    case WAYPOINT_COMMAND_MOVE_AT:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_BESIDE:  // Fallthough.
    case WAYPOINT_COMMAND_MOVE_CLOSE:  // Fallthrough.
    case WAYPOINT_COMMAND_MOVE_APPROACHING:
        log_writeln(F("waypoint: step:%d command:%c a:%d b:%d c:%d d:%d e:%d f:%d"),
                    waypoint.step,
                    waypoint.command,
                    (int)waypoint.motor.a,
                    (int)waypoint.motor.b,
                    (int)waypoint.motor.c,
                    (int)waypoint.motor.d,
                    (int)waypoint.motor.e,
                    (int)waypoint.motor.f);
        break;
    case WAYPOINT_COMMAND_GOTO_STEP:
        log_writeln(F("waypoint: step:%d command:%c (goto step) destination_step: %d"),
                    waypoint.step,
                    waypoint.command,
                    waypoint.goto_step);
        break;
    case WAYPOINT_COMMAND_GOTO_STEP_IF_IO:
        log_writeln(F("waypoint: step:%d command:%c (goto step if IO) destination_step: %d"),
                    waypoint.step,
                    waypoint.command,
                    waypoint.goto_step);
        break;
    case WAYPOINT_COMMAND_INTERROGATE_SWITCHES:
        log_writeln(F("waypoint: step:%d command:%c (interrogate limit switches)"),
                    waypoint.step,
                    waypoint.command);
        break;
    case WAYPOINT_COMMAND_WAIT_MILLIS:
        log_writeln(F("waypoint: step:%d command:%c (wait millis) %d"),
                    waypoint.step,
                    waypoint.command,
                    waypoint.wait_millis);
        break;
    default:
        log_writeln(F("waypoint.command=%d"), waypoint.command);
        assert(false);
        break;
    }
}
