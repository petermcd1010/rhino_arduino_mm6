/*
 * Implementation for waypoints.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "motor.h"
#include "sm.h"
#include "waypoint.h"

// k/K -> Run WayPointSeq()
// x -> set_waypoint_angle()
// r -> get_waypoint_angle()
// ! -> move_to_a_waypoint_angle()

typedef enum {
    COMMAND_MOVE_AT              = 'A',
    COMMAND_MOVE_BESIDE,
    COMMAND_MOVE_CLOSE,
    COMMAND_MOVE_APPROACHING,
    COMMAND_GOTO_STEP            = 'G',
    COMMAND_GOTO_STEP_IF_IO      = 'J',
    COMMAND_INTERROGATE_SWITCHES = 'I',
    COMMAND_WAIT_MILLIS          = 'W',
} command_t;

typedef enum {
    PROGRESS_AT = 0,  // Exactly at waypoint position.
    PROGRESS_BESIDE,  // Within 1 click of waypoint position.
    PROGRESS_CLOSE,  // Between 2 and 30 clicks of waypoint position.
    PROGRESS_APPROACHING,  // Between 30 and 200 clicks of waypoint position.
    PROGRESS_ON_WAY,  // More than 200 clicks away from waypoint position.
} progress_t;

static String in_buffer = "";
static String string_splits[8];  //Used to store waypoints
static int motion_status[MOTOR_ID_COUNT] = { 0 };  // Motion Status:
static int tracking = 0;
static int track_report_encoder_value[MOTOR_ID_COUNT] = { 0 };  // Last value logged while tracking.
static int limit_prev[MOTOR_ID_COUNT] = { 0 };  // Limit/Home switch Previous Value

static void move_to(config_waypoint_t waypoint);
static void move_to_a_waypoint_angle();
static void move_to_waypoint_angle(config_waypoint_t waypoint);
static bool check_progress(progress_t progress);  // Returns true if status <= progress.
static void track_report(void);
static String get_string_part_at_specific_index(String StringToSplit, char SplitChar, int StringPartIndex);

static void interrogate_limit_switches()
{
}

void waypoint_sm_run(void);

void waypoint_sm_enter(void)
{
    sm_set_state_name(F("waypoint_sm_enter"));
    sm_set_next_state(waypoint_sm_run);
}

static void waypoint_sm_run(void)
{
    sm_set_state_name(F("waypoint_sm_run"));
}

static void waypoint_sm_move_to(void)
{
    sm_set_state_name(F("waypoint_sm_move_to"));
}

static void waypoint_sm_interrogate_limit_switches(void)
{
    sm_set_state_name(F("waypoint_sm_interrogate_limit_switches"));
}

static void waypoint_sm_wait_millis(void)
{
    sm_set_state_name(F("waypoint_sm_wait_millis"));
}

static void waypoint_run_sequence(void)
{
    log_writeln(F("Running waypoint sequence."));
    config_waypoint_t waypoint = config_get_waypoint(0);
    int nwaypoints = waypoint.nwaypoints;

    if (nwaypoints < 100) {
        // TurnOnPID();
        for (int step = 1; step < nwaypoints; step++) {
            log_write(F("step: %d - "), step);
            waypoint = config_get_waypoint(step);
            switch (waypoint.command) {
            case COMMAND_MOVE_AT:  // Fallthrough.
            case COMMAND_MOVE_BESIDE:  // Fallthrough.
            case COMMAND_MOVE_CLOSE:  // Fallthrough.
            case COMMAND_MOVE_APPROACHING:
                move_to(waypoint);
                break;
            case COMMAND_GOTO_STEP:
                step = waypoint.step - 1;
                log_writeln(F("Goto step %d."), waypoint.step);
                break;
            case COMMAND_GOTO_STEP_IF_IO:
                // pin = waypoint.b;
                log_writeln(F("Goto step %d if button pressed."), waypoint.step);
                if (!hardware_get_button_pressed()) // TODO: Support configurable expansion IO line.
                    step = waypoint.step - 1;
                break;
            case COMMAND_INTERROGATE_SWITCHES:
                interrogate_limit_switches();
                break;
            case COMMAND_WAIT_MILLIS:
                log_writeln(F("Wait %d milliseconds."), waypoint.wait_millis);
                delay(waypoint.wait_millis);
                break;
            }
        }
        motor_disable_all();
        log_writeln(F("Done."));
    } else {
        log_writeln(F("No Waypoints."));
    }
}

static void move_to(config_waypoint_t waypoint)
{
    log_write(F("Move to "));
    move_to_waypoint_angle(waypoint);
    waypoint_print(waypoint);

    switch (waypoint.command) {
    case 'A':
        do {
            delay(50);
            track_report();
        } while (!check_progress(PROGRESS_AT));
        break;
    case 'B':
        do {
            delay(50);
            track_report();
        } while (!check_progress(PROGRESS_BESIDE));
        break;
    case 'C':
        do {
            delay(50);
            track_report();
        } while (!check_progress(PROGRESS_CLOSE));
        break;
    case 'D':
        do {
            delay(50);
            track_report();
        } while (!check_progress(PROGRESS_APPROACHING));
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
    return
        (motion_status[MOTOR_ID_C] <= progress) ||
        (motion_status[MOTOR_ID_D] <= progress) ||
        (motion_status[MOTOR_ID_E] <= progress) ||
        (motion_status[MOTOR_ID_F] <= progress);
}

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

static void set_waypoint_angle(config_waypoint_t waypoint)
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

    config_set_waypoint(step, waypoint);
    waypoint_print(waypoint);

    log_writeln(F("set."));
}

static void get_waypoint_angle(void)
{
    in_buffer.setCharAt(0, ' ');
    int index = in_buffer.toInt();
    config_waypoint_t waypoint = config_get_waypoint(index);

    waypoint_print(waypoint);
}

static void move_to_a_waypoint_angle(void)
{
    in_buffer.setCharAt(0, ' ');
    int step = in_buffer.toInt();

    log_write(F("Move to "));
    config_waypoint_t waypoint = config_get_waypoint(step);

    waypoint_print(waypoint);
    move_to_waypoint_angle(waypoint);
}

static void move_to_waypoint_angle(config_waypoint_t waypoint)
{
    motor_set_target_angle(MOTOR_ID_A, waypoint.motor.a);
    motor_set_target_angle(MOTOR_ID_B, waypoint.motor.b);
    motor_set_target_angle(MOTOR_ID_C, waypoint.motor.c);
    motor_set_target_angle(MOTOR_ID_D, waypoint.motor.d);
    motor_set_target_angle(MOTOR_ID_E, waypoint.motor.e);
    motor_set_target_angle(MOTOR_ID_F, waypoint.motor.f);
}

void waypoint_print(config_waypoint_t waypoint)
{
    assert(waypoint.step != -1);

    switch (waypoint.command) {
    case COMMAND_MOVE_AT:  // Fallthrough.
    case COMMAND_MOVE_BESIDE:  // Fallthough.
    case COMMAND_MOVE_CLOSE:  // Fallthrough.
    case COMMAND_MOVE_APPROACHING:
        log_write(F("wapyoint: step:%d command:%c a:%d b:%d c:%d d:%d e:%d f:%d"),
                  waypoint.step,
                  waypoint.command,
                  waypoint.motor.a,
                  waypoint.motor.b,
                  waypoint.motor.c,
                  waypoint.motor.d,
                  waypoint.motor.e,
                  waypoint.motor.f);
        break;
    case COMMAND_GOTO_STEP:
        log_write(F("wapyoint: step:%d command:%c (goto step) destination_step: %d"),
                  waypoint.step,
                  waypoint.command,
                  waypoint.goto_step);
        break;
    case COMMAND_GOTO_STEP_IF_IO:
        log_write(F("wapyoint: step:%d command:%c (goto step if IO) destination_step: %d"),
                  waypoint.step,
                  waypoint.command,
                  waypoint.goto_step);
        break;
    case COMMAND_INTERROGATE_SWITCHES:
        log_write(F("wapyoint: step:%d command:%c (interrogate limit switches)"),
                  waypoint.step,
                  waypoint.command);
        break;
    case COMMAND_WAIT_MILLIS:
        log_write(F("wapyoint: step:%d command:%c (wait millis) %d"),
                  waypoint.step,
                  waypoint.command,
                  waypoint.wait_millis);
        break;
    default:
        assert(false);
        break;
    }
}
