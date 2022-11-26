/*
 * Implementation for calibration.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <limits.h>
#include "calibrate.h"
#include "config.h"
#include "log.h"
#include "motor.h"
#include "motor_test.h"
#include "sm.h"

static int exit_motor_ids_mask = 0;
static sm_state_t exit_to_state = { 0 };  // Transition to this state when done running waypoints.
static bool calibrate_limits = false;
static int passing_motor_ids_mask = 0;
static int motor_ids_mask = 0;
static motor_id_t motor_id = (motor_id_t)-1;
static int prev_max_speed_percent = 0;
static int max_speed_percent = 0;
static int prev_min_encoder = 0;
static int prev_max_encoder = 0;

static bool is_gripper = false;
static int stalled_start_encoder = 0;
static unsigned long stalled_start_millis = 0;
static int home_forward_on_encoder = 0;  // Home switch transition from off to on heading in the positive direction.
static int home_forward_off_encoder = 0;
static int home_reverse_on_encoder = 0;  // Home switch transition from off to on heading in the negative direction.
static int home_reverse_off_encoder = 0;
static int ntimes_found_min_encoder = 0;  // Used to check if a home switch was never detected.
static int ntimes_found_max_encoder = 0;
static int min_encoder = 0;  // Limit found heading in the negative direction.
static int max_encoder = 0;  // Limit found heading in the positive direction.

static void calibrate_all(void);
static void test_one_enter(void);
static void calibrate_one_enter(void);
static void calibrate_one_reverse_then_forward(void);
static void calibrate_one_forward(void);
static void calibrate_one_reverse(void);
static void calibrate_one_go_home(void);
static void calibrate_one_failed(void);
static void calibrate_one_failed_goto_center(void);
static void calibrate_one_done(void);
static void exit_sm(void);
static void break_handler(void);

static const char state_calibrate_all_name[] PROGMEM = "calibrate all";
static const char state_test_one_enter_name[] PROGMEM = "test one enter";
static const char state_calibrate_one_enter_name[] PROGMEM = "calibrate one enter";
static const char state_calibrate_one_forward_name[] PROGMEM = "calibrate one forward";
static const char state_calibrate_one_reverse_then_forward_name[] PROGMEM = "calibrate one reverse then forward";
static const char state_calibrate_one_reverse_name[] PROGMEM = "calibrate one reverse";
static const char state_calibrate_one_go_home_name[] PROGMEM = "calibrate one go home";
static const char state_calibrate_one_failed_goto_center_name[] PROGMEM = "calibrate one failed goto center";
static const char state_calibrate_one_failed_name[] PROGMEM = "calibrate one failed";
static const char state_calibrate_one_done_name[] PROGMEM = "calibrate one done";

static const sm_state_t state_calibrate_all = { .run = calibrate_all, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_all_name };
static const sm_state_t state_test_one_enter = { .run = test_one_enter, .break_handler = break_handler, .process_break_only = true, .name = state_test_one_enter_name };
static const sm_state_t state_calibrate_one_enter = { .run = calibrate_one_enter, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_enter_name };
static const sm_state_t state_calibrate_one_forward = { .run = calibrate_one_forward, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_forward_name };
static const sm_state_t state_calibrate_one_reverse_then_forward = { .run = calibrate_one_reverse_then_forward, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_reverse_then_forward_name };
static const sm_state_t state_calibrate_one_reverse = { .run = calibrate_one_reverse, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_reverse_name };
static const sm_state_t state_calibrate_one_go_home = { .run = calibrate_one_go_home, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_go_home_name };
static const sm_state_t state_calibrate_one_failed_goto_center = { .run = calibrate_one_failed_goto_center, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_failed_goto_center_name };
static const sm_state_t state_calibrate_one_failed = { .run = calibrate_one_failed, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_failed_name };
static const sm_state_t state_calibrate_one_done = { .run = calibrate_one_done, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_done_name };

static void transition_to_calibrate_one_reverse_then_forward(void);
static void transition_to_calibrate_one_forward(void);
static void transition_to_calibrate_one_reverse(void);

static bool reset_stall_detection(void)
{
    stalled_start_encoder = 0;
    stalled_start_millis = millis();
}

static bool is_stalled(unsigned long stalled_duration_millis)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    static unsigned long trigger_millis = 0;
    unsigned long ms = millis();

    if (motor_stall_triggered(motor_id)) {
        motor_clear_stall(motor_id);
        const int stall_detect_millis = 250;
        if ((ms - trigger_millis) >= stall_detect_millis) {
            log_writeln(F("Calibrating motor %c: Exceeded stall current threshold %d for %dms."), 'A' + motor_id, config.motor[motor_id].stall_current_threshold, stall_detect_millis);
            trigger_millis = ms;
            return true;
        }
    }

    if (((ms - stalled_start_millis) >= stalled_duration_millis)) {
        const int stalled_check_encoder_count = 5;  // Manually determined through experimentation.
        int encoder = motor_get_encoder(motor_id);
        if (abs(encoder - stalled_start_encoder) <= stalled_check_encoder_count) {
            log_writeln(F("Calibrating motor %c: Stalled at encoder %d for > %lu ms."), 'A' + motor_id, encoder, stalled_duration_millis);
            return true;
        }
        stalled_start_encoder = encoder;
        stalled_start_millis = ms;
    }

    return false;
}

static bool is_gripper_and_get_home_center(int *gripper_home_center)
{
    assert(gripper_home_center);

    if ((min_encoder == INT_MIN) || (max_encoder == INT_MAX))
        return false;                  // Can't be a gripper if there's not a min or max encoder.

    // The gripper will have two or three (but not four) switch transitions:
    //    forward_on_encoder will have triggered.
    //    reverse_off_encoder will have triggered.
    //    At least one of reverse_on_encoder or forward_off_encoder will not have triggered.
    // ... or, exactly the opposite, depending on the arm.
    //
    // Gripper home is the midpoint between the "on encoder" and the respective min or max encoder.
    if (((home_forward_on_encoder != INT_MAX) && (home_reverse_off_encoder != INT_MIN)) && ((home_reverse_on_encoder == INT_MIN) || (home_forward_off_encoder == INT_MAX))) {
        *gripper_home_center = ((float)home_forward_on_encoder + (float)max_encoder) / 2.0;
        return true;
    } else if (((home_forward_on_encoder == INT_MAX) || (home_forward_off_encoder == INT_MIN)) && ((home_reverse_on_encoder != INT_MIN) && (home_forward_off_encoder != INT_MAX))) {
        *gripper_home_center = ((float)home_reverse_on_encoder + (float)min_encoder) / 2.0;
        return true;
    }

    return false;
}

static bool found_all_positions(void)
{
    static unsigned long ms = 0;

    if ((home_forward_on_encoder == INT_MAX) ||
        (home_forward_off_encoder == INT_MAX) ||
        (home_reverse_on_encoder == INT_MIN) ||
        (home_reverse_off_encoder == INT_MIN))
        return false;

    if (calibrate_limits &&
        ((min_encoder == INT_MIN) ||
         (max_encoder == INT_MAX)))
        return false;

    return true;
}

static bool cant_find_home_switch(void)
{
    if ((ntimes_found_min_encoder >= 2) || (ntimes_found_max_encoder >= 2))
        return true;                   // Never found a switch between the min and max encoders.
    else
        return false;
}

static void update_status(motor_id_t motor_id)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    static unsigned long prev_print_ms = 0;

    if ((motor_state[motor_id].pid_perror != 0) && (millis() - prev_print_ms > 1000)) {
        log_write(F("Calibrating motor %c: "), 'A' + motor_id);
        int encoder = motor_get_encoder(motor_id);
        int target_encoder = motor_get_target_encoder(motor_id);
        if (target_encoder == INT_MAX) {
            log_write(F("Forward direction, "));
        } else if (target_encoder == INT_MIN) {
            log_write(F("Reverse direction, "));
        } else if (target_encoder == 0) {
            if (encoder != 0)
                log_write(F("Heading home (encoder 0), currently at "));
            else
                log_write(F("At home, "));
        }

        log_writeln(F("encoder %d."), encoder * config.motor[motor_id].orientation);
        prev_print_ms = millis();
    }
}

static void calibrate_all(void)
{
    assert((motor_ids_mask >= 0) && (motor_ids_mask <= MOTOR_IDS_MASK));

    if (motor_ids_mask & (1 << motor_id))
        sm_set_next_state(state_test_one_enter);
    else
        motor_id = (motor_id_t)(motor_id + 1);

    if (motor_id >= MOTOR_ID_COUNT)
        exit_sm();
}

static void test_one_enter(void)
{
    // Transitions to motor_test state machine, and then back to this state when the motor_test has completed.

    static bool test_started = false;

    if (!test_started) {
        test_started = true;
        log_writeln(F("Calibrating motor %c: Running initial motor test and checking motor polarity."), 'A' + motor_id);
        motor_test(motor_id, sm_get_state());
    } else {
        test_started = false;
        log_write(F("Calibrating motor %c: Initial motor test "), 'A' + motor_id);
        if (!motor_test_passed(motor_id)) {
            log_writeln(F("** FAILED **. Canceling motor %c calibration."), 'A' + motor_id);
            sm_set_next_state(state_calibrate_one_done);
        } else {
            log_writeln(F("passed."));
            sm_set_next_state(state_calibrate_one_enter);
        }
    }
}

static void calibrate_one_enter(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    reset_stall_detection();
    is_gripper = false;

    prev_min_encoder = config.motor[motor_id].min_encoder;
    prev_max_encoder = config.motor[motor_id].max_encoder;
    config_set_motor_min_max_encoders(motor_id, INT_MIN, INT_MAX);

    home_forward_on_encoder = INT_MAX;
    home_forward_off_encoder = INT_MAX;
    home_reverse_on_encoder = INT_MIN;
    home_reverse_off_encoder = INT_MIN;
    ntimes_found_min_encoder = 0;
    ntimes_found_max_encoder = 0;
    min_encoder = INT_MIN;
    max_encoder = INT_MAX;

    // Enable only the motor under calbiration, because other motors may draw current if they have problems.
    motor_set_enabled_mask(1 << motor_id);
    motor_set_home_encoder(motor_id, motor_get_encoder(motor_id));

    prev_max_speed_percent = motor_get_max_speed_percent(motor_id);
    motor_set_max_speed_percent(motor_id, max_speed_percent);
    motor_clear_stall(motor_id);

    if (!calibrate_limits && motor_is_home_triggered_debounced(motor_id))
        transition_to_calibrate_one_reverse_then_forward();
    else if (motor_get_encoder(motor_id) <= 0)
        transition_to_calibrate_one_forward();
    else
        transition_to_calibrate_one_reverse();
}

static void transition_to_calibrate_one_reverse_then_forward(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    reset_stall_detection();
    motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
    motor_state[motor_id].home_reverse_off_encoder = INT_MIN;

    sm_set_next_state(state_calibrate_one_reverse_then_forward);
}

static void calibrate_one_reverse_then_forward(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    motor_set_target_encoder(motor_id, INT_MIN);  // Stall detection in motor.cpp may change target. Make sure it's INT_MIN.

    if (motor_state[motor_id].home_reverse_off_encoder != INT_MIN) {
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, backup past home switch complete."), 'A' + motor_id, motor_get_encoder(motor_id));
        home_reverse_off_encoder = motor_state[motor_id].home_reverse_off_encoder;
        transition_to_calibrate_one_forward();
    }

    if (motor_state[motor_id].home_reverse_on_encoder != INT_MIN) {
        motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, unexpected second reverse on home switch. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
    } else if (is_stalled(500)) {
        max_encoder = stalled_start_encoder;  // Set to stalled_start_encoder because it may not be able to command to present encoder when moving slowly.
        log_writeln(F("Calibrating motor %c: Forward backing up, encoder %d, motor stalled. Setting max encoder to stall start encoder %d"), 'A' + motor_id, motor_get_encoder(motor_id), max_encoder);
        reset_stall_detection();
        sm_set_next_state(state_calibrate_one_failed);
    }
}

static void transition_to_calibrate_one_forward(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    motor_state[motor_id].home_forward_on_encoder = INT_MAX;
    motor_state[motor_id].home_forward_off_encoder = INT_MAX;

    reset_stall_detection();
    sm_set_next_state(state_calibrate_one_forward);
}

static void calibrate_one_forward(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    motor_set_target_encoder(motor_id, INT_MAX);  // Stall detection in motor.cpp may change target. Make sure it's INT_MAX.
    update_status(motor_id);

    if (motor_state[motor_id].home_forward_on_encoder != INT_MAX) {
        if (home_forward_on_encoder == INT_MAX) {
            home_forward_on_encoder = motor_state[motor_id].home_forward_on_encoder;
            motor_state[motor_id].home_forward_on_encoder = INT_MAX;

            if (home_forward_off_encoder == INT_MAX) {
                log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, forward home switch on at encoder %d."), 'A' + motor_id, motor_get_encoder(motor_id), home_forward_on_encoder);
            } else {
                // Erase previously found forward_off encoder values.
                log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, forward home switch on at encoder %d, forgetting previous forward home switch off."), 'A' + motor_id, motor_get_encoder(motor_id), home_forward_on_encoder);
                home_forward_off_encoder = INT_MAX;
                motor_state[motor_id].home_forward_off_encoder = INT_MAX;
            }
        } else {
            // Second time finding forward on encoder.
            max_encoder = motor_get_encoder(motor_id);
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, found second forward home switch on at encoder %d. Setting max encoder to %d. Reversing."),
                        'A' + motor_id, motor_get_encoder(motor_id), motor_state[motor_id].home_forward_on_encoder, max_encoder);
            transition_to_calibrate_one_reverse();
        }
    }

    if (motor_state[motor_id].home_forward_off_encoder != INT_MAX) {
        if (home_forward_off_encoder == INT_MAX) {
            // First time finding forward off encoder.
            home_forward_off_encoder = motor_state[motor_id].home_forward_off_encoder;
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, forward home switch off at encoder %d."), 'A' + motor_id, motor_get_encoder(motor_id), home_forward_off_encoder);
        } else {
            // Second time finding forward off encoder.
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, unexpected second forward home switch off at encoder %d. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id), motor_state[motor_id].home_forward_off_encoder);
        }
        motor_state[motor_id].home_forward_off_encoder = INT_MAX;
    }

    if (found_all_positions()) {
        sm_set_next_state(state_calibrate_one_go_home);
    } else if (cant_find_home_switch()) {
        sm_set_next_state(state_calibrate_one_failed);
    } else if (!calibrate_limits && (home_forward_on_encoder != INT_MAX) && (home_forward_off_encoder != INT_MAX)) {
        // Not finding limits and found both the forward on and forward off encoders, so turn around.
        transition_to_calibrate_one_reverse();
    } else if (is_stalled(500)) {
        int gripper_home_center = 0;
        if (is_gripper_and_get_home_center(&gripper_home_center)) {
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, gripper found. Setting home encoder to %d."), 'A' + motor_id, motor_get_encoder(motor_id), gripper_home_center);
            is_gripper = true;
            home_forward_on_encoder = home_forward_off_encoder = home_reverse_on_encoder = home_reverse_off_encoder = gripper_home_center;
            sm_set_next_state(state_calibrate_one_go_home);
        } else {
            max_encoder = stalled_start_encoder;  // Set to stalled_start_encoder because it may not be able to command to present encoder when moving slowly.
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, motor stalled. Setting max encoder to %d. Reversing direction."), 'A' + motor_id, motor_get_encoder(motor_id), max_encoder);
            ntimes_found_max_encoder++;
            transition_to_calibrate_one_reverse();
        }
    }
}

static void transition_to_calibrate_one_reverse()
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    reset_stall_detection();
    motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
    motor_state[motor_id].home_reverse_off_encoder = INT_MIN;

    sm_set_next_state(state_calibrate_one_reverse);
}

static void calibrate_one_reverse(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));


    motor_set_target_encoder(motor_id, INT_MIN);  // Stall detection in motor.cpp may change target. Make sure it's INT_MIN.

    update_status(motor_id);

    if (motor_state[motor_id].home_reverse_on_encoder != INT_MIN) {
        if (home_reverse_on_encoder == INT_MIN) {
            home_reverse_on_encoder = motor_state[motor_id].home_reverse_on_encoder;
            motor_state[motor_id].home_reverse_on_encoder = INT_MIN;

            if (home_reverse_off_encoder == INT_MIN) {
                log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, reverse home switch on at encoder %d."), 'A' + motor_id, motor_get_encoder(motor_id), home_reverse_on_encoder);
            } else {
                // Erase previously found home_reverse_off_encoder values.
                log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, reverse home switch on at encoder %d, forgetting previous reverse home switch off."), 'A' + motor_id, motor_get_encoder(motor_id), home_reverse_on_encoder);
                home_reverse_off_encoder = INT_MIN;
                motor_state[motor_id].home_reverse_off_encoder = INT_MIN;
            }
        } else {
            // Second time finding reverse on encoder.
            min_encoder = motor_get_encoder(motor_id);
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, found second reverse home switch on at encoder %d. Setting min encoder to %d and reversing."),
                        'A' + motor_id, motor_get_encoder(motor_id), min_encoder, motor_state[motor_id].home_reverse_on_encoder, min_encoder);
            transition_to_calibrate_one_forward();
        }
    }

    if (motor_state[motor_id].home_reverse_off_encoder != INT_MIN) {
        if (home_reverse_off_encoder == INT_MIN) {
            home_reverse_off_encoder = motor_state[motor_id].home_reverse_off_encoder;
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, reverse home switch off at encoder %d."), 'A' + motor_id, motor_get_encoder(motor_id), home_reverse_off_encoder);
        } else {
            // Second time finding reverse off encoder.
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, unexpected second reverse home switch off at encoder %d. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id), motor_state[motor_id].home_reverse_off_encoder);
        }
        motor_state[motor_id].home_reverse_off_encoder = INT_MIN;
    }

    if (found_all_positions()) {
        sm_set_next_state(state_calibrate_one_go_home);
    } else if (cant_find_home_switch()) {
        sm_set_next_state(state_calibrate_one_failed);
    } else if (!calibrate_limits && (home_reverse_on_encoder != INT_MIN) && (home_reverse_off_encoder != INT_MIN)) {
        transition_to_calibrate_one_forward();
    } else if (is_stalled(500)) {
        int gripper_home_center = 0;
        if (is_gripper_and_get_home_center(&gripper_home_center)) {
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, gripper found. Setting home encoder to %d."), 'A' + motor_id, motor_get_encoder(motor_id), gripper_home_center);
            is_gripper = true;
            home_forward_on_encoder = home_forward_off_encoder = home_reverse_on_encoder = home_reverse_off_encoder = gripper_home_center;
            sm_set_next_state(state_calibrate_one_go_home);
        } else {
            min_encoder = stalled_start_encoder;  // Set to stalled_start_encoder because it may not be able to command to present encoder when moving slowly.
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, motor stalled. Setting min encoder to %d. Reversing direction."), 'A' + motor_id, motor_get_encoder(motor_id), min_encoder);
            ntimes_found_min_encoder++;
            transition_to_calibrate_one_forward();
        }
    }
}

static void calibrate_one_go_home(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));
    update_status(motor_id);

    if (motor_get_target_encoder(motor_id) != 0) {
        reset_stall_detection();

        int home_encoder = ((float)home_forward_on_encoder + (float)home_forward_off_encoder + (float)home_reverse_on_encoder + (float)home_reverse_off_encoder) / 4.0;
        log_writeln(F("Calibrating motor %c: Setting home position (encoder 0) to %d."), 'A' + motor_id, home_encoder);

        home_forward_on_encoder -= home_encoder;
        home_forward_off_encoder -= home_encoder;
        home_reverse_on_encoder -= home_encoder;
        home_reverse_off_encoder -= home_encoder;
        min_encoder -= home_encoder;
        max_encoder -= home_encoder;

        motor_set_home_encoder(motor_id, home_encoder);
        motor_set_target_encoder(motor_id, 0);
    }

    int encoder = motor_get_encoder(motor_id);
    int encoder_delta = motor_get_target_encoder(motor_id) - encoder;

    if ((encoder_delta == 0) || (motor_is_home_triggered_debounced(motor_id) && (encoder_delta < 5))) {
        if (!motor_is_home_triggered_debounced(motor_id)) {
            sm_set_next_state(state_calibrate_one_failed);
        } else {
            if (calibrate_limits) {
                log_writeln(F("Calibrating motor %c: Setting min encoder to %d and max encoder to %d."), 'A' + motor_id, min_encoder, max_encoder);
                config_set_motor_min_max_encoders(motor_id, min_encoder, max_encoder);
            }
            if (is_gripper) {
                log_writeln(F("Calibrating motor %c: Configuring motor as gripper."), 'A' + motor_id);
                int home_encoder = home_forward_on_encoder;
                // TODO add comment for difference beween A & B motor encoder readers.
                if (abs(min_encoder - home_encoder) < abs(max_encoder - home_encoder))
                    config_set_motor_gripper_open_close_encoders(motor_id, home_encoder, max_encoder);
                else
                    config_set_motor_gripper_open_close_encoders(motor_id, home_encoder, min_encoder);
            }

            log_writeln(F("Calibrating motor %c: Motor arrived at home position (encoder 0)."), 'A' + motor_id);
            log_writeln(F("Calibrating motor %c: Motor Calibration for motor %c ** PASSED **. *WRITE CONFIGURATION*"), 'A' + motor_id, 'A' + motor_id);

            passing_motor_ids_mask |= (1 << motor_id);
            sm_set_next_state(state_calibrate_one_done);
        }
    } else if (is_stalled(5 * 1000)) {
        log_writeln(F("Calibrating motor %c: Motor is stalled at encoder %d. Calibration for motor %c ** FAILED **."), 'A' + motor_id, encoder, 'A' + motor_id);
        sm_set_next_state(state_calibrate_one_failed);
    }
}

static void calibrate_one_failed(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));
    update_status(motor_id);

    reset_stall_detection();
    log_writeln(F("Calibrating motor %c: Could not find home switch: forward_on=%d, forward_off=%d, reverse_on=%d, reverse_off=%d)."),
                'A' + motor_id,
                home_forward_on_encoder,
                home_forward_off_encoder,
                home_reverse_on_encoder,
                home_reverse_off_encoder);
    log_writeln(F("Calibrating motor %c: Calibration for motor %c ** FAILED **."), 'A' + motor_id, 'A' + motor_id);

    if ((ntimes_found_min_encoder > 0) && (ntimes_found_max_encoder > 0)) {
        sm_set_next_state(state_calibrate_one_failed_goto_center);
    } else {
        motor_set_enabled(motor_id, false);
        sm_set_next_state(state_calibrate_one_done);
    }
}

static void calibrate_one_failed_goto_center(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));
    update_status(motor_id);

    int midpoint = ((float)min_encoder + (float)max_encoder) / 2.0;

    if (motor_get_target_encoder(motor_id) != midpoint) {
        log_writeln(F("Calibrating motor %c: Commanding motor to midpoint between min and max encoders (encoder %d) to mitigate failed calibration."), 'A' + motor_id, midpoint);
        reset_stall_detection();
        motor_set_target_encoder(motor_id, midpoint);
    }

    if (motor_get_encoder(motor_id) == midpoint) {
        log_writeln(F("Calibrating motor %c: Arrived at midpoint between min and max encoders (encoder %d)."), 'A' + motor_id, midpoint);
        sm_set_next_state(state_calibrate_one_done);
    } else if (is_stalled(5 * 1000)) {
        log_writeln(F("Calibrating motor %c: Stalled on way to midpoint between min and max encoders (encoder %d). ** FAILED **."), 'A' + motor_id, midpoint);
        sm_set_next_state(state_calibrate_one_done);
    }
}

static void calibrate_one_done(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    motor_set_enabled(motor_id, false);

    update_status(motor_id);

    motor_state[motor_id].home_forward_on_encoder = INT_MAX;
    motor_state[motor_id].home_forward_off_encoder = INT_MAX;
    motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
    motor_state[motor_id].home_reverse_off_encoder = INT_MIN;

    motor_set_max_speed_percent(motor_id, prev_max_speed_percent);

    motor_id = (motor_id_t)((int)motor_id + 1);

    sm_set_next_state(state_calibrate_all);
}

static void print_summary(void)
{
    if (motor_ids_mask == 0)
        return;

    log_writeln(F("Calibration summary:"));
    for (int motor_id = 0; motor_id < MOTOR_ID_COUNT; motor_id++) {
        if (motor_ids_mask & (1 << motor_id)) {
            if ((passing_motor_ids_mask & (1 << motor_id)) == 0)
                log_writeln(F("  Motor %c: ** FAILED **"), 'A' + motor_id);
            else
                config_print_one(motor_id);
        }
    }
}

static void exit_sm(void)
{
    if ((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT))
        config_set_motor_min_max_encoders(motor_id, prev_min_encoder, prev_max_encoder);

    motor_id = (motor_id_t)-1;
    motor_set_enabled_mask(0);  // Stop motors that may be moving.
    motor_set_enabled_mask(exit_motor_ids_mask);  // Re-enable motors enabled at start.
    print_summary();
    sm_set_next_state(exit_to_state);
}

static void break_handler(void)
{
    log_writeln(F("Break detected. Stopping calibration."));
    exit_sm();
}

static void prepare_to_calibrate(int in_motor_ids_mask, int in_max_speed_percent, bool in_calibrate_limits)
{
    assert((in_motor_ids_mask >= 0) && (in_motor_ids_mask <= MOTOR_IDS_MASK));

    calibrate_limits = in_calibrate_limits;
    motor_ids_mask = in_motor_ids_mask;
    passing_motor_ids_mask = 0;
    motor_id = (motor_id_t)0;
    max_speed_percent = in_max_speed_percent;
    exit_motor_ids_mask = motor_get_enabled_mask();
    exit_to_state = sm_get_state();
    sm_set_next_state(state_calibrate_all);
}

void calibrate_home_switch_and_limits(int in_motor_ids_mask, int in_max_speed_percent)
{
    prepare_to_calibrate(in_motor_ids_mask, in_max_speed_percent, true);
}

void calibrate_home_switch(int in_motor_ids_mask, int in_max_speed_percent)
{
    prepare_to_calibrate(in_motor_ids_mask, in_max_speed_percent, false);
}
