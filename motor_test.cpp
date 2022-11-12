/*
 * Implementation for motor testing.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <limits.h>
#include "config.h"
#include "log.h"
#include "motor.h"
#include "motor_test.h"
#include "sm.h"

static void half_wiggle(sm_state_t *state);
static void half_wiggle_wait_stop(sm_state_t *state);
static void test_one(sm_state_t *state);
static void test_one_exit(sm_state_t *state);
static void stop_moving_motor(sm_state_t *state);
static void test_mask(sm_state_t *state);
static void exit_sm(void);
static void break_handler(sm_state_t *state);

static const char state_half_wiggle_name[] PROGMEM = "half_wiggle";
static const char state_half_wiggle_wait_stop_name[] PROGMEM = "half_wiggle_wait_stop";
static const char state_motor_test_one_name[] PROGMEM = "test_one";
static const char state_motor_test_one_exit_name[] PROGMEM = "test_one_exit";
static const char state_stop_moving_motor_name[] PROGMEM = "stop_moving_motor";
static const char state_motor_test_mask_name[] PROGMEM = "test_mask";

static const sm_state_t state_half_wiggle = { .run = half_wiggle, .break_handler = break_handler, .process_break_only = true, .name = state_half_wiggle_name };
static const sm_state_t state_half_wiggle_wait_stop = { .run = half_wiggle_wait_stop, .break_handler = break_handler, .process_break_only = true, .name = state_half_wiggle_wait_stop_name };
static const sm_state_t state_test_one = { .run = test_one, .break_handler = break_handler, .process_break_only = true, .name = state_motor_test_one_name };
static const sm_state_t state_test_one_exit = { .run = test_one_exit, .break_handler = break_handler, .process_break_only = true, .name = state_motor_test_one_exit_name };
static const sm_state_t state_stop_moving_motor = { .run = stop_moving_motor, .break_handler = break_handler, .process_break_only = true, .name = state_stop_moving_motor_name };
static const sm_state_t state_test_mask = { .run = test_mask, .break_handler = break_handler, .process_break_only = true, .name = state_motor_test_mask_name };

static int exit_motor_ids_mask;
static sm_state_t exit_to_state;  // Transition to this state when done running test.
static int motor_ids_mask;
static int passing_motor_ids_mask;
static motor_id_t motor_id = (motor_id_t)-1;
static int prev_encoder;
static int forward_delta;
static int reverse_delta;
static int half_wiggle_speed;
static int half_wiggle_start_encoder;
static unsigned long half_wiggle_start_ms;

static void half_wiggle(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    assert(half_wiggle_speed != 0);

    const int desired_encoder_delta = 10;
    const int timeout_ms = 500;

    if (half_wiggle_start_ms == 0) {
        // Initialize when half_wiggle_start_ms == 0;
        half_wiggle_start_ms = millis();
        half_wiggle_start_encoder = motor_get_encoder(motor_id);
        motor_set_speed(motor_id, half_wiggle_speed);
        log_write(F("on"));
    } else if (abs(motor_get_encoder(motor_id) - half_wiggle_start_encoder) > desired_encoder_delta) {
        motor_set_speed(motor_id, 0);
        motor_set_enabled(motor_id, false);
        log_write(F(", off,"));
        sm_set_next_state(state_half_wiggle_wait_stop);
    } else if (millis() - half_wiggle_start_ms > timeout_ms) {
        // Timeout.
        motor_set_enabled(motor_id, false);
        sm_set_next_state(state_half_wiggle_wait_stop);
    }
}

static void half_wiggle_wait_stop(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    if (!motor_is_moving(motor_id))
        sm_set_next_state(state_test_one);
}

static void test_one(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    static const int speed = 200;  // 255 - motor_min_pwm.
    static bool retried_forward = false;

    motor_set_enabled(motor_id, true);

    half_wiggle_start_ms = 0;  // Set to 0 so half_wiggle initializes.

    if (half_wiggle_speed == 0) {
        half_wiggle_speed = speed;
        retried_forward = false;
        log_write(F("  %c: Forward "), 'A' + motor_id);
        sm_set_next_state(state_half_wiggle);
    } else if (half_wiggle_speed > 0) {
        forward_delta = motor_get_encoder(motor_id) - half_wiggle_start_encoder;
        log_write(F(" (%d encoders), "), forward_delta);
        half_wiggle_speed = -speed;
        log_write(F("reverse "));
        sm_set_next_state(state_half_wiggle);
    } else {
        reverse_delta = motor_get_encoder(motor_id) - half_wiggle_start_encoder;
        log_write(F(" (%d encoders)"), reverse_delta);
        half_wiggle_speed = 0;  // Reset.

        if ((forward_delta == 0) && (reverse_delta != 0) && !retried_forward) {
            // Retry if it moved one direction but not the other. This happens if the motor won't move due to a joint being structurally
            // blocked from moving.
            half_wiggle_speed = speed;
            retried_forward = true;
            log_write(F(", retry forward, "));
            sm_set_next_state(state_half_wiggle);
        } else {
            sm_set_next_state(state_test_one_exit);
        }
    }
}

static void test_one_exit(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    motor_set_home_encoder(motor_id, -prev_encoder);
    motor_set_target_encoder(motor_id, 0);
    motor_set_enabled(motor_id, false);

    const __FlashStringHelper *pfailure_message = NULL;

    if ((forward_delta == 0) && (reverse_delta == 0))
        pfailure_message = F(". FAILED to move.");
    else if (forward_delta == 0)
        pfailure_message = F(". FAILED to move forward direction.");
    else if (reverse_delta == 0)
        pfailure_message = F(". FAILED to move reverse direction.");
    else if ((forward_delta < 0) == (reverse_delta < 0))
        pfailure_message = F(". FAILED to switch direction.");  // To Test, execute pinMode(51, INPUT_PULLUP) to disable Motor F's direction pin.

    if (pfailure_message) {
        log_writeln(pfailure_message);
    } else {
        if ((reverse_delta > 0) && (forward_delta < 0)) {
            log_write(F(". (Wired backwards, reversing polarity. *PLEASE SAVE CONFIGURATION*)"));
            config_set_motor_forward_polarity(motor_id, !config.motor[motor_id].forward_polarity);
        }
        passing_motor_ids_mask |= 1 << motor_id;
        log_writeln(F(" ... passed."));
    }

    motor_id = motor_id + 1;
    sm_set_next_state(state_test_mask);
}

static void stop_moving_motor(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    motor_set_enabled(motor_id, false);
    if (!motor_is_moving(motor_id)) {
        sm_set_next_state(state_test_one);
        log_writeln(F("  %c: Stopped moving."), 'A' + motor_id);
    }
}

static void test_mask(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT + 1));
    assert((motor_ids_mask >= 0) && (motor_ids_mask <= MOTOR_IDS_MASK));

    if (motor_ids_mask & (1 << motor_id)) {
        log_writeln(F("Testing motor %c"), 'A' + motor_id);
        // Set encoder to 0 to avoid test failure when encder at integer limit.
        prev_encoder = motor_get_encoder(motor_id);
        motor_set_home_encoder(motor_id, prev_encoder);
        motor_set_target_encoder(motor_id, 0);
        motor_set_enabled(motor_id, true);

        half_wiggle_speed = 0;

        if (motor_is_moving(motor_id)) {
            sm_set_next_state(state_stop_moving_motor);
        } else {
            // Reset, since CTRL+C could have exited in the middle of state transitions.
            prev_encoder = 0;
            forward_delta = 0;
            reverse_delta = 0;
            half_wiggle_speed = 0;
            half_wiggle_start_encoder = 0;
            half_wiggle_start_ms = 0;
            sm_set_next_state(state_test_one);
        }
    } else {
        motor_id = (motor_id_t)((int)motor_id + 1);
    }

    if (motor_id >= MOTOR_ID_COUNT)
        exit_sm();
}

static void exit_sm(void)
{
    motor_id = (motor_id_t)-1;
    motor_set_enabled_mask(0);  // Stop motors that may be moving.
    motor_set_enabled_mask(exit_motor_ids_mask);  // Re-enable motors enabled at start.
    sm_set_next_state(exit_to_state);
    log_writeln(F("Motor test completed."));
}

static void break_handler(sm_state_t *state)
{
    assert(state);
    log_writeln(F("Break detected. Stopping calibration."));
    exit_sm();
}

void motor_test(motor_id_t motor_id, sm_state_t in_exit_to_state)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    passing_motor_ids_mask = 0;
    motor_test_mask(1 << motor_id, in_exit_to_state);
}

void motor_test_mask(int in_motor_ids_mask, sm_state_t in_exit_to_state)
{
    assert((in_motor_ids_mask >= 0) && (in_motor_ids_mask <= MOTOR_IDS_MASK));

    passing_motor_ids_mask = 0;
    motor_ids_mask = in_motor_ids_mask;
    motor_id = 0;
    exit_motor_ids_mask = motor_get_enabled_mask();
    motor_set_enabled_mask(0);

    exit_to_state = in_exit_to_state;

    sm_set_next_state(state_test_mask);
}

bool motor_test_passed(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    return (passing_motor_ids_mask & (1 << motor_id)) != 0;
}
