/*
 * Implementation for motor testing.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "config.h"
#include "log.h"
#include "motor.h"
#include "motor_test.h"

static void half_wiggle(motor_id_t motor_id, int speed)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    const int move_delay_ms = 25;
    const int stop_delay_ms = 75;

    motor_set_speed(motor_id, speed);
    log_write(F("on, "));
    delay(move_delay_ms);  // Short Delay to allow the motor to move.

    motor_set_speed(motor_id, 0);
    log_write(F("off. "));
    delay(stop_delay_ms);  // Short Delay to allow the motor to stop.
}

const int motor_test_speed = 200;  // 255 - motor_min_pwm.

static void wiggle(motor_id_t motor_id, int *forward_delta, int *reverse_delta)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    assert(forward_delta);
    assert(reverse_delta);

    int position1 = motor_get_encoder(motor_id);

    log_write(F("  %c: Reverse "), 'A' + motor_id);
    half_wiggle(motor_id, -motor_test_speed);
    int position2 = motor_get_encoder(motor_id);

    *reverse_delta = position2 - position1;
    log_write(F("Reverse delta: %+d. "), *reverse_delta);

    log_write(F("Forward "));
    half_wiggle(motor_id, motor_test_speed);
    int position3 = motor_get_encoder(motor_id);

    *forward_delta = position3 - position2;
    log_write(F("Forward delta: %+d"), *forward_delta);
}

bool motor_test(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    int forward_delta = 0;
    int reverse_delta = 0;

    if (motor_is_moving(motor_id)) {
        motor_set_enabled(motor_id, false);
        delay(250);  // Wait for motor to come to rest if it's moving.
    }

    // Disable all motors other than the one we are testing, as enabling other motors
    // increases probability of this test being flakey.
    int prev_motor_enabled_mask = motor_get_enabled_mask();

    motor_set_enabled_mask(1 << motor_id);

    wiggle(motor_id, &forward_delta, &reverse_delta);

    if (((forward_delta == 0) || (reverse_delta == 0)) && (forward_delta != reverse_delta)) {
        log_writeln(F(" ... FAILED"));
        log_write(F("  %c: Failed to move in "), 'A' + motor_id);
        if (forward_delta == 0) {
            log_writeln(F("forward direction. Moving reverse and retrying."));
            motor_set_speed(motor_id, motor_test_speed * -1);
        } else {
            log_writeln(F("reverse direction. Moving forward and retrying."));
            motor_set_speed(motor_id, motor_test_speed);
        }

        delay(250);

        wiggle(motor_id, &forward_delta, &reverse_delta);
    }

    motor_set_enabled(motor_id, false);
    motor_set_enabled_mask(prev_motor_enabled_mask);

    const __FlashStringHelper *pfailure_message = NULL;

    if ((forward_delta == 0) && (reverse_delta == 0))
        pfailure_message = F("Failed to move when commanded");
    else if ((forward_delta < 0) == (reverse_delta < 0))
        pfailure_message = F("Failed to switch direction when commanded");  // To Test, execute pinMode(51, INPUT_PULLUP) to disable Motor F's direction pin.

    if (pfailure_message) {
        log_write(F(" ... FAILED ("));
        log_write(pfailure_message);
        log_writeln(F(")."));
        return false;
    } else {
        if ((reverse_delta > 0) && (forward_delta < 0)) {
            log_write(F(". (Wired backwards, reversing polarity. *PLEASE SAVE CONFIGURATION*)"));
            config_set_motor_forward_polarity(motor_id, !config.motor[motor_id].forward_polarity);
        }
        log_writeln(F(" ... Passed."));
        return true;
    }
}

void motor_test_mask(int motor_ids_mask)
{
    log_writeln(F("Testing motors %d"), motor_ids_mask);

    int exit_motor_ids_mask = motor_get_enabled_mask();

    motor_set_enabled_mask(0);

    for (motor_id_t motor_id = 0; motor_id < MOTOR_ID_COUNT; motor_id = motor_id + 1) {
        if ((motor_ids_mask & (1 << motor_id)) == 0)
            continue;                  // Enable only the motor being calibrated.

        // Set encoder to 0 to avoid test failure when encder at integer limit.
        int prev_encoder = motor_get_encoder(motor_id);
        motor_set_home_encoder(motor_id, prev_encoder);
        motor_set_enabled_mask(1 << motor_id);

        motor_set_home_encoder(motor_id, motor_get_encoder(motor_id));

        delay(25);
        motor_test(motor_id);

        motor_set_home_encoder(motor_id, -prev_encoder);
        motor_set_target_encoder(motor_id, 0);
    }

    motor_set_enabled_mask(0);  // Stop motors that may be moving.
    motor_set_enabled_mask(exit_motor_ids_mask);  // Re-enable motors enabled at start.

    log_writeln(F("Done testing motors."));
}
