/*
 * Implementation for calibration.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <limits.h>
#include "calibrate.h"
#include "config.h"
#include "log.h"
#include "motor.h"
#include "sm.h"

static sm_state_t exit_to_state = { 0 };  // Transition to this state when done running waypoints.
static int motor_ids_mask = 0;
static motor_id_t motor_id = (motor_id_t)-1;

static int calibrate_one_min = 0;
static int calibrate_one_max = 0;
static int calibrate_one_center = 0;
static int stuck_start_encoder = 0;
static unsigned long stuck_start_ms = 0;

static void break_handler(void);
static void start(void);
static void calibrate_all(void);
static void calibrate_one_enter(void);
static void calibrate_one(void);
static void calibrate_one_go_home(void);
static void calibrate_one_done(void);
static void stop(void);

#if 0
static void calculate_mean_and_variance(float value, int nvalues, float *pM2, float *pmean, float *pvariance)
{
    // Welford's algorithm adapted from:
    // https://stackoverflow.com/questions/17052395/calculate-the-running-standard-deviation/17053010
    float delta = value - *pmean;

    *pmean += delta / nvalues;
    *pM2 += delta * (value - *pmean);
    *pvariance = *pM2 / nvalues;
}

static void iterate();
{
    int motor_current_draw = 0;
    int motor_current_draw_nvalues = 0;
    float motor_current_draw_M2 = 0;
    float motor_current_draw_mean = 0;
    float motor_current_draw_variance = 0;

    float encoders_per_second_M2 = 0;
    float encoders_per_second_mean = 0;
    float encoders_per_second_variance = 0;
    if (motor_state[motor_id].current_draw != motor_current_draw)
        motor_current_draw = motor_state[motor_id].current_draw;
    // log_writeln(F("current: %d"), motor_current_draw);

    int motor_current_draw = motor_get_current_draw(motor_id);      // motor_state[motor_id].current;
    calculate_mean_and_variance(motor_current_draw, ++motor_current_draw_nvalues,
                                &motor_current_draw_M2, &motor_current_draw_mean, &motor_current_draw_variance);

    char motor_current_draw_mean_str[15] = {};
    dtostrf(motor_current_draw_mean, 3, 2, motor_current_draw_mean_str);
    char motor_current_draw_variance_str[15] = {};
    dtostrf(motor_current_draw_variance, 3, 2, motor_current_draw_variance_str);
    // log_writeln(F("current:%d, mean:%s, variance:%s"), motor_current_draw, motor_current_draw_mean_str, motor_current_draw_variance_str);

    int mcmul = (int)(motor_current_draw_mean * 2.5);
    if ((motor_current_draw_nvalues > 100) && (motor_current_draw > mcmul))
        log_writeln(F("high motor_current mean = %s, (%d > %d)"), motor_current_draw_mean_str,
                    motor_current_draw, mcmul);

    motor_set_target_encoder(motor_id, encoder_target);

    int center_encoder =
        ((motor_state[motor_id].switch_forward_off + motor_state[motor_id].switch_reverse_on +
          motor_state[motor_id].switch_forward_on + motor_state[motor_id].switch_reverse_off) / 4);

    log_writeln(F("   Switch positions: %d, %d, %d, %d. Centering at position %d"),
                motor_state[motor_id].switch_reverse_off,
                motor_state[motor_id].switch_forward_on,
                motor_state[motor_id].switch_reverse_on,
                motor_state[motor_id].switch_forward_off,
                center_encoder);
}
#endif

static bool is_stuck(motor_id_t motor_it, int *stuck_start_encoder, unsigned long *stuck_start_ms, unsigned long stuck_duration_ms)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    assert(stuck_start_encoder);
    assert(stuck_start_ms);

    bool ret = false;

    const int stuck_check_encoder_count = 5;

    int encoder = motor_get_encoder(motor_id);
    unsigned long ms = millis();

    if (((ms - *stuck_start_ms) >= stuck_duration_ms)) {
        // log_writeln(F("ms:%lu, encoder:%d, stuck_check_start_encoder:%d"), ms, encoder, stuck_start_encoder);
        if (abs(encoder - *stuck_start_encoder) <= stuck_check_encoder_count) {
            log_writeln(F("Calibrating motor %c: Stuck at encoder %d for > %d ms."), 'A' + motor_id, encoder, stuck_duration_ms);
            ret = true;
        }

        *stuck_start_encoder = encoder;
        *stuck_start_ms = ms;
    }

    return ret;
}

void calibrate_run(int in_motor_ids_mask)
{
    assert(in_motor_ids_mask >= 0);

    log_writeln(F("calibrate_run"));

    motor_ids_mask = in_motor_ids_mask;
    exit_to_state = sm_get_state();
    sm_state_t s = { .run = start, .break_handler = break_handler, .name = F("calibrate start"), .data = NULL };

    sm_set_next_state(s);
}

void start(void)
{
    for (int i = MOTOR_ID_A; i <= MOTOR_ID_LAST; i++) {
        motor_set_enabled((motor_id_t)i, true);
    }

    motor_id = MOTOR_ID_A;
    sm_state_t s = { .run = calibrate_all, .break_handler = break_handler, .name = F("calibrate all"), .data = NULL };

    sm_set_next_state(s);
}

static void calibrate_all(void)
{
    // Sum of all powers of 2 of an n-bit number is 2^n-1.
    static int max_motor_ids_mask = (1 << MOTOR_ID_COUNT) - 1;

    assert((motor_ids_mask >= 0) && (motor_ids_mask <= max_motor_ids_mask));

    log_writeln(F("calibrate_all"));

    if (motor_ids_mask & (1 << motor_id)) {
        sm_state_t s = { .run = calibrate_one_enter, .break_handler = break_handler, .name = F("calibrate one enter"), .data = NULL };
        sm_set_next_state(s);
    } else {
        motor_id = (motor_id_t)((int)motor_id + 1);
    }

    if (motor_id > MOTOR_ID_LAST) {
        sm_state_t s = { .run = stop, .break_handler = NULL, .name = F("calibrate stop"), .data = NULL };
        sm_set_next_state(s);
    }
}

static void update_status(motor_id_t motor_id)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));

    static unsigned long print_ms = 0;
    static bool prev_triggered = !motor_state[motor_id].switch_triggered;

    if ((motor_state[motor_id].pid_perror != 0) && (millis() - print_ms > 1000)) {
        log_write(F("Calibrating motor %c: "), 'A' + motor_id);
        if (motor_state[motor_id].target_encoder >= INT_MAX - 1)
            log_write(F("Target: +inf, "));
        else
            log_write(F("Target: -inf, "));
        log_writeln(F("encoder: %d."), motor_get_encoder(motor_id) * motor_state[motor_id].logic);
        print_ms = millis();
    }

    if (prev_triggered != motor_state[motor_id].switch_triggered) {
        // Use !prev_triggered below, so we can better detect transients.
        log_writeln(F("Calibrating motor %c: Switch: %d (%d)."), 'A' + motor_id, !prev_triggered, motor_state[motor_id].switch_triggered);
        prev_triggered = !prev_triggered;
    }
}

static void calibrate_one_enter(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));

    log_writeln(F("Calibrating motor %c."), 'A' + motor_id);
    update_status(motor_id);

    calibrate_one_min = -1;
    calibrate_one_max = -1;
    calibrate_one_center = -1;
    stuck_start_encoder = 0;
    stuck_start_ms = millis();

    motor_set_enabled(motor_id, true);
    // When the motor max speed is ~85% and it's blocked, the switch mistriggers. Reducing
    // max speed to 80% eliminates mistriggers when blocked. It's set to 75% here to provide
    // some margin.
    motor_set_max_speed_percent(motor_id, 50.0f);
    motor_set_target_encoder(motor_id, INT_MAX);

    motor_state[motor_id].switch_forward_on = INT_MAX;
    motor_state[motor_id].switch_forward_off = INT_MAX;
    motor_state[motor_id].switch_reverse_on = INT_MAX;
    motor_state[motor_id].switch_reverse_off = INT_MAX;

    sm_state_t s = { .run = calibrate_one, .break_handler = break_handler, .name = F("calibrate one"), .data = NULL };

    sm_set_next_state(s);
}

static void calibrate_one(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    update_status(motor_id);

    if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_ms, 500)) {
        if (motor_get_target_encoder(motor_id) == INT_MAX) {
            log_writeln(F("Calibrating motor %c: Changing directions at %d."), 'A' + motor_id, stuck_start_encoder);
            stuck_start_ms = millis();
            calibrate_one_max = motor_get_encoder(motor_id);
            motor_set_target_encoder(motor_id, INT_MIN);
        } else if (motor_get_target_encoder(motor_id) == INT_MIN) {
            log_writeln(F("Calibrating motor %c: Changing directions at %d."), 'A' + motor_id, stuck_start_encoder);
            stuck_start_ms = millis();
            calibrate_one_max = motor_get_encoder(motor_id);
            motor_set_target_encoder(motor_id, INT_MAX - 1);
        } else {
            calibrate_one_center = (calibrate_one_max + stuck_start_encoder) / 2;

            stuck_start_ms = millis();
            motor_set_max_speed_percent(motor_id, 100.0f);

            if ((motor_state[motor_id].switch_forward_on == INT_MAX) ||
                (motor_state[motor_id].switch_forward_off == INT_MAX) ||
                (motor_state[motor_id].switch_reverse_on == INT_MAX) ||
                (motor_state[motor_id].switch_reverse_off == INT_MAX)) {
                LOG_ERROR(F("Failed to calibrate motor %c. Could not find switch position: forward_on=%d, forward_off=%d, reverse_on=%d, reverse_off=%d)."),
                          'A' + motor_id,
                          motor_state[motor_id].switch_forward_on,
                          motor_state[motor_id].switch_forward_off,
                          motor_state[motor_id].switch_reverse_on,
                          motor_state[motor_id].switch_reverse_off);
                sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one go home"), .data = NULL };
                sm_set_next_state(s);
            } else {
                // Divide by 4 before adding, to avoid integer rollover.
                int midpoint = motor_state[motor_id].switch_forward_on / 4 +
                               motor_state[motor_id].switch_forward_off / 4 +
                               motor_state[motor_id].switch_reverse_on / 4 +
                               motor_state[motor_id].switch_reverse_off / 4;

                log_writeln(F("Calibrating motor %c: Found switch midpoint at %d. Setting to home position."), 'A' + motor_id, midpoint);

                stuck_start_ms = millis();
                motor_set_home_encoder(motor_id, midpoint);
                motor_set_target_encoder(motor_id, 0);
                sm_state_t s = { .run = calibrate_one_go_home, .break_handler = break_handler, .name = F("calibrate one go home"), .data = NULL };
                sm_set_next_state(s);
            }
        }
    }
}

static void calibrate_one_go_home(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    update_status(motor_id);

    int encoder = motor_get_encoder(motor_id);

    if (motor_get_target_encoder(motor_id) - encoder == 0) {
        log_writeln(F("Calibrating motor %c: Motor is home."), 'A' + motor_id);
        sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
        sm_set_next_state(s);
    } else if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_ms, 5 * 1000)) {
        log_writeln(F("Calibrating motor %c: Motor is stuck at encoder %d. Calibration for motor %c failed."), 'A' + motor_id, encoder, 'A' + motor_id);
        sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
        sm_set_next_state(s);
    }
}

static void calibrate_one_done(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    update_status(motor_id);

    // Mark as failed if:
    //  - Switch not detected
    //  - Switch unreliable (more than 4 triggers?).
    //  - Thermal overload detected.

    motor_set_enabled(motor_id, false);
    motor_id = (motor_id_t)((int)motor_id + 1);
    sm_state_t s = { .run = calibrate_all, .break_handler = break_handler, .name = F("calibrate all"), .data = NULL };

    sm_set_next_state(s);
}

static void break_handler(void)
{
    log_writeln(F("Break detected. Stopping calibration."));
    motor_disable_all();
    sm_state_t s = { .run = stop, .break_handler = NULL, .name = F("calibrate stop"), .data = NULL };

    sm_set_next_state(s);
}

static void stop(void)
{
    motor_disable_all();
    log_writeln(F("Calibration complete."));

    motor_id = (motor_id_t)-1;
    sm_set_next_state(exit_to_state);
}
