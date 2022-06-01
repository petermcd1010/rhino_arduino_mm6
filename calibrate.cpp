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
static int calibrate_one_switch_min = 0;
static int calibrate_one_switch_max = 0;
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

static bool is_stuck(motor_id_t motor_it, int *stuck_start_encoder, unsigned long *stuck_start_ms)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    assert(stuck_start_encoder);
    assert(stuck_start_ms);

    bool ret = false;

    const int stuck_check_interval_ms = 500;
    const int stuck_check_encoder_count = 5;

    int encoder = motor_get_encoder(motor_id);
    unsigned long ms = millis();

    if (((ms - *stuck_start_ms) >= stuck_check_interval_ms)) {
        // log_writeln(F("ms:%lu, encoder:%d, stuck_check_start_encoder:%d"), ms, encoder, stuck_start_encoder);
        if (abs(encoder - *stuck_start_encoder) <= stuck_check_encoder_count) {
            LOG_DEBUG(F("stuck, encoder=%d, stuck_start_encoder=%d"), encoder, *stuck_start_encoder);
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

static void calibrate_one_enter(void)
{
    log_writeln(F("Calibrating motor %c."), 'A' + motor_id);

    calibrate_one_min = -1;
    calibrate_one_max = -1;
    calibrate_one_center = -1;
    calibrate_one_switch_min = INT_MAX;
    calibrate_one_switch_max = INT_MIN;
    stuck_start_encoder = 0;
    stuck_start_ms = millis();

    motor_set_enabled(motor_id, true);
    // When the motor max speed is ~85% and it's blocked, the switch mistriggers. Reducing
    // max speed to 80% eliminates mistriggers when blocked. It's set to 75% here to provide
    // some margin.
    motor_set_max_speed_percent(motor_id, 50.0f);
    motor_set_target_encoder(motor_id, INT_MAX);

    sm_state_t s = { .run = calibrate_one, .break_handler = break_handler, .name = F("calibrate one"), .data = NULL };

    sm_set_next_state(s);
}

static void calibrate_one(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));

    static unsigned long print_ms = 0;

    if ((motor_state[motor_id].pid_perror != 0) && (millis() - print_ms > 1000)) {
        LOG_DEBUG(F("target_encoder=%d, encoder=%d, pid_error=%d"), motor_state[motor_id].target_encoder, motor_get_encoder(motor_id) * motor_state[motor_id].logic, motor_state[motor_id].pid_perror);
        print_ms = millis();
    }

    if (motor_get_switch_triggered(motor_id)) {
        int encoder = motor_get_encoder(motor_id);
        int target_encoder = motor_get_target_encoder(motor_id);

        if (encoder < calibrate_one_switch_min) {
            log_writeln(F("**** calibrate_one_switch_min = %d"), encoder);
            calibrate_one_switch_min = encoder;
        }

        if (encoder > calibrate_one_switch_max) {
            log_writeln(F("**** calibrate_one_switch_max = %d"), encoder);
            calibrate_one_switch_max = encoder;
        } else {
            // log_writeln(F("**** calibrate_one_switch_max = %d <? %d"), encoder, calibrate_one_switch_max);
        }
    }

    if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_ms)) {
        if (motor_get_target_encoder(motor_id) == INT_MAX) {
            log_writeln(F("calibrate_one changing directions at %d"), stuck_start_encoder);
            stuck_start_ms = millis();
            calibrate_one_max = motor_get_encoder(motor_id);
            motor_set_target_encoder(motor_id, INT_MIN);
        } else {
            log_writeln(F("calibrate_one done at %d"), stuck_start_encoder);
            calibrate_one_center = (calibrate_one_max + stuck_start_encoder) / 2;
            // motor_set_target_encoder(motor_id, 0);

            log_writeln(F("calibrate_one switch min at %d, switch max at %d"), calibrate_one_switch_min, calibrate_one_switch_max);

            int midpoint = (calibrate_one_switch_min / 2) + (calibrate_one_switch_max / 2);  // Distribute division over terms to avoid overflow.
            motor_set_home_encoder(motor_id, midpoint);
            motor_set_target_encoder(motor_id, 0);

            stuck_start_ms = millis();
            motor_set_max_speed_percent(motor_id, 100.0f);
            sm_state_t s = { .run = calibrate_one_go_home, .break_handler = break_handler, .name = F("calibrate one go home"), .data = NULL };
            sm_set_next_state(s);
        }
    }
}

static void calibrate_one_go_home(void)
{
    if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_ms)) {
        log_writeln(F("calibrate_one_go_home stuck at %d. Calibration for motor %d failed."), stuck_start_encoder, motor_id);
        sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
        sm_set_next_state(s);
    }

    int encoder = motor_get_encoder(motor_id);

    if (abs(motor_get_target_encoder(motor_id) - encoder) < 5) {
        log_writeln(F("Motor %c at %d."), 'A' + motor_id, encoder);
        sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
        sm_set_next_state(s);
    }
}

static void calibrate_one_done(void)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));

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
