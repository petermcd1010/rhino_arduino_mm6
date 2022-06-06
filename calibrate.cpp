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

static int prev_max_speed_percent = 0;
static int max_speed_percent = 0;
static int calibrate_one_min = 0;
static int calibrate_one_max = 0;
static int calibrate_one_center = 0;
static int stuck_start_encoder = 0;
static unsigned long stuck_start_ms = 0;

static void break_handler(sm_state_t *state);
static void start(sm_state_t *state);
static void calibrate_all(sm_state_t *state);
static void calibrate_one_enter(sm_state_t *state);
static void calibrate_one(sm_state_t *state);
static void calibrate_one_go_home(sm_state_t *state);
static void calibrate_one_done(sm_state_t *state);
static void stop(sm_state_t *state);

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
            log_writeln(F("Calibrating motor %c: Encoder stuck at %d for > %d ms."), 'A' + motor_id, encoder, stuck_duration_ms);
            ret = true;
        }

        *stuck_start_encoder = encoder;
        *stuck_start_ms = ms;
    }

    return ret;
}

void calibrate_run(int in_motor_ids_mask, int max_speed_pct)
{
    assert(in_motor_ids_mask >= 0);

    log_writeln(F("calibrate_run"));

    motor_ids_mask = in_motor_ids_mask;
    max_speed_percent = max_speed_pct;
    exit_to_state = sm_get_state();
    sm_state_t s = { .run = start, .break_handler = break_handler, .name = F("calibrate start"), .data = NULL };

    sm_set_next_state(s);
}

void start(sm_state_t *state)
{
    assert(state);
    for (int i = MOTOR_ID_A; i <= MOTOR_ID_LAST; i++) {
        motor_set_enabled((motor_id_t)i, true);
    }

    motor_id = MOTOR_ID_A;
    sm_state_t s = { .run = calibrate_all, .break_handler = break_handler, .name = F("calibrate all"), .data = NULL };

    sm_set_next_state(s);
}

static void calibrate_all(sm_state_t *state)
{
    assert(state);
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
    static bool prev_triggered = motor_get_switch_triggered_debounced(motor_id);

    if ((motor_state[motor_id].pid_perror != 0) && (millis() - print_ms > 1000)) {
        log_write(F("Calibrating motor %c: "), 'A' + motor_id);
        if (motor_state[motor_id].target_encoder >= INT_MAX - 1)
            log_write(F("Forward direction, "));
        else
            log_write(F("Reverse direction, "));
        log_writeln(F("encoder: %d."), motor_get_encoder(motor_id) * motor_state[motor_id].logic);
        print_ms = millis();
    }

    bool triggered = motor_get_switch_triggered_debounced(motor_id);

    if (prev_triggered != triggered) {
        log_writeln(F("Calibrating motor %c: Switch: %d (%d)."), 'A' + motor_id, !prev_triggered, triggered);
        prev_triggered = triggered;
    }
}

bool prev_switch_triggered = false;
int num_switch_falling_edges_detected = 0;
int first_switch_forward_on = 0;
int first_switch_forward_off = 0;
int first_switch_reverse_on = 0;
int first_switch_reverse_off = 0;

static void calibrate_one_enter(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));

    log_writeln(F("Calibrating motor %c."), 'A' + motor_id);
    update_status(motor_id);

    calibrate_one_min = INT_MIN;
    calibrate_one_max = INT_MAX;
    calibrate_one_center = -1;
    stuck_start_encoder = 0;
    stuck_start_ms = millis();

    motor_set_enabled(motor_id, true);
    // When the motor max speed is ~85% and it's blocked, the switch mistriggers. Reducing
    // max speed to 80% eliminates mistriggers when blocked. It's set to 75% here to provide
    // some margin.
    prev_max_speed_percent = motor_get_max_speed_percent(motor_id);
    motor_set_max_speed_percent(motor_id, max_speed_percent);
    motor_set_target_encoder(motor_id, INT_MAX);

    prev_switch_triggered = motor_get_switch_triggered(motor_id);
    num_switch_falling_edges_detected = 0;

    first_switch_forward_on = INT_MIN;  // TODO: Should probably be INT_MAX.
    first_switch_forward_off = INT_MIN;
    first_switch_reverse_on = INT_MIN;
    first_switch_reverse_off = INT_MIN;

    motor_state[motor_id].switch_forward_on = INT_MAX;
    motor_state[motor_id].switch_forward_off = INT_MAX;
    motor_state[motor_id].switch_reverse_on = INT_MAX;  // TODO: these should probably be INT_MIN.
    motor_state[motor_id].switch_reverse_off = INT_MAX;

    sm_state_t s = { .run = calibrate_one, .break_handler = break_handler, .name = F("calibrate one"), .data = NULL };

    sm_set_next_state(s);
}

static void calibrate_one(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    update_status(motor_id);

    bool calibration_failed = false;

    if ((first_switch_forward_on == INT_MIN) && (motor_state[motor_id].switch_forward_on != INT_MAX)) {
        first_switch_forward_on = motor_state[motor_id].switch_forward_on;
        log_writeln(F("first_switch_forward_on=%d"), first_switch_forward_on);
    }

    if ((first_switch_forward_off == INT_MIN) && (motor_state[motor_id].switch_forward_off != INT_MAX)) {
        first_switch_forward_off = motor_state[motor_id].switch_forward_off;
        log_writeln(F("first_switch_forward_off=%d"), first_switch_forward_off);
    }

    if ((first_switch_reverse_on == INT_MIN) && (motor_state[motor_id].switch_reverse_on != INT_MAX)) {
        first_switch_reverse_on = motor_state[motor_id].switch_reverse_on;
        log_writeln(F("first_switch_reverse_on=%d"), first_switch_reverse_on);
    }

    if ((first_switch_reverse_off == INT_MIN) && (motor_state[motor_id].switch_reverse_off != INT_MAX)) {
        first_switch_reverse_off = motor_state[motor_id].switch_reverse_off;
        log_writeln(F("first_switch_reverse_off=%d"), first_switch_reverse_off);
    }

    if ((first_switch_forward_on != INT_MIN) && (first_switch_forward_off != INT_MIN) && (first_switch_reverse_on != INT_MIN) && (first_switch_reverse_off != INT_MIN) && (calibrate_one_min != INT_MIN) && (calibrate_one_max != INT_MAX)) {
        // Divide by 4 before adding, to avoid integer rollover.
        int midpoint = first_switch_forward_on / 4 + first_switch_forward_off / 4 + first_switch_reverse_on / 4 + first_switch_reverse_off / 4;

        first_switch_forward_on -= midpoint;
        first_switch_forward_off -= midpoint;
        first_switch_reverse_on -= midpoint;
        first_switch_reverse_off -= midpoint;
        calibrate_one_min -= midpoint;
        calibrate_one_max -= midpoint;

        num_switch_falling_edges_detected = 0;
        motor_set_home_encoder(motor_id, midpoint);
        motor_set_target_encoder(motor_id, 0);
        motor_set_max_speed_percent(motor_id, 100.0f);

        log_writeln(F("Calibrating motor %c: Set home position to midpoint at %d. Min encoder at %d, max encoder at %d"), 'A' + motor_id, midpoint, calibrate_one_min, calibrate_one_max);

        stuck_start_ms = millis();
        sm_state_t s = { .run = calibrate_one_go_home, .break_handler = break_handler, .name = F("calibrate one go home"), .data = NULL };
        sm_set_next_state(s);
    }

#if 1
    bool switch_triggered = motor_get_switch_triggered(motor_id);
    if (prev_switch_triggered != switch_triggered)
        LOG_DEBUG(F("**** SWITCH %d"), switch_triggered);

    if (prev_switch_triggered && !switch_triggered) {
        num_switch_falling_edges_detected++;
        if (num_switch_falling_edges_detected == 2) {
            num_switch_falling_edges_detected = 0;
            LOG_DEBUG(F("**** FALLING EDGE: Reset 1 to %d ****"), num_switch_falling_edges_detected);
        } else {
            LOG_DEBUG(F("**** FALLING EDGE: %d ****"), num_switch_falling_edges_detected);
        }
    } else if ((num_switch_falling_edges_detected == 1) && switch_triggered) {
        log_writeln(F("Calibrating motor %c: Switch triggered after falling edge, reversing direction at encoder %d."), 'A' + motor_id, motor_get_encoder(motor_id));
        if (motor_get_target_encoder(motor_id) == INT_MAX) {
            stuck_start_ms = millis();
            calibrate_one_max = motor_get_encoder(motor_id);
            motor_set_target_encoder(motor_id, INT_MIN);
        } else if (motor_get_target_encoder(motor_id) == INT_MIN) {
            stuck_start_ms = millis();
            calibrate_one_min = motor_get_encoder(motor_id);
            motor_set_target_encoder(motor_id, INT_MAX - 1);
        } else {
            calibration_failed = true;
        }
        num_switch_falling_edges_detected = 0;
        LOG_DEBUG(F("**** FALLING EDGE 2: Reset 2 to %d ****"), num_switch_falling_edges_detected);
    }
    prev_switch_triggered = switch_triggered;
#endif

    if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_ms, 500)) {
        log_writeln(F("Calibrating motor %c: Motor stuck, reversing directions at encoder %d."), 'A' + motor_id, stuck_start_encoder);
        if (motor_get_target_encoder(motor_id) == INT_MAX) {
            stuck_start_ms = millis();
            calibrate_one_max = motor_get_encoder(motor_id);
            motor_set_target_encoder(motor_id, INT_MIN);
        } else if (motor_get_target_encoder(motor_id) == INT_MIN) {
            stuck_start_ms = millis();
            calibrate_one_min = motor_get_encoder(motor_id);
            motor_set_target_encoder(motor_id, INT_MAX - 1);
        } else {
            calibration_failed = true;
        }
        num_switch_falling_edges_detected = 0;
    }

    if (calibration_failed) {
        stuck_start_ms = millis();
        motor_set_max_speed_percent(motor_id, 100.0f);
        LOG_ERROR(F("Failed to calibrate motor %c. Could not find switch position: forward_on=%d, forward_off=%d, reverse_on=%d, reverse_off=%d)."),
                  'A' + motor_id,
                  first_switch_forward_on,
                  first_switch_forward_off,
                  first_switch_reverse_on,
                  first_switch_reverse_off);
        sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one go home"), .data = NULL };
        sm_set_next_state(s);
    }
}

static void calibrate_one_go_home(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    update_status(motor_id);

    int encoder = motor_get_encoder(motor_id);

    if (motor_get_target_encoder(motor_id) - encoder == 0) {
        if (!motor_get_switch_triggered(motor_id)) {
            log_writeln(F("Calibrating motor %c: Switch did not trigger at home position (encoder 0). Calibration for motor %c failed."), 'A' + motor_id, 'A' + motor_id);
            sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
            sm_set_next_state(s);
        } else {
            log_writeln(F("Calibrating motor %c: Motor arrived at home position (encoder 0). Calibration for motor %c passed."), 'A' + motor_id, 'A' + motor_id);
            sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
            sm_set_next_state(s);
        }
    } else if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_ms, 5 * 1000)) {
        log_writeln(F("Calibrating motor %c: Motor is stuck at encoder %d. Calibration for motor %c failed."), 'A' + motor_id, encoder, 'A' + motor_id);
        sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
        sm_set_next_state(s);
    }
}

static void calibrate_one_done(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    update_status(motor_id);

    // Mark as failed if:
    //  - Switch not detected
    //  - Switch unreliable (more than 4 triggers?).
    //  - Thermal overload detected.

    motor_set_max_speed_percent(motor_id, prev_max_speed_percent);
    motor_set_enabled(motor_id, false);
    motor_id = (motor_id_t)((int)motor_id + 1);
    sm_state_t s = { .run = calibrate_all, .break_handler = break_handler, .name = F("calibrate all"), .data = NULL };

    sm_set_next_state(s);
}

static void break_handler(sm_state_t *state)
{
    assert(state);
    log_writeln(F("Break detected. Stopping calibration."));
    motor_disable_all();
    sm_state_t s = { .run = stop, .break_handler = NULL, .name = F("calibrate stop"), .data = NULL };

    sm_set_next_state(s);
}

static void stop(sm_state_t *state)
{
    assert(state);
    motor_disable_all();
    log_writeln(F("Calibration complete."));

    motor_id = (motor_id_t)-1;
    sm_set_next_state(exit_to_state);
}
