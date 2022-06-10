/*
 * Implementation for calibration.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <limits.h>
#include "calibrate.h"
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
static int stuck_start_encoder = 0;
static unsigned long stuck_start_millis = 0;
static int first_switch_forward_on = 0;
static int first_switch_forward_off = 0;
static int first_switch_reverse_on = 0;
static int first_switch_reverse_off = 0;

static void start(sm_state_t *state);
static void calibrate_all(sm_state_t *state);
static void calibrate_one_enter(sm_state_t *state);
static void calibrate_one_forward(sm_state_t *state);
static void calibrate_one_forward_backup(sm_state_t *state);
static void calibrate_one_reverse(sm_state_t *state);
static void calibrate_one_reverse_backup(sm_state_t *state);
static void calibrate_one_go_home(sm_state_t *state);
static void calibrate_one_failed(sm_state_t *state);
static void calibrate_one_done(sm_state_t *state);
static void break_handler(sm_state_t *state);
static void stop(sm_state_t *state);

static bool is_stuck(motor_id_t motor_it, int *stuck_start_encoder, unsigned long *stuck_start_millis, unsigned long stuck_duration_millis)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    assert(stuck_start_encoder);
    assert(stuck_start_millis);

    bool ret = false;

    const int stuck_check_encoder_count = 5;

    int encoder = motor_get_encoder(motor_id);
    unsigned long ms = millis();

    if (((ms - *stuck_start_millis) >= stuck_duration_millis)) {
        // log_writeln(F("ms:%lu, encoder:%d, stuck_check_start_encoder:%d"), ms, encoder, stuck_start_encoder);
        if (abs(encoder - *stuck_start_encoder) <= stuck_check_encoder_count) {
            log_writeln(F("Calibrating motor %c: Stuck at encoder %d for > %lu ms."), 'A' + motor_id, encoder, stuck_duration_millis);
            ret = true;
        }

        *stuck_start_encoder = encoder;
        *stuck_start_millis = ms;
    }

    return ret;
}

static bool found_home_position(void)
{
    if ((first_switch_forward_on == INT_MAX) ||
        (first_switch_forward_off == INT_MAX) ||
        (first_switch_reverse_on == INT_MIN) ||
        (first_switch_reverse_off == INT_MIN) ||
        (calibrate_one_max == INT_MAX) ||
        (calibrate_one_min == INT_MIN))
        return false;
    else
        return true;
}

static bool cant_find_home_switch(void)
{
    if ((calibrate_one_max != INT_MAX) &&
        (calibrate_one_min != INT_MIN) &&
        (first_switch_forward_on == INT_MAX) &&
        (first_switch_forward_off == INT_MAX) &&
        (first_switch_reverse_on == INT_MIN) &&
        (first_switch_reverse_off == INT_MIN))
        return true;
    else
        return false;
}

static void update_status(motor_id_t motor_id)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));

    static unsigned long print_ms = 0;
    static bool prev_triggered = motor_get_switch_triggered_debounced(motor_id);

    if ((motor_state[motor_id].pid_perror != 0) && (millis() - print_ms > 1000)) {
        log_write(F("Calibrating motor %c: "), 'A' + motor_id);
        if (motor_state[motor_id].target_encoder == INT_MAX)
            log_write(F("Forward direction, "));
        else if (motor_state[motor_id].target_encoder == INT_MIN)
            log_write(F("Reverse direction, "));
        else if (motor_state[motor_id].target_encoder == 0)
            log_write(F("Heading home, "));
        log_writeln(F("encoder %d."), motor_get_encoder(motor_id) * motor_state[motor_id].logic);
        print_ms = millis();
    }

    bool triggered = motor_get_switch_triggered_debounced(motor_id);

#if 0
    if (prev_triggered != triggered) {
        log_writeln(F("Calibrating motor %c: Switch: %d."), 'A' + motor_id, triggered);
        prev_triggered = triggered;
    }
#endif
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

static void start(sm_state_t *state)
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

static void calibrate_one_enter(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));

    log_writeln(F("Calibrating motor %c."), 'A' + motor_id);
    update_status(motor_id);

    calibrate_one_min = INT_MIN;
    calibrate_one_max = INT_MAX;
    stuck_start_encoder = 0;
    stuck_start_millis = millis();

    motor_set_enabled(motor_id, true);
    prev_max_speed_percent = motor_get_max_speed_percent(motor_id);

    first_switch_forward_on = INT_MAX;
    first_switch_forward_off = INT_MAX;
    first_switch_reverse_on = INT_MIN;
    first_switch_reverse_off = INT_MIN;
    calibrate_one_min = INT_MIN;
    calibrate_one_max = INT_MAX;

    sm_state_t s = { .run = calibrate_one_forward, .break_handler = break_handler, .name = F("calibrate one forward"), .data = NULL };

    sm_set_next_state(s);
}

static void calibrate_one_forward(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));

    update_status(motor_id);

    if (motor_get_target_encoder(motor_id) != INT_MAX) {
        stuck_start_millis = millis();
        motor_set_max_speed_percent(motor_id, max_speed_percent);
        motor_set_target_encoder(motor_id, INT_MAX);
        motor_state[motor_id].switch_forward_on = INT_MAX;
        motor_state[motor_id].switch_forward_off = INT_MAX;
        first_switch_forward_on = INT_MAX;
        first_switch_forward_off = INT_MAX;
    }

    if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_millis, 500)) {
        log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, motor stuck. Reversing direction."), 'A' + motor_id, stuck_start_encoder);
        stuck_start_millis = millis();
        calibrate_one_max = motor_get_encoder(motor_id);
        sm_state_t s = { .run = calibrate_one_reverse, .break_handler = break_handler, .name = F("calibrate one reverse"), .data = NULL };
        sm_set_next_state(s);
    }

    if (motor_state[motor_id].switch_forward_on != INT_MAX) {
        if (first_switch_forward_on == INT_MAX) {
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, forward switch on."), 'A' + motor_id, motor_get_encoder(motor_id));
            first_switch_forward_on = motor_state[motor_id].switch_forward_on;
            motor_state[motor_id].switch_forward_on = INT_MAX;
        } else {
            calibrate_one_max = motor_get_encoder(motor_id);
            sm_state_t s = { .run = calibrate_one_forward_backup, .break_handler = break_handler, .name = F("calibrate one forward backup"), .data = NULL };
            sm_set_next_state(s);
        }
    }

    if (motor_state[motor_id].switch_forward_off != INT_MAX) {
        if (first_switch_forward_off == INT_MAX) {
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, forward switch off."), 'A' + motor_id, motor_get_encoder(motor_id));
            if (first_switch_forward_on != INT_MAX)
                first_switch_forward_off = motor_state[motor_id].switch_forward_off;
            motor_state[motor_id].switch_forward_off = INT_MAX;
        } else {
            motor_state[motor_id].switch_forward_off = INT_MAX;
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, unexpected second forward switch off. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
        }
    }

    if (found_home_position()) {
        sm_state_t s = { .run = calibrate_one_go_home, .break_handler = break_handler, .name = F("calibrate one go home"), .data = NULL };
        sm_set_next_state(s);
    } else if (cant_find_home_switch()) {
        sm_state_t s = { .run = calibrate_one_failed, .break_handler = break_handler, .name = F("calibrate one failed"), .data = NULL };
        sm_set_next_state(s);
    }
}

static void calibrate_one_forward_backup(sm_state_t *state)
{
    if (motor_get_target_encoder(motor_id) != INT_MIN) {
        stuck_start_millis = millis();
        motor_set_max_speed_percent(motor_id, max_speed_percent);
        motor_set_target_encoder(motor_id, INT_MIN);
        motor_state[motor_id].switch_reverse_on = INT_MIN;
        motor_state[motor_id].switch_reverse_off = INT_MIN;
        first_switch_reverse_on = INT_MIN;
        first_switch_reverse_off = INT_MIN;
    }

    if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_millis, 500)) {
        log_writeln(F("Calibrating motor %c: Forward backing up, encoder %d, motor stuck."), 'A' + motor_id, stuck_start_encoder);
        stuck_start_millis = millis();
        calibrate_one_max = motor_get_encoder(motor_id);
        sm_state_t s = { .run = calibrate_one_failed, .break_handler = break_handler, .name = F("calibrate one failed"), .data = NULL };
        sm_set_next_state(s);
    }

    if (motor_state[motor_id].switch_reverse_off != INT_MIN) {
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, backup past switch complete."), 'A' + motor_id, motor_get_encoder(motor_id));
        motor_state[motor_id].switch_reverse_on = INT_MIN;
        motor_state[motor_id].switch_reverse_off = INT_MIN;
        first_switch_reverse_on = INT_MIN;
        first_switch_reverse_off = INT_MIN;
        sm_state_t s = { .run = calibrate_one_reverse, .break_handler = break_handler, .name = F("calibrate one reverse"), .data = NULL };
        sm_set_next_state(s);
    }

    if (motor_state[motor_id].switch_reverse_on != INT_MIN) {
        motor_state[motor_id].switch_reverse_on = INT_MIN;
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, unexpected second reverse on switch. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
    }
}

static void calibrate_one_reverse(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));

    update_status(motor_id);

    if (motor_get_target_encoder(motor_id) != INT_MIN) {
        stuck_start_millis = millis();
        motor_set_max_speed_percent(motor_id, max_speed_percent);
        motor_set_target_encoder(motor_id, INT_MIN);
        motor_state[motor_id].switch_reverse_on = INT_MIN;
        motor_state[motor_id].switch_reverse_off = INT_MIN;
        first_switch_reverse_on = INT_MIN;
        first_switch_reverse_off = INT_MIN;
    }

    if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_millis, 500)) {
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, motor stuck. Reversing direction."), 'A' + motor_id, stuck_start_encoder);
        stuck_start_millis = millis();
        calibrate_one_min = motor_get_encoder(motor_id);
        motor_set_target_encoder(motor_id, INT_MAX);
        sm_state_t s = { .run = calibrate_one_forward, .break_handler = break_handler, .name = F("calibrate one forward"), .data = NULL };
        sm_set_next_state(s);
    }

    if (motor_state[motor_id].switch_reverse_on != INT_MIN) {
        if (first_switch_reverse_on == INT_MIN) {
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, reverse switch on."), 'A' + motor_id, motor_get_encoder(motor_id));
            first_switch_reverse_on = motor_state[motor_id].switch_reverse_on;
            motor_state[motor_id].switch_reverse_on = INT_MIN;
        } else {
            calibrate_one_min = motor_get_encoder(motor_id);
            sm_state_t s = { .run = calibrate_one_reverse_backup, .break_handler = break_handler, .name = F("calibrate one forward"), .data = NULL };
            sm_set_next_state(s);
        }
    }

    if (motor_state[motor_id].switch_reverse_off != INT_MIN) {
        if (first_switch_reverse_off == INT_MIN) {
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, reverse switch off."), 'A' + motor_id, motor_get_encoder(motor_id));
            first_switch_reverse_off = motor_state[motor_id].switch_reverse_off;
            motor_state[motor_id].switch_reverse_off = INT_MIN;
        } else {
            motor_state[motor_id].switch_reverse_off = INT_MIN;
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, unexpected second reverse switch off. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
        }
    }

    if (found_home_position()) {
        sm_state_t s = { .run = calibrate_one_go_home, .break_handler = break_handler, .name = F("calibrate one go home"), .data = NULL };
        sm_set_next_state(s);
    } else if (cant_find_home_switch()) {
        sm_state_t s = { .run = calibrate_one_failed, .break_handler = break_handler, .name = F("calibrate one failed"), .data = NULL };
        sm_set_next_state(s);
    }
}

static void calibrate_one_reverse_backup(sm_state_t *state)
{
    if (motor_get_target_encoder(motor_id) != INT_MAX) {
        stuck_start_millis = millis();
        motor_set_max_speed_percent(motor_id, max_speed_percent);
        motor_set_target_encoder(motor_id, INT_MAX);
        motor_state[motor_id].switch_forward_on = INT_MAX;
        motor_state[motor_id].switch_forward_off = INT_MAX;
    }

    if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_millis, 500)) {
        log_writeln(F("Calibrating motor %c: Reverse backing up, encoder %d, motor stuck."), 'A' + motor_id, stuck_start_encoder);
        stuck_start_millis = millis();
        calibrate_one_max = motor_get_encoder(motor_id);
        sm_state_t s = { .run = calibrate_one_failed, .break_handler = break_handler, .name = F("calibrate one failed"), .data = NULL };
        sm_set_next_state(s);
    }

    if (motor_state[motor_id].switch_forward_off != INT_MAX) {
        log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, backup past switch complete."), 'A' + motor_id, motor_get_encoder(motor_id));
        motor_state[motor_id].switch_forward_on = INT_MAX;
        motor_state[motor_id].switch_forward_off = INT_MAX;
        sm_state_t s = { .run = calibrate_one_forward, .break_handler = break_handler, .name = F("calibrate one forward"), .data = NULL };
        sm_set_next_state(s);
    }

    if (motor_state[motor_id].switch_forward_on != INT_MAX) {
        motor_state[motor_id].switch_forward_on = INT_MAX;
        log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, unexpected second forward on switch. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
    }
}

static void calibrate_one_go_home(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    update_status(motor_id);

    if (motor_get_target_encoder(motor_id) != 0) {
        stuck_start_millis = millis();

        // Divide by 4 before adding, to avoid integer rollover.
        int midpoint = first_switch_forward_on / 4 + first_switch_forward_off / 4 + first_switch_reverse_on / 4 + first_switch_reverse_off / 4;
        log_writeln(F("Calibrating motor %c: Setting home position (encoder 0) to %d."), 'A' + motor_id, midpoint);

        first_switch_forward_on -= midpoint;
        first_switch_forward_off -= midpoint;
        first_switch_reverse_on -= midpoint;
        first_switch_reverse_off -= midpoint;
        calibrate_one_min -= midpoint;
        calibrate_one_max -= midpoint;

        motor_set_home_encoder(motor_id, midpoint);
        motor_set_target_encoder(motor_id, 0);
        motor_set_max_speed_percent(motor_id, 100.0f);
    }

    int encoder = motor_get_encoder(motor_id);

    if (is_stuck(motor_id, &stuck_start_encoder, &stuck_start_millis, 5 * 1000)) {
        log_writeln(F("Calibrating motor %c: Motor is stuck at encoder %d. Calibration for motor %c failed."), 'A' + motor_id, encoder, 'A' + motor_id);
        sm_state_t s = { .run = calibrate_one_failed, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
        sm_set_next_state(s);
    }

    if (motor_get_target_encoder(motor_id) - encoder == 0) {
        if (!motor_get_switch_triggered_debounced(motor_id)) {
            log_writeln(F("Calibrating motor %c: Switch did not trigger at home position (encoder 0). Calibration for motor %c failed."), 'A' + motor_id, 'A' + motor_id);
            sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
            sm_set_next_state(s);
        } else {
            log_writeln(F("Calibrating motor %c: Motor arrived at home position (encoder 0). Calibration for motor %c passed."), 'A' + motor_id, 'A' + motor_id);
            sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };
            sm_set_next_state(s);
        }
    }
}

static void calibrate_one_failed(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    update_status(motor_id);

    stuck_start_millis = millis();
    motor_set_max_speed_percent(motor_id, 100.0f);
    log_debug(F("Failed to calibrate motor %c. Could not find home switch: forward_on=%d, forward_off=%d, reverse_on=%d, reverse_off=%d)."),
              'A' + motor_id,
              first_switch_forward_on,
              first_switch_forward_off,
              first_switch_reverse_on,
              first_switch_reverse_off);

    sm_state_t s = { .run = calibrate_one_done, .break_handler = break_handler, .name = F("calibrate one done"), .data = NULL };

    sm_set_next_state(s);
}

static void calibrate_one_done(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id <= MOTOR_ID_LAST));
    update_status(motor_id);

    motor_state[motor_id].switch_forward_on = INT_MAX;
    motor_state[motor_id].switch_forward_off = INT_MAX;
    motor_state[motor_id].switch_reverse_on = INT_MIN;
    motor_state[motor_id].switch_reverse_off = INT_MIN;

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
