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

static int exit_motor_ids_mask = 0;
static sm_state_t exit_to_state = { 0 };  // Transition to this state when done running waypoints.
static bool calibrate_limits = false;
static int motor_ids_mask = 0;
static motor_id_t motor_id = (motor_id_t)-1;
static int prev_max_speed_percent = 0;
static int max_speed_percent = 0;
static int stalled_start_encoder = 0;
static unsigned long stalled_start_millis = 0;

static int home_forward_on_encoder = 0;
static int home_forward_off_encoder = 0;
static int home_reverse_on_encoder = 0;
static int home_reverse_off_encoder = 0;
static int ntimes_found_min_encoder = 0;
static int ntimes_found_max_encoder = 0;
static int min_encoder = 0;
static int max_encoder = 0;

static void calibrate_all(sm_state_t *state);
static void calibrate_one_enter(sm_state_t *state);
static void calibrate_one_reverse_then_forward(sm_state_t *state);
static void calibrate_one_forward(sm_state_t *state);
static void calibrate_one_forward_backup(sm_state_t *state);
static void calibrate_one_reverse(sm_state_t *state);
static void calibrate_one_reverse_backup(sm_state_t *state);
static void calibrate_one_go_home(sm_state_t *state);
static void calibrate_one_failed(sm_state_t *state);
static void calibrate_one_failed_center(sm_state_t *state);
static void calibrate_one_done(sm_state_t *state);
static void break_handler(sm_state_t *state);

static const char state_calibrate_all_name[] PROGMEM = "calibrate all";
static const char state_calibrate_one_enter_name[] PROGMEM = "calibrate one enter";
static const char state_calibrate_one_forward_name[] PROGMEM = "calibrate one forward";
static const char state_calibrate_one_forward_backup_name[] PROGMEM = "calibrate one forward backup";
static const char state_calibrate_one_reverse_then_forward_name[] PROGMEM = "calibrate one reverse then forward";
static const char state_calibrate_one_reverse_name[] PROGMEM = "calibrate one reverse";
static const char state_calibrate_one_reverse_backup_name[] PROGMEM = "calibrate one reverse backup";
static const char state_calibrate_one_go_home_name[] PROGMEM = "calibrate one go home";
static const char state_calibrate_one_failed_name[] PROGMEM = "calibrate one failed";
static const char state_calibrate_one_failed_center_name[] PROGMEM = "calibrate one center";
static const char state_calibrate_one_done_name[] PROGMEM = "calibrate one done";

static const sm_state_t state_calibrate_all = { .run = calibrate_all, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_all_name, .data = NULL };
static const sm_state_t state_calibrate_one_enter = { .run = calibrate_one_enter, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_enter_name, .data = NULL };
static const sm_state_t state_calibrate_one_forward = { .run = calibrate_one_forward, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_forward_name, .data = NULL };
static const sm_state_t state_calibrate_one_forward_backup = { .run = calibrate_one_forward_backup, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_forward_backup_name, .data = NULL };
static const sm_state_t state_calibrate_one_reverse_then_forward = { .run = calibrate_one_reverse_then_forward, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_reverse_then_forward_name, .data = NULL };
static const sm_state_t state_calibrate_one_reverse = { .run = calibrate_one_reverse, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_reverse_name, .data = NULL };
static const sm_state_t state_calibrate_one_reverse_backup = { .run = calibrate_one_reverse_backup, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_reverse_backup_name, .data = NULL };
static const sm_state_t state_calibrate_one_go_home = { .run = calibrate_one_go_home, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_go_home_name, .data = NULL };
static const sm_state_t state_calibrate_one_failed_center = { .run = calibrate_one_failed_center, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_failed_center_name, .data = NULL };
static const sm_state_t state_calibrate_one_failed = { .run = calibrate_one_failed, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_failed_name, .data = NULL };
static const sm_state_t state_calibrate_one_done = { .run = calibrate_one_done, .break_handler = break_handler, .process_break_only = true, .name = state_calibrate_one_done_name, .data = NULL };


static bool is_stalled(motor_id_t motor_id, int *stalled_start_encoder, unsigned long *stalled_start_millis, unsigned long stalled_duration_millis)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));
    assert(stalled_start_encoder);
    assert(stalled_start_millis);

    bool ret = false;

    static unsigned long trigger_millis = 0;
    static unsigned int stall_current_sum = 0;
    static unsigned int stall_current_nmeasurements = 0;

    const int stalled_check_encoder_count = 5;

    int encoder = motor_get_encoder(motor_id);
    unsigned long ms = millis();

    if (motor_stall_triggered(motor_id)) {
        motor_clear_stall(motor_id);
        if ((ms - trigger_millis) >= 500) {
            LOG_DEBUG(F("motor %c stall current threshold %d exceeded"), 'A' + motor_id, config.motor[motor_id].stall_current_threshold);
            trigger_millis = ms;
            return true;
        }
    }

    if (((ms - *stalled_start_millis) >= stalled_duration_millis)) {
        // log_writeln(F("ms:%lu, encoder:%d, stalled_check_start_encoder:%d"), ms, encoder, stalled_start_encoder);
        if (abs(encoder - *stalled_start_encoder) <= stalled_check_encoder_count) {
            log_writeln(F("Calibrating motor %c: Stuck at encoder %d for > %lu ms."), 'A' + motor_id, encoder, stalled_duration_millis);
            log_writeln(F("Average stall current = %d (sum=%d n=%d)"), stall_current_sum / stall_current_nmeasurements, stall_current_sum, stall_current_nmeasurements);
            ret = true;
        }
        *stalled_start_encoder = encoder;
        *stalled_start_millis = ms;
        stall_current_sum = motor_get_current(motor_id);
        stall_current_nmeasurements = 1;
    } else {
        stall_current_sum += motor_get_current(motor_id);
        stall_current_nmeasurements++;
    }

    return ret;
}

static bool is_gripper(void)
{
    if ((max_encoder == INT_MAX) ||
        (min_encoder == INT_MIN))
        return false;

    // If exactly one on_encoder and exactly one off_encoder, return true;
    if (((home_forward_on_encoder == INT_MAX) != (home_reverse_on_encoder == INT_MIN)) &&
        ((home_forward_off_encoder == INT_MAX) != (home_reverse_off_encoder == INT_MIN))) {
        LOG_DEBUG(F("is_gripper: fon=%d foff=%d ron=%d roff=%d"), home_forward_on_encoder, home_forward_off_encoder, home_reverse_on_encoder, home_reverse_off_encoder);
        return true;
    }

    return false;
}

static bool found_all_positions(void)
{
    if ((home_forward_on_encoder == INT_MAX) ||
        (home_forward_off_encoder == INT_MAX) ||
        (home_reverse_on_encoder == INT_MIN) ||
        (home_reverse_off_encoder == INT_MIN))
        return false;

    if (calibrate_limits &&
        ((max_encoder == INT_MAX) ||
         (min_encoder == INT_MIN)))
        return false;

    return true;
}

static bool cant_find_home_switch(void)
{
    if ((ntimes_found_max_encoder >= 2) || (ntimes_found_min_encoder >= 2))
        return true;
    else
        return false;
}

static void exit_sm(void)
{
    motor_id = (motor_id_t)-1;
    motor_set_enabled_mask(0);  // Stop motors that may be moving.
    motor_set_enabled_mask(exit_motor_ids_mask);  // Re-enable motors enabled at start.
    sm_set_next_state(exit_to_state);
}

static void update_status(motor_id_t motor_id)
{
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    static unsigned long print_ms = 0;
    static bool prev_triggered = motor_is_home_triggered_debounced(motor_id);

    if ((motor_state[motor_id].pid_perror != 0) && (millis() - print_ms > 1000)) {
        log_write(F("Calibrating motor %c: "), 'A' + motor_id);
        int encoder = motor_get_encoder(motor_id);
        int target_encoder = motor_get_target_encoder(motor_id);
        if (target_encoder == INT_MAX) {
            log_write(F("Forward direction, "));
        } else if (target_encoder == INT_MIN) {
            log_write(F("Reverse direction, "));
        } else if (target_encoder == 0) {
            if (encoder == 0)
                log_write(F("At home, "));
            else
                log_write(F("Heading home, "));
        }
        log_writeln(F("encoder %d. current %d"), encoder * config.motor[motor_id].orientation, motor_get_current(motor_id));
        print_ms = millis();
    }

    bool triggered = motor_is_home_triggered_debounced(motor_id);

#if 0
    LOG_DEBUG(F("target_encoder=%d, encoder=%d, pid_perror=%d"), motor_state[motor_id].target_encoder, motor_get_encoder(motor_id), motor_state[motor_id].pid_perror);
    if (prev_triggered != triggered) {
        log_writeln(F("Calibrating motor %c: Home switch: %d."), 'A' + motor_id, triggered);
        prev_triggered = triggered;
    }
#endif
}



void calibrate_gripper(motor_id_t in_motor_id, int max_speed_pct)
{
    assert((in_motor_id >= 0) && (in_motor_id < MOTOR_ID_COUNT));

    motor_id = in_motor_id;
    log_writeln(F("calibrate_gripper motor %c"), 'A' + motor_id);

    calibrate_limits = true;
    motor_ids_mask = 1 << motor_id;
    motor_id = 0;
    max_speed_percent = max_speed_pct;
    exit_motor_ids_mask = motor_get_enabled_mask();
    exit_to_state = sm_get_state();

    sm_set_next_state(state_calibrate_all);
}

void calibrate_home_switch_and_limits(int in_motor_ids_mask, int max_speed_pct)
{
    assert(in_motor_ids_mask >= 0);

    log_writeln(F("calibrate_home_switch_and_limits"));

    calibrate_limits = true;
    motor_ids_mask = in_motor_ids_mask;
    motor_id = 0;
    max_speed_percent = max_speed_pct;
    exit_motor_ids_mask = motor_get_enabled_mask();
    exit_to_state = sm_get_state();

    sm_set_next_state(state_calibrate_all);
}

void calibrate_home_switch(int in_motor_ids_mask, int max_speed_pct)
{
    assert(in_motor_ids_mask >= 0);

    log_writeln(F("calibrate_home_switch"));

    calibrate_limits = false;
    motor_ids_mask = in_motor_ids_mask;
    motor_id = 0;
    max_speed_percent = max_speed_pct;
    exit_motor_ids_mask = motor_get_enabled_mask();
    exit_to_state = sm_get_state();

    sm_set_next_state(state_calibrate_all);
}

static void calibrate_all(sm_state_t *state)
{
    assert(state);
    // Sum of all powers of 2 of an n-bit number is 2^n-1.
    static int max_motor_ids_mask = (1 << MOTOR_ID_COUNT) - 1;

    assert((motor_ids_mask >= 0) && (motor_ids_mask <= max_motor_ids_mask));

    if (motor_ids_mask & (1 << motor_id))
        sm_set_next_state(state_calibrate_one_enter);
    else
        motor_id = (motor_id_t)((int)motor_id + 1);

    if (motor_id >= MOTOR_ID_COUNT)
        exit_sm();
}

static void calibrate_one_enter(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    log_writeln(F("Calibrating motor %c."), 'A' + motor_id);
    update_status(motor_id);

    min_encoder = INT_MIN;
    max_encoder = INT_MAX;
    stalled_start_encoder = 0;
    stalled_start_millis = millis();

    motor_set_enabled(motor_id, true);

    log_writeln(F("Calibrating motor %c: Running initial motor test and checking motor polarity."), 'A' + motor_id);
    if (!motor_test(motor_id)) {
        sm_set_next_state(state_calibrate_one_failed);
        return;
    }

    prev_max_speed_percent = motor_get_max_speed_percent(motor_id);
    motor_clear_stall(motor_id);

    home_forward_on_encoder = INT_MAX;
    home_forward_off_encoder = INT_MAX;
    home_reverse_on_encoder = INT_MIN;
    home_reverse_off_encoder = INT_MIN;
    ntimes_found_min_encoder = 0;
    ntimes_found_max_encoder = 0;
    min_encoder = INT_MIN;
    max_encoder = INT_MAX;

    if (!calibrate_limits && motor_is_home_triggered_debounced(motor_id))
        sm_set_next_state(state_calibrate_one_reverse_then_forward);
    else if (motor_get_encoder(motor_id) <= 0)
        sm_set_next_state(state_calibrate_one_forward);
    else
        sm_set_next_state(state_calibrate_one_reverse);
}

static void calibrate_one_reverse_then_forward(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    if (motor_get_target_encoder(motor_id) != INT_MIN) {
        stalled_start_millis = millis();
        motor_set_max_speed_percent(motor_id, max_speed_percent);
        motor_set_target_encoder(motor_id, INT_MIN);
        motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
        motor_state[motor_id].home_reverse_off_encoder = INT_MIN;
        home_reverse_on_encoder = INT_MIN;
        home_reverse_off_encoder = INT_MIN;
    }

    if (motor_state[motor_id].home_reverse_off_encoder != INT_MIN) {
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, backup past home switch complete."), 'A' + motor_id, motor_get_encoder(motor_id));
        home_reverse_off_encoder = motor_state[motor_id].home_reverse_off_encoder;
        // motor_set_target_encoder(motor_id, INT_MAX);
        sm_set_next_state(state_calibrate_one_forward);
    }

    if (motor_state[motor_id].home_reverse_on_encoder != INT_MIN) {
        motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, unexpected second reverse on home switch. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
    } else if (is_stalled(motor_id, &stalled_start_encoder, &stalled_start_millis, 500)) {
        log_writeln(F("Calibrating motor %c: Forward backing up, encoder %d, motor stalled."), 'A' + motor_id, stalled_start_encoder);
        stalled_start_millis = millis();
        max_encoder = motor_get_encoder(motor_id);
        sm_set_next_state(state_calibrate_one_failed);
    }
}

static void calibrate_one_forward(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    update_status(motor_id);

    if (motor_get_target_encoder(motor_id) != INT_MAX) {
        stalled_start_millis = millis();
        motor_set_max_speed_percent(motor_id, max_speed_percent);
        motor_set_target_encoder(motor_id, INT_MAX);
        motor_state[motor_id].home_forward_on_encoder = INT_MAX;
        motor_state[motor_id].home_forward_off_encoder = INT_MAX;
        home_forward_on_encoder = INT_MAX;
        home_forward_off_encoder = INT_MAX;
    }

    if (motor_state[motor_id].home_forward_on_encoder != INT_MAX) {
        if (home_forward_on_encoder == INT_MAX) {
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, forward home switch on."), 'A' + motor_id, motor_get_encoder(motor_id));
            home_forward_on_encoder = motor_state[motor_id].home_forward_on_encoder;
            motor_state[motor_id].home_forward_on_encoder = INT_MAX;
        } else {
            max_encoder = motor_get_encoder(motor_id);
            sm_set_next_state(state_calibrate_one_forward_backup);
        }
    }

    if (motor_state[motor_id].home_forward_off_encoder != INT_MAX) {
        if (home_forward_off_encoder == INT_MAX) {
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, forward home switch off."), 'A' + motor_id, motor_get_encoder(motor_id));
            if (home_forward_on_encoder != INT_MAX) // TODO: needed?
                home_forward_off_encoder = motor_state[motor_id].home_forward_off_encoder;
            motor_state[motor_id].home_forward_off_encoder = INT_MAX;
        } else {
            motor_state[motor_id].home_forward_off_encoder = INT_MAX;
            log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, unexpected second forward home switch off. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
        }
    }

    if (found_all_positions()) {
        sm_set_next_state(state_calibrate_one_go_home);
    } else if (is_gripper()) {
        LOG_DEBUG(F("is gripper"));
        if (home_forward_on_encoder == INT_MAX)
            home_forward_on_encoder = home_reverse_on_encoder;
        else
            home_reverse_on_encoder = home_forward_on_encoder;

        if (home_forward_off_encoder == INT_MAX)
            home_forward_off_encoder = home_reverse_off_encoder;
        else
            home_reverse_off_encoder = home_forward_off_encoder;
        sm_set_next_state(state_calibrate_one_go_home);
    } else if (cant_find_home_switch()) {
        sm_set_next_state(state_calibrate_one_failed);
    } else if (!calibrate_limits && (home_forward_on_encoder != INT_MAX) && (home_forward_off_encoder != INT_MAX)) {
        sm_set_next_state(state_calibrate_one_reverse);
    } else if (is_stalled(motor_id, &stalled_start_encoder, &stalled_start_millis, 500)) {
        log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, motor stalled. Reversing direction."), 'A' + motor_id, stalled_start_encoder);
        stalled_start_millis = millis();
        ntimes_found_max_encoder++;
        max_encoder = motor_get_encoder(motor_id);
        sm_set_next_state(state_calibrate_one_reverse);
    }
}

static void calibrate_one_forward_backup(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    if (motor_get_target_encoder(motor_id) != INT_MIN) {
        stalled_start_millis = millis();
        motor_set_max_speed_percent(motor_id, max_speed_percent);
        motor_set_target_encoder(motor_id, INT_MIN);
        motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
        motor_state[motor_id].home_reverse_off_encoder = INT_MIN;
        home_reverse_on_encoder = INT_MIN;
        home_reverse_off_encoder = INT_MIN;
    }

    if (motor_state[motor_id].home_reverse_off_encoder != INT_MIN) {
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, backup past home switch complete."), 'A' + motor_id, motor_get_encoder(motor_id));
        motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
        motor_state[motor_id].home_reverse_off_encoder = INT_MIN;
        home_reverse_on_encoder = INT_MIN;
        home_reverse_off_encoder = INT_MIN;
        sm_set_next_state(state_calibrate_one_reverse);
    }

    if (motor_state[motor_id].home_reverse_on_encoder != INT_MIN) {
        motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, unexpected second reverse on home switch. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
    } else if (is_stalled(motor_id, &stalled_start_encoder, &stalled_start_millis, 500)) {
        log_writeln(F("Calibrating motor %c: Forward backing up, encoder %d, motor stalled."), 'A' + motor_id, stalled_start_encoder);
        stalled_start_millis = millis();
        max_encoder = motor_get_encoder(motor_id);
        sm_set_next_state(state_calibrate_one_failed);
    }
}

static void calibrate_one_reverse(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    update_status(motor_id);

    if (motor_get_target_encoder(motor_id) != INT_MIN) {
        stalled_start_millis = millis();
        motor_set_max_speed_percent(motor_id, max_speed_percent);
        motor_set_target_encoder(motor_id, INT_MIN);
        motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
        motor_state[motor_id].home_reverse_off_encoder = INT_MIN;
        home_reverse_on_encoder = INT_MIN;
        home_reverse_off_encoder = INT_MIN;
    }

    if (motor_state[motor_id].home_reverse_on_encoder != INT_MIN) {
        if (home_reverse_on_encoder == INT_MIN) {
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, reverse home switch on."), 'A' + motor_id, motor_get_encoder(motor_id));
            home_reverse_on_encoder = motor_state[motor_id].home_reverse_on_encoder;
            motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
        } else {
            min_encoder = motor_get_encoder(motor_id);
            sm_set_next_state(state_calibrate_one_reverse_backup);
        }
    }

    if (motor_state[motor_id].home_reverse_off_encoder != INT_MIN) {
        if (home_reverse_off_encoder == INT_MIN) {
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, reverse home switch off."), 'A' + motor_id, motor_get_encoder(motor_id));
            if (home_reverse_on_encoder != INT_MAX) // TODO: needed?
                home_reverse_off_encoder = motor_state[motor_id].home_reverse_off_encoder;
            motor_state[motor_id].home_reverse_off_encoder = INT_MIN;
        } else {
            motor_state[motor_id].home_reverse_off_encoder = INT_MIN;
            log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, unexpected second reverse home switch off. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
        }
    }

    if (found_all_positions()) {
        sm_set_next_state(state_calibrate_one_go_home);
    } else if (is_gripper()) {
        LOG_DEBUG(F("is gripper"));
        if (home_forward_on_encoder == INT_MAX)
            home_forward_on_encoder = home_reverse_on_encoder;
        else
            home_reverse_on_encoder = home_forward_on_encoder;

        if (home_forward_off_encoder == INT_MAX)
            home_forward_off_encoder = home_reverse_off_encoder;
        else
            home_reverse_off_encoder = home_forward_off_encoder;
        sm_set_next_state(state_calibrate_one_go_home);
    } else if (cant_find_home_switch()) {
        sm_set_next_state(state_calibrate_one_failed);
    } else if (!calibrate_limits && (home_reverse_on_encoder != INT_MIN) && (home_reverse_off_encoder != INT_MIN)) {
        sm_set_next_state(state_calibrate_one_forward);
    } else if (is_stalled(motor_id, &stalled_start_encoder, &stalled_start_millis, 500)) {
        log_writeln(F("Calibrating motor %c: Reverse direction, encoder %d, motor stalled. Reversing direction."), 'A' + motor_id, stalled_start_encoder);
        stalled_start_millis = millis();
        min_encoder = motor_get_encoder(motor_id);
        ntimes_found_min_encoder++;
        motor_set_target_encoder(motor_id, INT_MAX);
        sm_set_next_state(state_calibrate_one_forward);
    }
}

static void calibrate_one_reverse_backup(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));

    if (motor_get_target_encoder(motor_id) != INT_MAX) {
        stalled_start_millis = millis();
        motor_set_max_speed_percent(motor_id, max_speed_percent);
        motor_set_target_encoder(motor_id, INT_MAX);
        motor_state[motor_id].home_forward_on_encoder = INT_MAX;
        motor_state[motor_id].home_forward_off_encoder = INT_MAX;
    }

    if (motor_state[motor_id].home_forward_off_encoder != INT_MAX) {
        log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, backup past home switch complete."), 'A' + motor_id, motor_get_encoder(motor_id));
        motor_state[motor_id].home_forward_on_encoder = INT_MAX;
        motor_state[motor_id].home_forward_off_encoder = INT_MAX;
        sm_set_next_state(state_calibrate_one_forward);
    } else if (motor_state[motor_id].home_forward_on_encoder != INT_MAX) {
        motor_state[motor_id].home_forward_on_encoder = INT_MAX;
        log_writeln(F("Calibrating motor %c: Forward direction, encoder %d, unexpected second forward on home switch. Ignoring."), 'A' + motor_id, motor_get_encoder(motor_id));
    } else if (is_stalled(motor_id, &stalled_start_encoder, &stalled_start_millis, 500)) {
        log_writeln(F("Calibrating motor %c: Reverse backing up, encoder %d, motor stalled."), 'A' + motor_id, stalled_start_encoder);
        stalled_start_millis = millis();
        max_encoder = motor_get_encoder(motor_id);
        sm_set_next_state(state_calibrate_one_failed);
    }
}

static void calibrate_one_go_home(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));
    update_status(motor_id);

    if (motor_get_target_encoder(motor_id) != 0) {
        stalled_start_millis = millis();

        // Divide by 4 before adding, to avoid integer rollover.
        int midpoint = home_forward_on_encoder / 4 + home_forward_off_encoder / 4 + home_reverse_on_encoder / 4 + home_reverse_off_encoder / 4;
        log_writeln(F("Calibrating motor %c: Setting home position (encoder 0) to %d."), 'A' + motor_id, midpoint);

        home_forward_on_encoder -= midpoint;
        home_forward_off_encoder -= midpoint;
        home_reverse_on_encoder -= midpoint;
        home_reverse_off_encoder -= midpoint;
        min_encoder -= midpoint;
        max_encoder -= midpoint;

        motor_set_home_encoder(motor_id, midpoint);
        motor_set_target_encoder(motor_id, 0);
        motor_set_max_speed_percent(motor_id, max_speed_percent);
    }

    int encoder = motor_get_encoder(motor_id);

    if (motor_get_target_encoder(motor_id) - encoder == 0) {
        if (!motor_is_home_triggered_debounced(motor_id)) {
            sm_set_next_state(state_calibrate_one_failed);
        } else {
            log_writeln(F("Calibrating motor %c: Motor arrived at home position (encoder 0). Calibration for motor %c passed."), 'A' + motor_id, 'A' + motor_id);

            if (calibrate_limits)
                config_set_motor_min_max_encoders(motor_id, min_encoder, max_encoder);
            config_set_motor_home_encoders(motor_id, home_forward_on_encoder, home_forward_off_encoder, home_reverse_on_encoder, home_reverse_off_encoder);

            sm_set_next_state(state_calibrate_one_done);
        }
    } else if (is_stalled(motor_id, &stalled_start_encoder, &stalled_start_millis, 5 * 1000)) {
        log_writeln(F("Calibrating motor %c: Motor is stalled at encoder %d. Calibration for motor %c failed."), 'A' + motor_id, encoder, 'A' + motor_id);
        sm_set_next_state(state_calibrate_one_failed);
    }
}

static void calibrate_one_failed(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));
    update_status(motor_id);

    stalled_start_millis = millis();
    log_writeln(F("Failed to calibrate motor %c. Could not find home switch: forward_on=%d, forward_off=%d, reverse_on=%d, reverse_off=%d)."),
                'A' + motor_id,
                home_forward_on_encoder,
                home_forward_off_encoder,
                home_reverse_on_encoder,
                home_reverse_off_encoder);

    if ((ntimes_found_min_encoder > 0) && (ntimes_found_max_encoder > 0)) {
        sm_set_next_state(state_calibrate_one_failed_center);
    } else {
        motor_set_enabled(motor_id, false);
        sm_set_next_state(state_calibrate_one_done);
    }
}

static void calibrate_one_failed_center(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));
    update_status(motor_id);

    int midpoint = min_encoder / 2 + max_encoder / 2;

    if (motor_get_target_encoder(motor_id) != midpoint) {
        stalled_start_millis = millis();
        motor_set_target_encoder(motor_id, midpoint);
    }

    if (motor_get_encoder(motor_id) == midpoint) {
        log_writeln(F("Calibrating motor %c: Motor arrived at midpoint %d between min and max encoders."), 'A' + motor_id, midpoint);
        sm_set_next_state(state_calibrate_one_done);
    } else if (is_stalled(motor_id, &stalled_start_encoder, &stalled_start_millis, 5 * 1000)) {
        log_writeln(F("Calibrating motor %c: Motor stalled attempting to move to midpoint %d between min and max encoders. Centering for motor %c failed."), 'A' + motor_id, midpoint, 'A' + motor_id);
        sm_set_next_state(state_calibrate_one_done);
    }
}

static void calibrate_one_done(sm_state_t *state)
{
    assert(state);
    assert((motor_id >= MOTOR_ID_A) && (motor_id < MOTOR_ID_COUNT));
    update_status(motor_id);

    motor_state[motor_id].home_forward_on_encoder = INT_MAX;
    motor_state[motor_id].home_forward_off_encoder = INT_MAX;
    motor_state[motor_id].home_reverse_on_encoder = INT_MIN;
    motor_state[motor_id].home_reverse_off_encoder = INT_MIN;

    motor_set_max_speed_percent(motor_id, prev_max_speed_percent);
    motor_set_enabled(motor_id, false);
    motor_id = (motor_id_t)((int)motor_id + 1);

    sm_set_next_state(state_calibrate_all);
}

static void break_handler(sm_state_t *state)
{
    assert(state);
    log_writeln(F("Break detected. Stopping calibration."));
    exit_sm();
}
