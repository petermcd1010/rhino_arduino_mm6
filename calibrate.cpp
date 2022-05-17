/*
 * Implementation for calibration.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "calibrate.h"
#include "config.h"
#include "log.h"
#include "motor.h"
#include "sm.h"

static bool initialized = false;

static int motor_ids_mask = 0;

typedef enum {
    CAL_STATE_INIT = 0,
    CAL_STATE_SEARCH,
    CAL_STATE_SWITCH_FORWARD_ON,
    CAL_STATE_SWITCH_FORWARD_OFF,
    CAL_STATE_SWITCH_REVERSE_ON,
    CAL_STATE_SWITCH_REVERSE_OFF,
    CAL_STATE_DONE,
} cal_state_t;

static const char *cal_state_name_by_index[] = {
    "CAL_STATE_INIT",
    "CAL_STATE_SEARCH",
    "CAL_STATE_SWITCH_FORWARD_ON",
    "CAL_STATE_SWITCH_FORWARD_OFF",
    "CAL_STATE_SWITCH_REVERSE_ON",
    "CAL_STATE_SWITCH_REVERSE_OFF",
    "CAL_STATE_DONE",
};

typedef enum {
    CAL_ERROR_NONE = 0,
    CAL_ERROR_NOT_CONFIGURED,
    CAL_ERROR_UNEXPECTED_MIN_ENCODER,
    CAL_ERROR_UNEXPECTED_MAX_ENCODER,
    CAL_ERROR_MOTOR_ERROR,
} cal_error_t;

typedef struct {
    bool        initialized;
    int         motor_ids_mask;
    motor_id_t  motor_id;
    cal_state_t state;
    cal_error_t error;
    int         delta;
    int         target;
    int         stuck_check_start_encoder;
    int         stuck_check_start_ms;
    bool        found_min_encoder;
    bool        found_max_encoder;
    int         min_encoder;
    int         max_encoder;
    bool        switch_triggered;
    int         switch_forward_on_encoder;
    int         switch_forward_off_encoder;
    int         switch_reverse_on_encoder;
    int         switch_reverse_off_encoder;
} cal_data_t;

cal_data_t cal_data = { 0 };

static void select_next_motor(void);
static void calibrate_current_motor(void);

void calibrate_init(int motor_ids_mask)
{
    memset(&cal_data, 0, sizeof(cal_data));

    log_writeln(F("mask = %08x"), motor_ids_mask);
    cal_data.motor_ids_mask = motor_ids_mask;
    cal_data.initialized = true;
}

void calibrate_begin(void)
{
    assert(cal_data.initialized);

    log_writeln(F("Calibration beginning."));

    sm_set_next_state(select_next_motor, NULL);
}

static void calibrate_end(void)
{
    memset(&cal_data, 0, sizeof(cal_data));
    sm_set_next_state(sm_motors_off_enter, NULL);

    log_writeln(F("Calibration ended."));
}

static void select_next_motor(void)
{
    do {
        if ((cal_data.motor_ids_mask & (1 << cal_data.motor_id)) != 0) {
            sm_set_next_state(calibrate_current_motor, NULL);
            break;
        } else {
            log_writeln(F("Not calibrating motor %c."), 'A' + cal_data.motor_id);
            cal_data.motor_id = (int)cal_data.motor_id + 1;
        }

        if (cal_data.motor_id >= MOTOR_ID_COUNT) {
            sm_set_next_state(calibrate_end, NULL);
            break;
        }
    } while (1);
}

static void calibrate_current_motor(void)
{
    log_writeln(F("Calibrating motor %c."), 'A' + cal_data.motor_id);
    cal_data.motor_id = (int)cal_data.motor_id + 1;
    sm_set_next_state(select_next_motor, NULL);
}

static bool is_stuck(cal_data_t *pcal_data)
{
    bool ret = false;

    const int stuck_check_interval_ms = 500;
    const int stuck_check_encoder_count = 5;

    int encoder = 0;  // noinit_data.encoder[pcal_data->motor_id];
    int ms = millis();

    if ((ms - pcal_data->stuck_check_start_ms >= stuck_check_interval_ms)) {
        // log_writeln(F("ms:%d, encoder:%d, stuck_check_start_encoder:%d"), ms, encoder, pcal_data->stuck_check_start_encoder);
        if (abs(encoder - pcal_data->stuck_check_start_encoder) <= stuck_check_encoder_count) {
            LOG_DEBUG(F("stuck"));
            ret = true;
        }

        pcal_data->stuck_check_start_encoder = encoder;
        pcal_data->stuck_check_start_ms = ms;
    }

    return ret;
}

static void calculate_mean_and_variance(float value, int nvalues, float *pM2, float *pmean, float *pvariance)
{
    // Welford's algorithm adapted from:
    // https://stackoverflow.com/questions/17052395/calculate-the-running-standard-deviation/17053010
    float delta = value - *pmean;

    *pmean += delta / nvalues;
    *pM2 += delta * (value - *pmean);
    *pvariance = *pM2 / nvalues;
}

// Track per-motor mean and variance for current
// Track per-motor mean and variance for qe-transitions/second

/*
 * CAL_LIMIT_FORWARD
 * CAL_LIMIT_BACKWARD
 * CAL_SWITCH_FORWARD
 * CAL_SWITCH_BACKWARD
 * CAL_DONE
 */
static cal_state_t get_cal_state(cal_data_t *pcal_data, bool forward, bool switch_on, bool stuck)
{
    return CAL_STATE_INIT;
}

static bool calibrate_found_encoder_extents(cal_data_t *pcal_data)
{
    assert(pcal_data);
    return pcal_data->found_min_encoder && pcal_data->found_max_encoder;
}

static bool calibrate(cal_data_t *pcal_data)
{
    assert(pcal_data);
    // Returns true while calibration is running, false when complete.

    /*
     * Strategy:
     *   cal_state_t cal_state;
     *   int negative_limit = current_encoder + 5000;  # 5000 or some number that we never expect to see more/less of.
     *   int positive_limit = current_encoder - 5000;
     *   int negative_stop = MIN_INT;
     *   int positive_stop = MAX_INT;
     *
     *   CAL_STATE_INIT rotating in the direction that takes the motor toward 0, setting max/min encoder limits.
     *     If the encoder exceeds the negative/positive limits, go to CAL_STATE_ERROR.
     *     SWITCH_ON triggered -> go to CAL_STATE_SWITCH_ON
     *     SWITCH_OFF triggered -> go to CAL_STATE_SWITCH_OFF
     *     If the encoder exceeds the negative/positive stop, reverse direction
     *     If is_stuck()
     *       update encoder negative or positive stop.
     *       reverse direction.
     *   CAL_STATE_SWITCH_ON:
     *     If the encoder exceeds the min/max limits, go to CAL_STATE_ERROR.
     *     SWITCH_OFF triggered ->
     *       note encoder value if this is >= the second transition to off in this direction
     *       If both the min and max encoder have encoder values
     *         set the zero point to the center of the two encoder values.
     *         set sensible encoder negative/positive limits
     *         go to CAL_STATE_SEARCH_STOP
     *       else
     *         set encoder value go to reverse direction and then go to CAL_STATE_SWITCH_OFF
     *     is_stuck -> go to CAL_STATE_ERROR
     *   CAL_STATE_SWITCH_OFF:
     *     If the encoder exceeds the min/max limits, go to CAL_STATE_ERROR.
     *     SWITCH_ON triggered -> go to CAL_STATE_SWITCH_ON
     *     is_stuck -> go to CAL_STATE_ERROR
     *   CAL_STATE_SEARCH_STOP:
     *     If the encoder exceeds the min/max limits, go to CAL_STATE_ERROR.
     *     If SWITCH_ON triggered, and not "close to" where expected, goto CAL_STATE_ERROR
     *     IF SWITCH_OFF triggered, and not "close to" where expected, goto CAL_STATE_ERROR
     *     If is_stuck -> mark stop, if other stop not marked, reverse direction, other wise go to CAL_STATE_SUCCESS
     *   CAL_STATE_SUCCESS:
     *   CAL_STATE_ERROR:
     */


    static cal_state_t prev_state = CAL_STATE_INIT;

    if (prev_state != pcal_data->state) {
        log_writeln(F("%s"), cal_state_name_by_index[pcal_data->state]);
        prev_state = pcal_data->state;
    }

    motor_id_t motor_id = pcal_data->motor_id;

    assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
    if (!config.motor[motor_id].configured) {
        pcal_data->error = CAL_ERROR_NOT_CONFIGURED;
        return false;
    }

    if (motor_state[motor_id].error_flags != 0) {
        LOG_ERROR(F("Motor %c signaled error during calibration"), motor_id);
        motor_log_errors(motor_id);
        pcal_data->error = CAL_ERROR_MOTOR_ERROR;
        return false;
    }

    int encoder = 0;  // noinit_data.encoder[motor_id];

    switch (pcal_data->state) {
    case CAL_STATE_INIT:
        pcal_data->state = CAL_STATE_SEARCH;
        pcal_data->delta = +10000;
        pcal_data->found_min_encoder = false;
        pcal_data->found_max_encoder = false;
        pcal_data->min_encoder = 0;
        pcal_data->max_encoder = 0;
        pcal_data->switch_triggered = false;

        pcal_data->stuck_check_start_encoder = 0;  // noinit_data.encoder[pcal_data->motor_id];
        pcal_data->stuck_check_start_ms = millis();
        pcal_data->target = encoder + pcal_data->delta;
        motor_set_target_encoder(motor_id, pcal_data->target);
        break;
    case CAL_STATE_SEARCH:
        if (encoder < pcal_data->min_encoder) {
            if (pcal_data->found_min_encoder) {
                LOG_ERROR(F("Unexpected minimum encoder %d lower than previous minimum encoder %d"), encoder, pcal_data->min_encoder);
                pcal_data->error = CAL_ERROR_UNEXPECTED_MIN_ENCODER;
            }
            pcal_data->min_encoder = encoder;
        }

        if (encoder > pcal_data->max_encoder) {
            if (pcal_data->found_max_encoder) {
                LOG_ERROR(F("Unexpected maximum encoder %d higher than previous maximum encoder %d"), encoder, pcal_data->max_encoder);
                pcal_data->error = CAL_ERROR_UNEXPECTED_MAX_ENCODER;
            }
            pcal_data->max_encoder = encoder;
        }

#if 0
        if (is_stuck(pcal_data)) {
            if (pcal_data->delta > 0) {
                log_writeln(F("Motor %c found maximum encoder %d"), 'A' + pcal_data->motor_id, encoder);
                pcal_data->found_max_encoder = true;
            } else {
                log_writeln(F("Motor %c found minimum encoder %d"), 'A' + pcal_data->motor_id, encoder);
                pcal_data->found_min_encoder = true;
            }
        }
#endif

        if (pcal_data->switch_triggered != motor_state[motor_id].switch_previously_triggered) {
            pcal_data->state = get_cal_state(pcal_data, pcal_data->delta > 0, pcal_data->switch_triggered, false);
            pcal_data->switch_triggered = motor_state[motor_id].switch_previously_triggered;

            log_writeln(F("Motor %c switch %d at encoder %d"), 'A' + motor_id, pcal_data->switch_triggered, encoder);
#if FALSE
            if (pcal_data->delta > 0) {
                if (pcal_data->switch_triggered)
                    pcal_data->state = CAL_STATE_SWITCH_FORWARD_ON;
                else
                    pcal_data->state = CAL_STATE_SWITCH_FORWARD_OFF;
            } else {
                if (pcal_data->switch_triggered)
                    pcal_data->state = CAL_STATE_SWITCH_REVERSE_ON;
                else
                    pcal_data->state = CAL_STATE_SWITCH_REVERSE_OFF;
            }
#endif
        }

        if ((pcal_data->delta < 0) &&
            ((encoder <= pcal_data->target) ||
             pcal_data->found_min_encoder)) {
            // Reached encoder target in negative direction.
            if (!pcal_data->found_max_encoder) {
                pcal_data->delta *= -1;
                pcal_data->target = pcal_data->max_encoder + pcal_data->delta;
            } else {
                pcal_data->target = encoder + pcal_data->delta;
            }
            LOG_DEBUG(F("target: %d"), pcal_data->target);
            motor_set_target_encoder(motor_id, pcal_data->target);
        } else if ((pcal_data->delta > 0) &&
                   ((encoder >= pcal_data->target) ||
                    pcal_data->found_max_encoder)) {
            // Reached encoder target in positive direction.
            if (!pcal_data->found_min_encoder) {
                pcal_data->delta = -pcal_data->delta;
                pcal_data->target = pcal_data->min_encoder + pcal_data->delta;
            } else {
                pcal_data->target = pcal_data->target + pcal_data->delta;
            }
            LOG_DEBUG(F("target: %d"), pcal_data->target);
            motor_set_target_encoder(motor_id, pcal_data->target);
        }

        if (calibrate_found_encoder_extents(pcal_data))
            pcal_data->state = CAL_STATE_DONE;

        break;
    case CAL_STATE_SWITCH_FORWARD_ON:
        // Move forward at full speed until off.
        return false;
        break;
    case CAL_STATE_SWITCH_FORWARD_OFF:
        // Forward a bit, then reverse.
        return false;
        break;
    case CAL_STATE_SWITCH_REVERSE_ON:
        // Reverse until off.
        return false;
        break;
    case CAL_STATE_SWITCH_REVERSE_OFF:
        return false;
        break;
    case CAL_STATE_DONE:
        log_writeln(F("Motor %c calibration complete."), 'A' + pcal_data->motor_id);
        return false;
        break;
    default:
        assert(false);
        return true;
    }

    return true;

    // if (motor_id == MOTOR_ID_A)
    //  return motor_interrogate_limit_switch_a();

    bool encoder_min_found = false;
    bool encoder_max_found = false;
    int encoder_min = 0;  // noinit_data.encoder[motor_id];
    int encoder_max = encoder_min;
    const int delta = 100;
    int encoder_target = 0;  // noinit_data.encoder[motor_id] + delta;

    int motor_current_draw = 0;
    int motor_current_draw_nvalues = 0;
    float motor_current_draw_M2 = 0;
    float motor_current_draw_mean = 0;
    float motor_current_draw_variance = 0;

#if 0
    float encoders_per_second_M2 = 0;
    float encoders_per_second_mean = 0;
    float encoders_per_second_variance = 0;
#endif

    int start_time_millis = millis();
    int start_encoder = 0;  // noinit_data.encoder[motor_id];

    bool switch_triggered = motor_state[motor_id].switch_previously_triggered;

    log_writeln(F("Motor %c encoder start:%d, switch_triggered=%d"),
                'A' + motor_id, 0);  // noinit_data.encoder[motor_id], switch_triggered);

#if 0
    // Find the switch and limits.
    do {
        int encoder = noinit_data.encoder[motor_id];

        encoder_min = encoder < encoder_min ? encoder: encoder_min;
        encoder_max = encoder > encoder_max ? encoder : encoder_max;

        if ((encoder_delta < 0) && ((encoder <= encoder_target) || encoder_min_found)) {
            // Reached encoder target in negative direction.
            if (!encoder_max_found) {
                encoder_delta = -encoder_delta;
                encoder_target = encoder_max + encoder_delta;
            } else {
                encoder_target = encoder + encoder_delta;
            }
        } else if ((encoder_delta > 0) && ((encoder >= encoder_target) || encoder_max_found)) {
            // Reached encoder target in positive direction.
            if (!encoder_min_found) {
                encoder_delta = -encoder_delta;
                encoder_target = encoder_min + encoder_delta;
            } else {
                encoder_target = encoder + encoder_delta;
            }
        }

        int ms = millis();
        if (ms - start_time_millis > 1000) {
            if (abs(encoder - start_encoder) < 5) {
                /*
                 * if (encoder_delta < 0) {
                 * encoder_min_found = true;
                 * log_writeln(F("Motor %c minimum found at position %d"), 'A' + motor_id, encoder);
                 * } else if (encoder_delta > 0) {
                 * encoder_max_found = true;
                 * log_writeln(F("Motor %c maximum found at position %d"), 'A' + motor_id, encoder);
                 * }
                 */
            }

            start_encoder = encoder;
            start_time_millis = ms;
        }
        // TODO: Deal with apparent PWM overflow when calibrating motor B.
        // TODO: Find switch centers, other switch values.
        // TODO: Zero motor angles to switch centers.
        // TODO: Make robust for A-motor when fixing fasterner is loose.
        // TODO: Fail calibration if switch not found.
        // TODO: Consider checkpointing?

        if (switch_triggered != motor_state[motor_id].switch_previously_triggered) {
            switch_triggered = motor_state[motor_id].switch_previously_triggered;
            log_writeln(F("Motor %c switch %d at encoder %d"), 'A' + motor_id, switch_triggered, encoder);
        }

#if 0
        if (motor_state[motor_id].current_draw != motor_current_draw)
            motor_current_draw = motor_state[motor_id].current_draw;
        // log_writeln(F("current: %d"), motor_current_draw);

        int motor_current_draw = motor_get_current_draw(motor_id);  // motor_state[motor_id].current;
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

#endif

        motor_set_target_encoder(motor_id, encoder_target);
    } while (!(encoder_min_found && encoder_max_found));  // (!motor_state[motor_id].switch_previously_triggered);

    log_writeln(F("Motor %c encoder_min=%d, encoder_max=%d"), 'A' + motor_id, encoder_min, encoder_max);
#endif

    return false;  // Calibration complete.

#if 0

    int r = 0;
    int f = 0;

    if (motor_state[motor_id].switch_previously_triggered) {
        //Serial.print(" Centering Motor ");
        //Serial.println(char(m+65));
        //Serial.print("   Moving to: ");
        noinit_data.encoder[motor_id] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
        motor_state[motor_id].target_encoder = 0;

        // Move to one side of switch and wait for the switch to be unpressed.
        log_write(F("  "));
        motor_set_target_encoder(motor_id, r - 130);
        do {
            motor_track_report(motor_id);
        } while (motor_state[motor_id].switch_previously_triggered);

        // Move to the other side of switch and wait for the switch to be pressed and then unpressed.
        log_write(F("  "));
        motor_set_target_encoder(motor_id, f + 130);
        do {
            motor_track_report(motor_id);
        } while (!motor_state[motor_id].switch_previously_triggered);

        do {
            motor_track_report(motor_id);
        } while (motor_state[motor_id].switch_previously_triggered);

        //do {motor_track_report(m);} while (motor_get_encoder(m) < f);

        // Move back to first side of switch and wait for the switch to be pressed and then unpressed.
        log_write(F("  "));
        motor_set_target_encoder(motor_id, r - 130);
        do {
            motor_track_report(motor_id);
        } while (!motor_state[motor_id].switch_previously_triggered);

        do {
            motor_track_report(motor_id);
        } while (motor_state[motor_id].switch_previously_triggered);
        //do {motor_track_report(m);} while (motor_get_encoder(m) > r);

        // Calculate center of switches and then move to that place.
        int center_encoder =
            ((motor_state[motor_id].switch_forward_off + motor_state[motor_id].switch_reverse_on +
              motor_state[motor_id].switch_forward_on + motor_state[motor_id].switch_reverse_off) / 4);
        log_writeln(F("   Switch positions: %d, %d, %d, %d. Centering at position %d"),
                    motor_state[motor_id].switch_reverse_off,
                    motor_state[motor_id].switch_forward_on,
                    motor_state[motor_id].switch_reverse_on,
                    motor_state[motor_id].switch_forward_off,
                    center_encoder);
        motor_set_target_encoder(motor_id, center_encoder);
        do {
            track_report(motor_id);
        } while (noinit_data.encoder[motor_id] != center_encoder);

        // Set Encoder and Target Values to 0.
        noinit_data.encoder[motor_id] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
        motor_state[motor_id].target_encoder = 0;
    } else {
        log_writeln(F(" Motor %c home switch not closed - skipping."), 'A' + motor_id);
    }
#endif
}

bool motor_calibrate(motor_id_t motor_id)
{
    assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

    if (!config.motor[motor_id].configured) {
        log_writeln(F("ERROR: Motor %c not configured. Calibration failed."), 'A' + motor_id);
        return false;
    }

    cal_data_t cal_data = {};

    cal_data.motor_id = motor_id;
    log_writeln(F("Calibrating motor %c ..."), 'A' + motor_id);
    while (calibrate(&cal_data)) {
    }
    ;
    if (cal_data.error != CAL_ERROR_NONE) {
        log_writeln(F("calibration of motor %c failed with error %d."), 'A' + motor_id, cal_data.error);
        return false;
    }

    log_writeln(F("calibration of motor %c passed."), 'A' + motor_id);
    return true;

#if 0
    if (motor_id != -1)
        log_writeln(F("Calibrating motor %c ... %s"), 'A' + motor_id, motor_calibrate(motor_id) ? "passed" : "FAILED");
    else
        log_writeln(F("Calibrating all motors ... %s"), motor_calibrate_all() ? "passed" : "FAILED");
#endif
}

bool motor_calibrate_all(void)
{
    bool ret = true;

    for (int i = MOTOR_ID_A; i <= MOTOR_ID_A; i++) {
        if (!config.motor[i].configured) {
            log_writeln(F("ERROR: Motor %c not configured. Skipping calibration."), 'A' + i);
            continue;
        }

        if (!motor_calibrate(i))
            ret = false;
    }
    return ret;
}
