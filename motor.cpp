/*
 * Implementation for MegaMotor6 hardware functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <limits.h>
#include <stdlib.h>
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "motor.h"

// MM6 motor I/O lines.
typedef struct {
    unsigned short out_direction;      // Digital. LOW = forward direction. HIGH = reverse direction.
    unsigned short out_pwm;            // Digital.
    unsigned short out_brake;          // Digital. LOW = disable brake. HIGH = enable brake.
    unsigned short in_current;         // Analog. 377uA/A. What's the resistance?
    unsigned short in_thermal_overload;    // Digital. Becomes active at 145C. Chip shuts off at 170C.
    unsigned short in_home_switch;     // Digital. LOW = home switch triggered. HIGH = home switch not triggered.
    unsigned short in_quadrature_encoder_a;  // Digital.
    unsigned short in_quadrature_encoder_b;  // Digital.
} motor_pinout_t;

static const motor_pinout_t motor_pinout[MOTOR_ID_COUNT] = {
    { 11, 10, 12, A5,  14,  A8, 47, 46 }, // Motor A.
    { A9, 7,  39, A0,  A11, 26, 32, 33 }, // Motor B.
    { 3,  5,  4,  A6,  6,   28, 45, 44 }, // Motor C.
    { A1, 8,  A2, A4,  A3,  30, 34, 35 }, // Motor D.
    { 17, 2,  16, A7,  15,  40, 43, 42 }, // Motor E.
    { 51, 9,  50, A10, 52,  38, 36, 37 }, // Motor F.
};

motor_state_t motor_state[MOTOR_ID_COUNT] = { 0 };  // Intentionally not static, so can be accessed globally.

// The speed sign bit is used to set the LMD18200 direction pin. So the PWM register accepts 0-255 for + and -.
static const int motor_min_speed = -255;
static const int motor_max_speed = 255;
static const int motor_min_pwm = 55;

static int enabled_motors_mask = 0;

// Configuration stored in RAM and saved across reset/reboot that don't include a power-cycle of the board.

typedef struct {
    // nbytes, version, magic are used to verify valid data.
    size_t nbytes;
    int    version;
    int    magic;
    struct {
        int previous_quadrature_encoder;
        int encoder;
    } motor[MOTOR_ID_COUNT];
} noinit_data_t;

static noinit_data_t noinit_data __attribute__((section(".noinit")));  // NOT reset to 0 when the CPU is reset.

void motor_clear_ram_data(void)
{
    // External to motor.cpp, call it ram_data instead of noinit_data.
    memset(&noinit_data, 0, sizeof(noinit_data_t));
}

static void check_noinit_data(void)
{
    // Zero out saved variables on power cycle. On reset, these values are NOT erased.
    const int noinit_data_version = 2;
    const int noinit_data_magic = 0xABCD1234;

    if ((noinit_data.nbytes != sizeof(noinit_data_t) || (noinit_data.version != noinit_data_version) || (noinit_data.magic != noinit_data_magic))) {
        log_writeln(F("Initializing noinit data."));
        noinit_data.nbytes = sizeof(noinit_data_t);
        noinit_data.version = noinit_data_version;
        noinit_data.magic = noinit_data_magic;
        for (int i = 0; i < MOTOR_ID_COUNT; i++) {
            noinit_data.motor[i].encoder = 0;
            noinit_data.motor[i].previous_quadrature_encoder = 0;
        }
    } else {
        log_writeln(F("Detected reset without RAM clear. Reusing in-RAM motor encoder values."));
        log_writeln();
    }
}

static void motor_init(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    // Configure and initialize outputs.
    pinMode(motor_pinout[motor_id].out_brake, OUTPUT);
    pinMode(motor_pinout[motor_id].out_direction, OUTPUT);
    pinMode(motor_pinout[motor_id].out_pwm, OUTPUT);
    digitalWrite(motor_pinout[motor_id].out_direction, config.motor[motor_id].forward_polarity);

    // Configure inputs.
    pinMode(motor_pinout[motor_id].in_home_switch, INPUT_PULLUP);
    pinMode(motor_pinout[motor_id].in_thermal_overload, INPUT_PULLUP);
    pinMode(motor_pinout[motor_id].in_quadrature_encoder_a, INPUT_PULLUP);
    pinMode(motor_pinout[motor_id].in_quadrature_encoder_b, INPUT_PULLUP);

    for (int motor_id = 0; motor_id < MOTOR_ID_COUNT; motor_id++) {
        memset(&motor_state[motor_id], 0, sizeof(motor_state_t));
        // Motor orientation is used to contol which way the motors turn in responce to the Positions.
        //   Each Rhino Robot may have the motor assembled on either side - which winds up reversing the motors direction mechanically.
        //   So the motor orientation is used to correct that.
        //     The values for the Motor orientation are set by the setup.
        //       Since the not-inverted and inverted orientation are used to invert the position the values are 1 or -1
        motor_set_max_speed_percent((motor_id_t)motor_id, 100);
        motor_state[motor_id].prev_home_triggered = motor_is_home_triggered((motor_id_t)motor_id);
        motor_state[motor_id].prev_home_triggered_millis = millis();
        motor_state[motor_id].prev_home_triggered_encoder = INT_MAX;
        motor_state[motor_id].home_triggered_debounced = motor_state[motor_id].prev_home_triggered;
    }
    motor_set_enabled(motor_id, false);
}

void motor_init_all(void)
{
    check_noinit_data();
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        motor_init((motor_id_t)i);
    }

    // Timer setup: Allows preceise timed measurements of the quadrature encoder.
    cli();  // Disable interrupts.

    // Configure timer1 interrupt at 1kHz.
    TCCR1A = 0;  // Set entire TCCR1A register to 0.
    TCCR1B = 0;  // Same for TCCR1B.
    TCNT1 = 0;  // Initialize counter value to 0.

    // Set timer count for 2khz increments.
    OCR1A = 1000;  // = (16*10^6) / (2000*8) - 1

    TCCR1B |= (1 << WGM12);  // Turn on CTC mode.
    TCCR1B |= (1 << CS11);  // Set CS11 bit for 8 prescaler.
    TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt.
    sei();  // Enable interrupts.
}

static bool get_thermal_overload_active(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    /*
     * From the LMD18200 datasheet:
     * Pin 9, THERMAL FLAG Output: This pin provides the thermal warning flag output signal.
     * Pin 9 becomes active- low at 145°C (junction temperature). However the chip will not
     * shut itself down until 170°C is reached at the junction.
     */
    return digitalRead(motor_pinout[motor_id].in_thermal_overload) == 0;
}

bool motor_get_thermal_overload_detected(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    if (motor_state[motor_id].error_flags & MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED)
        return true;
}

bool motor_get_thermal_overload_detected(void)
{
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (motor_get_thermal_overload_detected((motor_id_t)i))
            return true;
    }

    return false;
}

void motor_clear_thermal_overload(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    motor_state[motor_id].error_flags &= ~MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED;
}

int motor_get_current(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    // LMD18200 datasheet says 377uA/A. What's the resistance?
    return analogRead(motor_pinout[motor_id].in_current);  // 0 - 1023.
}

bool motor_stall_triggered(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return motor_state[motor_id].stall_triggered;
}

void motor_clear_stall(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    motor_state[motor_id].current = 0;  // Prevent retriggering until next time current is read.
    motor_state[motor_id].stall_triggered = false;
}

void motor_disable_all(void)
{
    for (int i = MOTOR_ID_A; i < MOTOR_ID_COUNT; i++) {
        motor_set_enabled((motor_id_t)i, false);
    }
    log_writeln(F("All motors disabled."));
}

void motor_set_enabled(motor_id_t motor_id, bool enabled)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    if (enabled && motor_state[motor_id].enabled)
        return;

    if (enabled) {
        motor_set_speed(motor_id, motor_max_speed);
        digitalWrite(motor_pinout[motor_id].out_brake, LOW);
        enabled_motors_mask |= 1 << (int)motor_id;
    } else {
        motor_set_speed(motor_id, 0);
        digitalWrite(motor_pinout[motor_id].out_brake, HIGH);
        enabled_motors_mask &= ~(1 << (int)motor_id);
    }

    motor_state[motor_id].home_triggered_debounced = motor_is_home_triggered(motor_id);
    motor_state[motor_id].target_encoder = noinit_data.motor[motor_id].encoder;
    motor_state[motor_id].pid_perror = 0;
    motor_state[motor_id].pid_dvalue = 0;
    motor_state[motor_id].enabled = enabled;
}

bool motor_get_enabled(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return motor_state[motor_id].enabled;
}

int motor_get_enabled_mask(void)
{
    return enabled_motors_mask;
}

void motor_set_enabled_mask(int mask)
{
    for (int i = MOTOR_ID_A; i < MOTOR_ID_COUNT; i++) {
        if (mask & 1 << i)
            motor_set_enabled((motor_id_t)i, true);
        else
            motor_set_enabled((motor_id_t)i, false);
    }
}

static int get_num_enabled(void)
{
    int num_enabled = 0;

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (motor_get_enabled((motor_id_t)i))
            num_enabled++;
    }

    return num_enabled;
}

void motor_set_home_encoder(motor_id_t motor_id, int home_encoder)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    noinit_data.motor[motor_id].encoder -= home_encoder;
    motor_state[motor_id].target_encoder -= home_encoder;
}

void motor_set_target_encoder(motor_id_t motor_id, int encoder)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    // TODO: assert valid encoder?
    if (!motor_get_enabled(motor_id)) {
        LOG_ERROR(F("Motor %c not enabled."), 'A' + motor_id);
        return;
    }

    if (motor_get_target_encoder(motor_id) == encoder)
        return;

    motor_state[motor_id].target_encoder = encoder * config.motor[motor_id].orientation;
    motor_state[motor_id].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET;
    motor_state[motor_id].encoders_per_second = 0;
}

int motor_get_target_encoder(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return motor_state[motor_id].target_encoder * config.motor[motor_id].orientation;
}

int motor_get_encoder(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    if (!motor_get_enabled(motor_id))
        return 0;

    return noinit_data.motor[motor_id].encoder * config.motor[motor_id].orientation;
}

bool motor_is_moving(motor_id_t motor_id)
{
    if (!motor_get_enabled(motor_id))
        return false;

    return motor_state[motor_id].target_encoder != noinit_data.motor[motor_id].encoder;
}

void motor_print_encoders(void)
{
    log_writeln(F("Current Positions: "));
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        log_write(F("%c=%d%c"), 'A' + i, motor_get_encoder((motor_id_t)i), (i < MOTOR_ID_COUNT - 1) ? ',' : ' ');
    }
    log_writeln();
}

float motor_get_encoder_steps_per_degree(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    switch (motor_id) {
    case MOTOR_ID_A:
        return 1;
    case MOTOR_ID_B:
        return 12.5;  // (5.51)(165.4/1) XR4
        break;
    case MOTOR_ID_C:
    case MOTOR_ID_D:  // Fallthrough.
    case MOTOR_ID_E:  // Fallthrough.
        if (config.robot_id == CONFIG_ROBOT_ID_RHINO_XR_4)
            return 35;                 // (8.8)(66.1/1) XR4
        else
            return 36;                 // (8.8)(66.1/1) XR3
        break;
    case MOTOR_ID_F:
        if (config.robot_id == CONFIG_ROBOT_ID_RHINO_XR_4)
            return 17.5;               // (4.4)(66.1/1) XR4
        else
            return 29.5;               // (4.4)(66.1/1) XR3
        break;
    }

    assert(false);
}

int motor_angle_to_encoder(motor_id_t motor_id, float angle)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return (angle * motor_get_encoder_steps_per_degree(motor_id)) + config.motor[motor_id].angle_offset;
}

void motor_set_target_angle(motor_id_t motor_id, float angle)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    // TODO: assert valid angle?
    int encoder = motor_angle_to_encoder(motor_id, angle);

    motor_set_target_encoder(motor_id, encoder);
}

float motor_get_angle(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return (motor_get_encoder(motor_id) - config.motor[motor_id].angle_offset) / motor_get_encoder_steps_per_degree(motor_id);
}

void motor_set_speed(motor_id_t motor_id, int speed)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    assert((speed >= motor_min_speed) && (speed <= motor_max_speed));

    if (!motor_state[motor_id].enabled) {
        motor_state[motor_id].pwm = 0;
        analogWrite(motor_pinout[motor_id].out_pwm, 0);
        return;
    }

    // Convert speed's +/- 255 value to PWM and Direction.
    if (speed == 0) {
        motor_state[motor_id].pwm = 0;
    } else {
        unsigned int pwm = abs(speed);
        unsigned int max_pwm = abs(motor_state[motor_id].max_speed);
        pwm = (pwm > max_pwm) ? max_pwm : pwm;
        pwm = (pwm < motor_min_pwm) ? motor_min_pwm : pwm;
        motor_state[motor_id].pwm = pwm;
    }

    digitalWrite(motor_pinout[motor_id].out_direction, speed >= 0 ? config.motor[motor_id].forward_polarity : !config.motor[motor_id].forward_polarity);
    analogWrite(motor_pinout[motor_id].out_pwm, motor_state[motor_id].pwm);
    motor_state[motor_id].speed = speed;
}

void motor_set_max_speed_percent(motor_id_t motor_id, int max_speed_percent)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    assert((max_speed_percent >= 0.0f) && (max_speed_percent <= 100.0f));

    motor_state[motor_id].max_speed = (motor_max_speed * max_speed_percent) / 100;
}

int motor_get_max_speed_percent(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return motor_state[motor_id].max_speed * 100 / motor_max_speed;
}

bool motor_is_home_triggered(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    return !digitalRead(motor_pinout[motor_id].in_home_switch);  // The switches are active LOW, so invert.
}

bool motor_is_home_triggered_debounced(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    return motor_state[motor_id].home_triggered_debounced;
}

static void half_wiggle(motor_id_t motor_id, int speed)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    const int delay_ms = 50;

    motor_set_speed(motor_id, speed);
    log_write(F("on, "));
    delay(delay_ms);  // Short Delay to allow the motor to move.

    motor_set_speed(motor_id, 0);
    log_write(F("off. "));
    delay(delay_ms);  // Short Delay to allow the motor to stop.
}

const int motor_test_speed = 255 - motor_min_pwm;

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

    bool was_enabled = motor_get_enabled(motor_id);
    int forward_delta = 0;
    int reverse_delta = 0;

    if (motor_is_moving(motor_id)) {
        motor_set_enabled(motor_id, false);
        delay(250);  // Wait for motor to come to rest if it's moving.
    }

    motor_set_enabled(motor_id, true);

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

        delay(1000);

        wiggle(motor_id, &forward_delta, &reverse_delta);
    }

    motor_set_enabled(motor_id, was_enabled);

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

void motor_test_enabled(void)
{
    log_writeln(F("Testing motors"));

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (motor_state[i].enabled == false)
            continue;
        delay(25);
        motor_test((motor_id_t)i);
    }

    log_writeln(F("Done testing motors."));
}

void motor_dump(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    log_writeln(F("%c: encoder:%d qe_prev:%d, speed:%d target_speed:%d orientation:%d prev_dir:%d pid_dvalue:%d pid_perror:%d target_encoder:%d current:%d progress:%d"),
                'A' + motor_id,
                noinit_data.motor[motor_id].encoder,
                noinit_data.motor[motor_id].previous_quadrature_encoder,
                motor_state[motor_id].speed,
                motor_state[motor_id].target_speed,
                config.motor[motor_id].orientation,
                motor_state[motor_id].previous_direction,
                motor_state[motor_id].pid_dvalue,
                motor_state[motor_id].pid_perror,
                motor_state[motor_id].target_encoder,
                motor_state[motor_id].current,
                motor_state[motor_id].progress);
}

void motor_set_user_error(bool enable)
{
    if (enable)
        motor_state[0].error_flags |= MOTOR_ERROR_FLAG_USER_FLAG;
    else
        motor_state[0].error_flags &= ~MOTOR_ERROR_FLAG_USER_FLAG;
}

void motor_log_errors(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    int ef = motor_state[motor_id].error_flags;

    if (ef == 0)
        return;

    if (ef & MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED)
        log_writeln(F("Motor %c thermal overload detected."), 'A' + motor_id);
    if (ef & MOTOR_ERROR_FLAG_INVALID_ENCODER_TRANSITION)
        log_writeln(F("Motor %c invalid quadrature encoder transition."), 'A' + motor_id);
    if (ef & MOTOR_ERROR_FLAG_OPPOSITE_DIRECTION)
        log_writeln(F("Motor %c direction opposite of expectation."), 'A' + motor_id);
    if (ef & MOTOR_ERROR_FLAG_UNEXPECTED_HOME_SWITCH_ENCODER)
        log_writeln(F("Motor %c unexpected home switch encoder."), 'A' + motor_id);
}

static void maybe_blink_led(void)
{
    static int led_counter = 0;

    bool motor_error = false;

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (motor_state[i].error_flags != 0)
            motor_error = true;
    }

    if (motor_error) {
        // Blink S-O-S if there's an error.
        bool led_state;
        const int len = 225;
        switch (led_counter) {
        case len * 0:  // dot.
        case len * 2:  // dot.
        case len * 4:  // dot.
        case len * 8:  // dash.
        case len * 12:  // dash.
        case len * 16:  // dash.
        case len * 23:  // dot.
        case len * 25:  // dot.
        case len * 27:  // dot.
            hardware_set_led_enabled(true);
            hardware_set_speaker_enabled(true);
            led_counter++;
            break;
        case len * 1:  // space after first dot.
        case len * 3:  // space after second dot.
        case len * 5:  // space after third dot (3x between words).
        case len * 11:  // space after first dash.
        case len * 15:  // space after second dash.
        case len * 20:  // space after third dash.
        case len * 24:  // space after fourth dot (3x between words).
        case len * 26:  // space after fifth dot.
        case len * 28:  // space after sixth dot.
            hardware_set_led_enabled(false);
            hardware_set_speaker_enabled(false);
            led_counter++;
            break;
        default:
            led_counter++;
            if (led_counter > len * 35) // len*35 = len*28 + 7.
                led_counter = 0;
            break;
        }
        return;
    }

    if (motor_get_enabled_mask() == 0) {
        hardware_set_led_enabled(false);
        hardware_set_speaker_enabled(false);
    } else {
        led_counter++;
        if (led_counter > 500) {
            bool led_state = !hardware_get_led_enabled();
            hardware_set_led_enabled(led_state);
            hardware_set_speaker_enabled(led_state);
            led_counter = 0;
        }
    }
}

static void check_home_switch(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    motor_state_t *motor = &motor_state[motor_id];

    bool home_triggered_debounced = motor_is_home_triggered_debounced(motor_id);
    bool home_triggered = motor_is_home_triggered(motor_id);

    unsigned const debounce_delay_millis = 50;

    if (home_triggered != motor->prev_home_triggered) {
        motor->prev_home_triggered = home_triggered;
        motor->prev_home_triggered_millis = millis();
        motor->prev_home_triggered_encoder = noinit_data.motor[motor_id].encoder;
    }

    // TODO: Tighten debounce timing. Example:
    // https://stackoverflow.com/questions/48434575/switch-debouncing-logic-in-c
    if ((millis() - motor->prev_home_triggered_millis) > debounce_delay_millis)
        home_triggered_debounced = home_triggered;

    // See if the home switch has changed from on to off or off to on.
    // There are 4 different motor positions stored for the home switches:
    //  - The on and off locations when the motor is moving forward.
    //  - The on and off locations when the motor is moving reverse.

    if (home_triggered_debounced != motor->home_triggered_debounced) {
        int encoder = motor->prev_home_triggered_encoder;
        if (motor->previous_direction == 1) {
            if (home_triggered_debounced)
                motor->home_forward_on_encoder = encoder;
            else
                motor->home_forward_off_encoder = encoder;
        } else if (motor->previous_direction == -1) {
            if (home_triggered_debounced)
                motor->home_reverse_on_encoder = encoder;
            else
                motor->home_reverse_off_encoder = encoder;
        }
        motor->home_triggered_debounced = home_triggered_debounced;
    }
}

// Interrupt routine that interrupts the main program at freq of 2kHz.
ISR(TIMER1_COMPA_vect) {
    static motor_id_t motor_id = MOTOR_ID_COUNT;

    const int max_error = 255 - motor_min_pwm;
    const int min_error = -(255 - motor_min_pwm);
    const int qe_inc_states[] = { 1, 3, 0, 2 };  // 01 -> 11 -> 00 -> 10.
    const int qe_dec_states[] = { 2, 0, 3, 1 };  // 10 -> 00 -> 11 -> 01.

    for (motor_id_t qe_motor_id = (motor_id_t)0; qe_motor_id < MOTOR_ID_COUNT; qe_motor_id = motor_id_t(qe_motor_id + 1)) {
        // Quadrature Encoders - read at rate of 2kHz.
        int qe_value_a = digitalRead(motor_pinout[qe_motor_id].in_quadrature_encoder_a);
        int qe_value_b = digitalRead(motor_pinout[qe_motor_id].in_quadrature_encoder_b);
        int qe_state = qe_value_a + (qe_value_b << 1);  // 0-3.

        if (qe_state == noinit_data.motor[qe_motor_id].previous_quadrature_encoder) {
            // Same state as last time through loop.
            motor_state[qe_motor_id].pid_dvalue++;
            if (motor_state[qe_motor_id].pid_dvalue > 10000)
                motor_state[qe_motor_id].pid_dvalue = 10000;
        } else if (qe_state == qe_inc_states[noinit_data.motor[qe_motor_id].previous_quadrature_encoder]) {
            // Quadrature encoder reading indicates moving in positive direction.
            motor_state[qe_motor_id].previous_direction = 1;
            if (noinit_data.motor[qe_motor_id].encoder != INT_MAX) {
                noinit_data.motor[qe_motor_id].encoder++;
                motor_state[qe_motor_id].pid_dvalue = 0;
            }
        } else if (qe_state == qe_dec_states[noinit_data.motor[qe_motor_id].previous_quadrature_encoder]) {
            // Quadrature encoder reading indicates moving in negative direction.
            motor_state[qe_motor_id].previous_direction = -1;
            if (noinit_data.motor[qe_motor_id].encoder != INT_MIN) {
                noinit_data.motor[qe_motor_id].encoder--;
                motor_state[qe_motor_id].pid_dvalue = 0;
            }
        } else if (motor_id != MOTOR_ID_COUNT) {
            // It's easy to get invalid quadrature encoder readings at boot. So motor_id is initialized to
            // MOTOR_ID_COUNT (and then quickly changed to MOTOR_ID_A below), to avoid triggering invalid
            // encoder transitions errors at boot.
            motor_state[qe_motor_id].error_flags |= MOTOR_ERROR_FLAG_INVALID_ENCODER_TRANSITION;
        }
        noinit_data.motor[qe_motor_id].previous_quadrature_encoder = qe_state;

        check_home_switch(qe_motor_id);

        if (get_thermal_overload_active(qe_motor_id))
            motor_state[qe_motor_id].error_flags |= MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED;

        if (motor_state[qe_motor_id].encoders_per_second_counts++ == (2000 / MOTOR_ID_COUNT / 3)) {
            // ISR runs at 2kHz and round-robins the six motors, Every 1/3 of a second

            // Update encoders_per_second.
            motor_state[qe_motor_id].encoders_per_second = noinit_data.motor[qe_motor_id].encoder - motor_state[qe_motor_id].encoders_per_second_start_encoder;
            motor_state[qe_motor_id].encoders_per_second *= 3;
            motor_state[qe_motor_id].encoders_per_second_start_encoder = noinit_data.motor[qe_motor_id].encoder;
            motor_state[qe_motor_id].encoders_per_second_counts = 0;

            // Update current draw.
            motor_state[qe_motor_id].current = motor_get_current(qe_motor_id);
        }
    }

    maybe_blink_led();
    hardware_debounce_buttons();

    //==========================================================
    // Calculate Motor status values.
    //==========================================================
    // Calculate only one motor per interupt - rate of 333Hz
    // This is done by stepping motor_id once per interupt.
    // motor_id is then used to specifiy which motor to do
    // calculations on.
    //==========================================================
    motor_id = motor_id_t(motor_id + 1);
    if (motor_id >= MOTOR_ID_COUNT)
        motor_id = MOTOR_ID_A;

    //==========================================================
    // See if the Motor PID needs to be turned on.
    if (false) {
        // The IntMotor PID is NOT running.
        // ( =1230 instead of =0 is just to kill the routine for now because it isn't doing what it needs to do.)
        motor_state[motor_id].pid_dvalue = 0;  // Clear the PID DValue for this motor.
        int intMDiff = abs(motor_state[motor_id].target_encoder - noinit_data.motor[motor_id].encoder);
        // if (intMDiff > 3)
        //    Motor_PID[motor_id] = 1;   // Turn on the PID for this Motor.
    } else {
        // Brakes are not on - so the IntMotor is running.
        //==========================================================
        // Check for stall.
        // Note that the Gripper is excluded because it needs to
        // apply tension on whatever it is gripping.
        //==========================================================

        if (motor_id != config.gripper_motor_id) {
            // For Motors other than the gripper, High Current means
            // that the motor is in a stall situation. To unstall,
            // the target position is set back a bit from the
            // current position.

            const int destall_delta = 50;
            if (motor_state[motor_id].current > config.motor[motor_id].stall_current_threshold) {
                if (motor_state[motor_id].previous_direction == 1)
                    motor_state[motor_id].target_encoder = noinit_data.motor[motor_id].encoder - destall_delta;
                else if (motor_state[motor_id].previous_direction == -1)
                    motor_state[motor_id].target_encoder = noinit_data.motor[motor_id].encoder + destall_delta;
                motor_state[motor_id].current = 0;  // Prevent retriggering until next time current is read.
                motor_state[motor_id].stall_triggered = true;
            }
        } else if (motor_state[motor_id].current > 150) {
            // The gripper motor is a special case where high current means that the gripper is gripping
            // something. Create gripper pressure on by setting the target position to the currernt
            // position. This will cause the PWM to drop to off, and if the relaxed gripper opens a
            // little, it will turn back on but at a much lower PWM duty cycle.

            static int gripper_stall_encoder = INT_MAX;
            static unsigned long gripper_stall_start_ms = 0;

            if (abs(noinit_data.motor[motor_id].encoder - gripper_stall_encoder) > 3) {
                gripper_stall_encoder = noinit_data.motor[motor_id].encoder;
                gripper_stall_start_ms = millis();
            } else if (millis() - gripper_stall_start_ms > 250) {
                motor_state[motor_id].target_encoder = noinit_data.motor[motor_id].encoder;
                gripper_stall_start_ms = millis();
            }
        }

        //==========================================================
        // Calculate PID Proportional Error
        //==========================================================
        int pid_perror = 0;
        int target = motor_state[motor_id].target_encoder;
        int encoder = -noinit_data.motor[motor_id].encoder;
        if ((encoder > 0) && (target > INT_MAX - encoder))
            pid_perror = INT_MAX;      // Handle overflow by clamping to INT_MAX.
        else if ((encoder < 0) && (target < INT_MIN - encoder))
            pid_perror = INT_MIN;      // Handle underflow by clamping to INT_MIN.
        else
            pid_perror = target + encoder;

        motor_state[motor_id].pid_perror = pid_perror;  // Save.

        //==========================================================
        // Calc the Target Speed from the Proportional Error
        // The target speed is just the difference between the
        //  Current Position and the Target Position (with limits).
        // Results in a speed of +/- 255.
        //==========================================================
        if (pid_perror > max_error) {
            motor_state[motor_id].target_speed = max_error;
            // Set the Status that indicates that the Motor is more than 200 clicks from target.
            motor_state[motor_id].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET;
        } else if (pid_perror < min_error) {
            motor_state[motor_id].target_speed = min_error;
            // Set the Status that indicates that the Motor is more than 200 clicks from target.
            motor_state[motor_id].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET;
        } else if (pid_perror > 0) {  // TODO: Refactor to combine pid_perror > 0 and < 0 cases.
            motor_state[motor_id].target_speed = motor_state[motor_id].pid_perror + (motor_state[motor_id].pid_dvalue / 6);
            if (pid_perror < 2)
                // Set the Status that indicates that the Motor is 1 click from target
                motor_state[motor_id].progress = MOTOR_PROGRESS_BESIDE_TARGET;
            else if (pid_perror < 30)
                // Set the Status that indicates that the Motor is 2-29 clicks from target
                motor_state[motor_id].progress = MOTOR_PROGRESS_NEAR_TARGET;
            else
                // Set the Status that indicates that the Motor is 30-200 clicks from target
                motor_state[motor_id].progress = MOTOR_PROGRESS_APPROACHING_TARGET;
        } else if (pid_perror < 0) {
            motor_state[motor_id].target_speed = motor_state[motor_id].pid_perror - (motor_state[motor_id].pid_dvalue / 6), -255;  // TODO: Check.
            if (pid_perror > -2)
                // Set the Status that indicates that the Motor is 1 click from target
                motor_state[motor_id].progress = MOTOR_PROGRESS_BESIDE_TARGET;
            else if (pid_perror > -30)
                // Set the Status that indicates that the Motor is 2-29 clicks from target
                motor_state[motor_id].progress = MOTOR_PROGRESS_NEAR_TARGET;
            else
                // Set the Status that indicates that the Motor is 30-200 clicks from target
                motor_state[motor_id].progress = MOTOR_PROGRESS_APPROACHING_TARGET;
        } else {
            motor_state[motor_id].target_speed = 0;
            motor_state[motor_id].progress = MOTOR_PROGRESS_AT_TARGET;  // Clear the flag that indicates that the Motor is in motion.
            // Motor_PID[motor_id] = 0;  // Turn off motor_id's PID.
        }

        //==========================================================
        // PID (Currenty Just the P)
        //==========================================================
        if (motor_state[motor_id].enabled) {
            //============================================
            // Ramp Up/Down Current Speed to Target Speed.
            // Prevents the motors from jumping from dead
            //  stop to full speed and vice-versa
            //============================================
            int CurrentSpeedChanged = 0;
            if (motor_state[motor_id].target_speed > motor_state[motor_id].speed) {
                if (motor_state[motor_id].speed < (255 - motor_min_pwm)) {
                    motor_state[motor_id].speed++;  // if the target is higher, then inc up to the target.
                    if (motor_state[motor_id].target_speed > motor_state[motor_id].speed)
                        if (motor_state[motor_id].speed < (255 - motor_min_pwm))
                            motor_state[motor_id].speed++;  // if the target is higher, then inc up to the target a second time.
                    CurrentSpeedChanged = 1;
                }
            } else if (motor_state[motor_id].target_speed < motor_state[motor_id].speed) {
                if (motor_state[motor_id].speed > -(255 - motor_min_pwm)) {
                    motor_state[motor_id].speed--;  // if the target is lower, then inc down to the target.
                    if (motor_state[motor_id].target_speed < motor_state[motor_id].speed)
                        if (motor_state[motor_id].speed > -(255 - motor_min_pwm))
                            motor_state[motor_id].speed--;  // if the target is lower, then inc down to the target a second time.
                    CurrentSpeedChanged = 1;
                }
            }

            if (CurrentSpeedChanged == 1)
                motor_set_speed(motor_id, motor_state[motor_id].speed);
        }
    }
}
