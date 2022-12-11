/*
 * Implementation for MegaMotor6 hardware functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <limits.h>
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "motor.h"

// MM6 motor I/O lines.
typedef struct {
    unsigned short out_direction;      // Digital. LOW = forward direction. HIGH = reverse direction.
    unsigned short out_pwm;            // Digital.
    unsigned short out_brake;          // Digital. LOW = disable brake. HIGH = enable brake.
    unsigned short in_current_draw;    // Analog. Units unclear in datasheet.
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

motor_t motor[MOTOR_ID_COUNT] = { 0 };  // Intentionally not static, so can be accessed globally.

// The velocity sign bit is used to set the LMD18200 direction pin. So the PWM register accepts 0-255 for + and -.
static const int motor_min_pwm = 55;
static const int motor_max_velocity = 255 - motor_min_pwm;
static const int motor_min_velocity = -motor_max_velocity;

static int enabled_motors_mask = 0;
static bool high_level_error_detected = false;

static const char error_name_thermal_overload_detected[] PROGMEM = "thermal overload";
static const char error_name_invalid_quadrature_encoder_transition[] PROGMEM = "invalid quadrature encoder transition";
static const char error_name_stall_current_threshold_exceeded[] PROGMEM = "stall current threshold exceeded";

const char *const motor_error_name_by_id[MOTOR_ERROR_COUNT] = {
    error_name_thermal_overload_detected,
    error_name_invalid_quadrature_encoder_transition,
    error_name_stall_current_threshold_exceeded,
};

// Configuration stored in RAM and saved across reset/reboot but not saved across power-cycling of the board.
static struct {
    // nbytes, version, magic are used to verify valid data.
    size_t nbytes;
    int    version;
    int    magic;
    struct {
        int prev_encoder_state;
        int encoder;
    } motor[MOTOR_ID_COUNT];
} persistent_ram_data __attribute__((section(".noinit")));  // NOT reset to 0 when the CPU is reset.

const int prev_encoder_state_init_value = -1;

void motor_clear_persistent_ram_data(void)
{
    memset(&persistent_ram_data, 0, sizeof(persistent_ram_data));
}

static void check_persistent_ram_data(void)
{
    // Zero out saved variables on power cycle. On reset, these values are NOT erased.
    const int persistent_ram_data_version = 2;
    const int persistent_ram_data_magic = 0xABCD1234;

    if ((persistent_ram_data.nbytes != sizeof(persistent_ram_data) || (persistent_ram_data.version != persistent_ram_data_version) || (persistent_ram_data.magic != persistent_ram_data_magic))) {
        log_writeln(F("Initializing persistent RAM data."));
        persistent_ram_data.nbytes = sizeof(persistent_ram_data);
        persistent_ram_data.version = persistent_ram_data_version;
        persistent_ram_data.magic = persistent_ram_data_magic;
        for (int i = 0; i < MOTOR_ID_COUNT; i++) {
            persistent_ram_data.motor[i].encoder = 0;
            persistent_ram_data.motor[i].prev_encoder_state = prev_encoder_state_init_value;
        }
    } else {
        log_writeln(F("Detected reset without RAM clear. Reusing in-RAM motor encoder values."));
        log_writeln();
    }
}

static void init_motor(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    // Configure and initialize GPIO outputs.
    pinMode(motor_pinout[motor_id].out_brake, OUTPUT);
    pinMode(motor_pinout[motor_id].out_direction, OUTPUT);
    pinMode(motor_pinout[motor_id].out_pwm, OUTPUT);
    digitalWrite(motor_pinout[motor_id].out_direction, config.motor[motor_id].forward_polarity);

    // Configure GPIO inputs.
    pinMode(motor_pinout[motor_id].in_current_draw, INPUT);
    pinMode(motor_pinout[motor_id].in_thermal_overload, INPUT_PULLUP);
    pinMode(motor_pinout[motor_id].in_home_switch, INPUT_PULLUP);
    pinMode(motor_pinout[motor_id].in_quadrature_encoder_a, INPUT_PULLUP);
    pinMode(motor_pinout[motor_id].in_quadrature_encoder_b, INPUT_PULLUP);

    // Initialize motor state.
    memset(&motor[motor_id], 0, sizeof(motor_t));
    motor_set_max_velocity_percent((motor_id_t)motor_id, 100);
    motor[motor_id].prev_home_is_pressed = motor_home_is_pressed((motor_id_t)motor_id);
    motor[motor_id].prev_home_millis = millis();
    motor[motor_id].prev_home_encoder = INT_MAX;
    motor[motor_id].home_is_pressed_debounced = motor[motor_id].prev_home_is_pressed;

    motor_set_enabled(motor_id, false);
}

void motor_init(void)
{
    check_persistent_ram_data();

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        init_motor(i);
    }

    // Timer setup: Allows preceise timed measurements of the quadrature encoder.
    // From http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
    // TIMER 1 for interrupt frequency 2000 Hz:
    cli();     // stop interrupts
    TCCR1A = 0;  // set entire TCCR1A register to 0
    TCCR1B = 0;  // same for TCCR1B
    TCNT1 = 0;  // initialize counter value to 0
    // set compare match register for 2000 Hz increments
    OCR1A = 7999;  // = 16000000 / (1 * 2000) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12, CS11 and CS10 bits for 1 prescaler
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();  // allow interrupts
}

static bool get_thermal_overload_active(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    /*
     * From the LMD18200 datasheet:
     * Pin 9, THERMAL FLAG Output: This pin provides the thermal warning flag output signal.
     * Pin 9 becomes active-low at 145°C (junction temperature). However the chip will not
     * shut itself down until 170°C is reached at the junction.
     */
    return digitalRead(motor_pinout[motor_id].in_thermal_overload) == 0;
}

int motor_get_current_draw(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    // LMD18200 datasheet says 377uA/A. What's the resistance?
    return analogRead(motor_pinout[motor_id].in_current_draw);  // 0 - 1023.
}

bool motor_stall_triggered(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return motor[motor_id].stall_triggered;
}

void motor_clear_stall(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    motor[motor_id].current_draw = 0;  // Prevent retriggering until next time current is read.
    motor[motor_id].stall_triggered = false;
}

void motor_disable_all(void)
{
    for (int i = MOTOR_ID_A; i < MOTOR_ID_COUNT; i++) {
        motor_set_enabled((motor_id_t)i, false);
    }
    log_writeln(F("All motors disabled."));
}

void motor_stop_all(void)
{
    for (int i = MOTOR_ID_A; i < MOTOR_ID_COUNT; i++) {
        motor_set_target_encoder((motor_id_t)i, motor_get_encoder(i));
    }
    log_writeln(F("All motor targets set to current positions."));
}

void motor_set_enabled(motor_id_t motor_id, bool enabled)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    if (enabled && motor[motor_id].enabled)
        return;

    if (enabled) {
        digitalWrite(motor_pinout[motor_id].out_brake, LOW);  // Enable LM18200 motor-driver transistors by disabling motor BREAK.
        analogWrite(motor_pinout[motor_id].out_pwm, 0);
        enabled_motors_mask |= 1 << (int)motor_id;
    } else {
        // Disable LM18200 motor-driver transistors by enabling motor BREAK and disabling PWM output.
        digitalWrite(motor_pinout[motor_id].out_brake, HIGH);
        analogWrite(motor_pinout[motor_id].out_pwm, 0);
        enabled_motors_mask &= ~(1 << (int)motor_id);
    }

    motor[motor_id].velocity = 0;
    motor[motor_id].pwm = 0;
    motor[motor_id].home_is_pressed_debounced = motor_home_is_pressed(motor_id);
    motor[motor_id].target_encoder = persistent_ram_data.motor[motor_id].encoder;
    motor[motor_id].pid_perror = 0;
    motor[motor_id].pid_dvalue = 0;

    motor[motor_id].enabled = enabled;
}

bool motor_get_enabled(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return motor[motor_id].enabled;
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
        if (enabled_motors_mask & (1 << i))
            num_enabled++;
    }

    return num_enabled;
}

void motor_set_home_encoder(motor_id_t motor_id, int home_encoder)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    // TODO: Determine if/how orientation should be considered here.
    persistent_ram_data.motor[motor_id].encoder -= home_encoder;
    motor[motor_id].target_encoder -= home_encoder;
}

int motor_get_encoder(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return persistent_ram_data.motor[motor_id].encoder * config.motor[motor_id].orientation;
}

void motor_set_target_encoder(motor_id_t motor_id, int encoder)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    if (motor_get_target_encoder(motor_id) == encoder)
        return;

    encoder = encoder = max(config.motor[motor_id].min_encoder, min(config.motor[motor_id].max_encoder, encoder));

    motor[motor_id].target_encoder = encoder * config.motor[motor_id].orientation;
}

int motor_get_target_encoder(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return motor[motor_id].target_encoder * config.motor[motor_id].orientation;
}

float motor_get_angle(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return config_motor_encoders_to_angle(motor_id, motor_get_encoder(motor_id));
}

void motor_set_target_angle(motor_id_t motor_id, float angle)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    motor_set_target_encoder(motor_id, config_motor_angle_to_encoders(motor_id, angle));
}

float motor_get_percent(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    float percent = ((motor_get_encoder(motor_id) - config.motor[motor_id].min_encoder) * 100.0) / (config.motor[motor_id].max_encoder - config.motor[motor_id].min_encoder);

    return fmax(0.0, fmin(100.0, percent));  // Paranoid clamp to avoid rounding < 0.0 or > 100.0.
}

void motor_set_target_percent(motor_id_t motor_id, float percent)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    assert((percent >= 0) && (percent <= 100.0));
    assert(config.motor[motor_id].max_encoder < INT_MAX);
    assert(config.motor[motor_id].min_encoder > INT_MIN);

    int encoder = config.motor[motor_id].min_encoder + (config.motor[motor_id].max_encoder - config.motor[motor_id].min_encoder) * percent / 100.0;

    motor_set_target_encoder(motor_id, encoder);
}

bool motor_is_moving(motor_id_t motor_id)
{
    if (!motor_get_enabled(motor_id))
        return false;

    return motor[motor_id].target_encoder != persistent_ram_data.motor[motor_id].encoder;
}

static void set_velocity(motor_id_t motor_id, int velocity)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    assert((velocity >= motor_min_velocity) && (velocity <= motor_max_velocity));

    if (!motor[motor_id].enabled) {
        motor[motor_id].velocity = 0;
        motor[motor_id].pwm = 0;
        analogWrite(motor_pinout[motor_id].out_pwm, 0);
        return;
    }

    // Convert velocity's +/- 255 value to PWM and Direction.
    if (velocity == 0) {
        motor[motor_id].pwm = 0;
    } else {
        // TODO: Cleanup/refactor velocity, max_velocity to be 0-100.
        unsigned int pwm = abs(velocity) + motor_min_pwm;
        unsigned int max_pwm = abs(motor[motor_id].max_velocity) + motor_min_pwm;
        pwm = max(min(pwm, max_pwm), motor_min_pwm);
        motor[motor_id].pwm = pwm;
    }

    digitalWrite(motor_pinout[motor_id].out_direction, velocity >= 0 ? config.motor[motor_id].forward_polarity : !config.motor[motor_id].forward_polarity);
    analogWrite(motor_pinout[motor_id].out_pwm, motor[motor_id].pwm);
    motor[motor_id].velocity = velocity;
}

void motor_set_max_velocity_percent(motor_id_t motor_id, int max_velocity_percent)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    assert((max_velocity_percent >= 0.0f) && (max_velocity_percent <= 100.0f));

    motor[motor_id].max_velocity = (motor_max_velocity * max_velocity_percent) / 100;
}

int motor_get_max_velocity_percent(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    return motor[motor_id].max_velocity * 100 / motor_max_velocity;
}

bool motor_home_is_pressed(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    return !digitalRead(motor_pinout[motor_id].in_home_switch);  // Home switches are active LOW, so invert.
}

bool motor_home_is_pressed_debounced(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    return motor[motor_id].home_is_pressed_debounced;
}

void motor_dump(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    char angle_str[15] = {};

    dtostrf(motor_get_angle(motor_id), 3, 2, angle_str);

    int orientation = config.motor[motor_id].orientation;

    log_writeln(F("  %c%s: enc:%d target_enc:%d perror:%d dvalue:%d vel:%d orient:%d prev_dir:%d PWM:%d cur:%d is_home:%d hs:%d,%d,%d,%d->%d angle:%s"),
                'A' + motor_id,
                ((motor_get_enabled_mask() & (1 << motor_id)) == 0) ? " [not enabled]" : "",
                motor_get_encoder(motor_id),
                motor[motor_id].target_encoder * orientation,
                motor[motor_id].pid_perror * orientation,
                motor[motor_id].pid_dvalue,
                motor[motor_id].velocity * orientation,
                orientation,
                motor[motor_id].prev_direction,
                motor[motor_id].pwm,
                motor[motor_id].current_draw,
                motor[motor_id].home_is_pressed_debounced,
                motor[motor_id].home_reverse_off_encoder,
                motor[motor_id].home_forward_on_encoder,
                motor[motor_id].home_reverse_on_encoder,
                motor[motor_id].home_forward_off_encoder,
                (motor[motor_id].home_forward_off_encoder + motor[motor_id].home_reverse_on_encoder + motor[motor_id].home_forward_on_encoder + motor[motor_id].home_reverse_off_encoder) / 4,
                angle_str);
}

static void set_error(motor_id_t motor_id, motor_error_t motor_error_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));
    assert((motor_error_id >= 0) && (motor_error_id < MOTOR_ERROR_COUNT));

    cli();
    motor[motor_id].error_flags_isr |= (1 << motor_error_id);
    motor[motor_id].error_flags |= (1 << motor_error_id);
    sei();
}

void motor_set_high_level_error(bool has_error)
{
    cli();
    high_level_error_detected = has_error;
    sei();
}

int motor_get_error_flags(motor_id_t motor_id)
{
    assert((motor_id >= 0) && (motor_id < MOTOR_ID_COUNT));

    cli();
    unsigned char ret = motor[motor_id].error_flags;  // OR-mask of all errors set since last call to this function.

    motor[motor_id].error_flags = motor[motor_id].error_flags_isr;  // Reset to flags set previous time through ISR.
    sei();

    return ret;
}

static void isr_read_encoder(motor_id_t i)
{
    /* To understand the quadrature encoder state_to_{next,prev}_value[]:
     *   If the previous reading was 0, and the new reading is 1, the direction is +1.
     *   If the previous reading was 0, and the new reading is 2, the direction is -1.
     *   If the previous reading was 1, and the new reading is 3, the direction is +1.
     *   If the previous reading was 1, and the new reading is 0, the direction is -1.
     *   etc.
     */
    const unsigned char state_to_next_value[] = { 1, 3, 0, 2 };
    const unsigned char state_to_prev_value[] = { 2, 0, 3, 1 };

    unsigned char quadrature_encoder_a = digitalRead(motor_pinout[i].in_quadrature_encoder_a);
    unsigned char quadrature_encoder_b = digitalRead(motor_pinout[i].in_quadrature_encoder_b);
    unsigned char encoder_state = quadrature_encoder_a + (quadrature_encoder_b << 1);  // Is in [0, 3].

    if (encoder_state == state_to_next_value[persistent_ram_data.motor[i].prev_encoder_state]) {
        motor[i].prev_direction = +1;
        if (persistent_ram_data.motor[i].encoder != INT_MAX) {
            persistent_ram_data.motor[i].encoder++;
            motor[i].pid_dvalue = 0;
        }
    } else if (encoder_state == state_to_prev_value[persistent_ram_data.motor[i].prev_encoder_state]) {
        motor[i].prev_direction = -1;
        if (persistent_ram_data.motor[i].encoder != INT_MIN) {
            persistent_ram_data.motor[i].encoder--;
            motor[i].pid_dvalue = 0;
        }
    } else if (encoder_state == persistent_ram_data.motor[i].prev_encoder_state) {
        // No movement detected.
        // TOOD !!: motor[i].previous_direction = 0; ??
        motor[i].pid_dvalue++;
        if (motor[i].pid_dvalue > 10000)
            motor[i].pid_dvalue = 10000;
    } else {
        // Only set an error if persistent ram data wasn't just initialized.
        if (persistent_ram_data.motor[i].prev_encoder_state != prev_encoder_state_init_value)
            set_error(i, MOTOR_ERROR_INVALID_ENCODER_TRANSITION);
    }

    persistent_ram_data.motor[i].prev_encoder_state = encoder_state;
}

static void isr_read_home_switch(motor_id_t motor_id)
{
    motor_t *m = &motor[motor_id];

    bool is_pressed = !digitalRead(motor_pinout[motor_id].in_home_switch);  // Switch reads active-low when pressed.

    if (is_pressed != m->prev_home_is_pressed) {
        // Switch reading inverted from previous reading, record the new reading, time, and encoder.
        m->prev_home_is_pressed = is_pressed;
        m->prev_home_millis = millis();
        m->prev_home_encoder = persistent_ram_data.motor[motor_id].encoder;
        return;
    }

    if (is_pressed == m->home_is_pressed_debounced)
        return;                        // No inversion (or inverted an even number of times), so return directly.

    const unsigned debounce_delay_millis = 50;  // Empirically determined.

    if ((millis() - m->prev_home_millis) <= debounce_delay_millis)
        return;                        // Not enough time has elapsed to debounce the switch reading, so return directly.

    m->home_is_pressed_debounced = is_pressed;

    // There are 4 different motor positions stored for the home switches:
    //   The on and off locations when the motor is moving forward.
    //   The on and off locations when the motor is moving reverse.

    int encoder = m->prev_home_encoder;

    if (m->prev_direction == 1) {
        if (is_pressed)
            m->home_forward_on_encoder = encoder;
        else
            m->home_forward_off_encoder = encoder;
    } else if (m->prev_direction == -1) {
        if (is_pressed)
            m->home_reverse_on_encoder = encoder;
        else
            m->home_reverse_off_encoder = encoder;
    }
}

static void isr_read_all_encoders_and_home_switches(void)
{
    for (motor_id_t i = 0; i < MOTOR_ID_COUNT; i = motor_id_t(i + 1)) {
        isr_read_encoder(i);
        isr_read_home_switch(i);
    }
}

static void isr_check_thermal_overload(motor_id_t motor_id)
{
    if (get_thermal_overload_active(motor_id))
        set_error(motor_id, MOTOR_ERROR_THERMAL_OVERLOAD_DETECTED);
}

static void isr_check_is_stalled(motor_id_t motor_id)
{
    motor[motor_id].current_draw = motor_get_current_draw(motor_id);  // Update current reading even if disabled.

    // stall_current_threshold <= 0 disables checking stall current.
    if ((config.motor[motor_id].stall_current_threshold) <= 0 || (motor[motor_id].current_draw <= config.motor[motor_id].stall_current_threshold))
        return;

    if (!motor[motor_id].enabled) {
        // If the motor is disabled, but the threshold is exceeded, trigger an error.
        // Or, if the motor is at its target, but the threshold is exceeded, trigger an error.
        set_error(motor_id, MOTOR_ERROR_UNEXPECTED_STALL_CURRENT_THRESHOLD_EXCEEDED);
        return;
    }

    if (config.motor[motor_id].is_gripper) {
        // The gripper motor is a special case where high current means that the gripper is gripping
        // something. Create gripper pressure on by setting the target position to the currernt
        // position. This will cause the PWM to drop to off, and if the relaxed gripper opens a
        // little, it will turn back on but at a much lower PWM duty cycle.

        static int gripper_stall_encoder = INT_MAX;
        static unsigned long gripper_stall_start_ms = 0;

        if (abs(persistent_ram_data.motor[motor_id].encoder - gripper_stall_encoder) > 3) {
            gripper_stall_encoder = persistent_ram_data.motor[motor_id].encoder;
            gripper_stall_start_ms = millis();
        } else if (millis() - gripper_stall_start_ms > 250) {
            motor[motor_id].target_encoder = persistent_ram_data.motor[motor_id].encoder;
            gripper_stall_start_ms = millis();
        }
        return;
    }

    // For Motors other than the gripper, high current means that the motor is in a stall situation. To unstall,
    // the target position is set back a bit from the current position.
    const int destall_encoder_delta = 50;

    motor[motor_id].target_encoder = persistent_ram_data.motor[motor_id].encoder + (destall_encoder_delta * -motor[motor_id].prev_direction);
    motor[motor_id].current_draw = 0;  // Prevent retriggering until next time current is read.
    motor[motor_id].stall_triggered = true;
}

static void isr_calculate_pid_values(motor_id_t motor_id)
{
    if (!motor[motor_id].enabled)
        return;

    // Calculate proportional error.
    int32_t pid_perror = (int32_t)motor[motor_id].target_encoder - (int32_t)persistent_ram_data.motor[motor_id].encoder;

    pid_perror = min(INT_MAX, max(INT_MIN, pid_perror));
    motor[motor_id].pid_perror = pid_perror;

    // Calculate the target velocity from the proportional error. The target velocity is just the difference between the
    // current position and the target position clamped to [-255, 255].
    int target_velocity = min(motor_max_velocity, max(motor_min_velocity, pid_perror));

    // Enable or disable the LM18200 motor-driver transistors when the motor is going into or coming out of rest.
    if (motor[motor_id].pwm == 0) {
        if (target_velocity == 0) {
            // At rest: Disable transistors by enabling motor BREAK and disabling PWM output.
            digitalWrite(motor_pinout[motor_id].out_brake, HIGH);
            analogWrite(motor_pinout[motor_id].out_pwm, 0);
            return;
        } else {
            // Coming out of rest: Enable transistors by disabling motor BREAK. PWM output is updated below.
            digitalWrite(motor_pinout[motor_id].out_brake, LOW);  // Disable motor BREAK.
        }
    }

    if (target_velocity == motor[motor_id].velocity)
        return;

    // TODO: Make this a proper proporitional controller.
    int velocity = motor[motor_id].velocity;

    velocity += ((target_velocity > velocity) ? 1 : -1);  // Ramp to target_velocity.
    if (target_velocity > velocity)
        velocity++;
    else if (target_velocity < velocity)
        velocity--;

    velocity = min(motor_max_velocity, max(motor_min_velocity, velocity));
    set_velocity(motor_id, velocity);
}

static void isr_process_next_motor(void)
{
    static motor_id_t motor_id = MOTOR_ID_A;

    motor[motor_id].error_flags_isr = 0;  // Clear flags.

    isr_check_thermal_overload(motor_id);
    isr_check_is_stalled(motor_id);
    isr_calculate_pid_values(motor_id);

    motor[motor_id].error_flags |= motor[motor_id].error_flags_isr;  // Or into the error mask.

    motor_id = motor_id_t(motor_id + 1);
    if (motor_id >= MOTOR_ID_COUNT)
        motor_id = MOTOR_ID_A;
}

static void isr_maybe_toggle_led(void)
{
    // Blink the LED at 1Hz when motors are active and there's no error, or at 10Hz when there's an error.
    static char ticks_remaining = 10;

    if (--ticks_remaining > 0)
        return;

    bool error_detected = high_level_error_detected;

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (motor[i].error_flags != 0)
            error_detected = true;
    }

    if (error_detected) {
        ticks_remaining = 1;  // Error, so check again in 1 tick (0.1 seconds).
    } else {
        ticks_remaining = 10;  // No error, so check again in 10 ticks (1 second).
        if (motor_get_enabled_mask() == 0) {
            // If no error, and not enabled, disable LED and speaker.
            hardware_set_led_enabled(false);
            hardware_set_speaker_enabled(false);
            return;
        }
    }

    bool led_state = !hardware_get_led_enabled();

    hardware_set_led_enabled(led_state);
    hardware_set_speaker_enabled(led_state);
}

ISR(TIMER1_COMPA_vect) {
    // Interrupt Service Routine (ISR) that interrupts the main program at frequency of 2kHz.

    const int counter_10Hz_reset = 200;
    static int counter_10Hz = counter_10Hz_reset;

    if (--counter_10Hz <= 0) {
        hardware_debounce_buttons();
        isr_maybe_toggle_led();
        counter_10Hz = counter_10Hz_reset;
    }

    isr_read_all_encoders_and_home_switches();
    isr_process_next_motor();  // One motor at 2kHz, all motors at 333Hz (2kHz/MOTOR_ID_COUNT).
}
