/*
 * Implementation for command processing functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <limits.h>
#include <stdlib.h>
#include "calibrate.h"
#include "command.h"
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "menu.h"
#include "motor.h"
#include "motor_test.h"
#include "parse.h"
#include "sm.h"
#include "waypoint.h"

static const float software_version = 2.00;
static const size_t command_args_max_nbytes = 64;
static sm_state_t exit_to_state = { 0 };  // Used by go_home and poll_pins.
static bool poll_pins_prev_val[8] = { false };  // TODO: Fix this hard-coded size of 8.

int command_print_config(char *args, size_t args_nbytes)
{
    assert(args);

    // Confirm arguments are empty.
    size_t nbytes = parse_whitespace(args, args_nbytes);

    if (args_nbytes != nbytes)
        return -1;

    config_print();

    return 0;
}

int command_config_angle_offset(char *args, size_t args_nbytes)
{
    assert(args);

    float angle_offset = 0;

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_float(p, args_nbytes, &angle_offset);
    if (nbytes <= 0)
        return -1;
    p += nbytes;
    args_nbytes -= nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    config_set_motor_angle_offset(motor_id, angle_offset);
    config_print_one_motor(motor_id);

    return p - args;
}

int command_config_boot_mode(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;

    size_t nbytes = parse_whitespace(p, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes == 0) {
        log_write(F("Maintaining boot mode as '"));
        log_write((const __FlashStringHelper *)config_boot_mode_by_id[config.boot_mode]);
        log_writeln(F("'."));
        return p - args;
    }

    char *command_table[] = { "U", "W" };
    const int command_table_nentries = 2;
    int boot_mode = -1;

    nbytes = parse_string_in_table(p, 1, command_table, command_table_nentries, &boot_mode);
    args_nbytes -= nbytes;
    p += nbytes;

    if ((boot_mode < 0) || (boot_mode > CONFIG_BOOT_MODE_COUNT)) {
        log_writeln(F("ERROR: Invalid boot mode."));
        return 0;
    }

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_write(F("Setting boot mode to '"));
    log_write((const __FlashStringHelper *)config_boot_mode_by_id[boot_mode]);
    log_writeln(F("'."));

    config_set_boot_mode(boot_mode);

    return p - args;
}

int command_config_encoders_per_degree(char *args, size_t args_nbytes)
{
    assert(args);

    float encoders_per_degree = 0;

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_float(p, args_nbytes, &encoders_per_degree);
    if (nbytes <= 0)
        return -1;
    p += nbytes;
    args_nbytes -= nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    config_set_motor_encoders_per_degree(motor_id, encoders_per_degree);
    config_print_one_motor(motor_id);

    return p - args;
}

int command_config_gpio_pin_mode(char *args, size_t args_nbytes)
{
    assert(args);

    int gpio_pin = 0;
    bool cancel = false;
    char *p = args;
    hardware_gpio_pin_mode_t gpio_mode = -1;

    // Read.

    size_t nbytes = parse_int(p, args_nbytes, &gpio_pin);

    if (nbytes <= 0)
        return -1;
    p += nbytes;
    args_nbytes -= nbytes;

    char *command_table[] = { "i", "z", "o" };
    const int command_table_nentries = 3;

    nbytes = parse_string_in_table(p, 1, command_table, command_table_nentries, (int *)&gpio_mode);
    if (gpio_mode == -1) {
        log_writeln(F("Invalid GPIO mode. Mode should be 'i', 'z', or 'o'."));
        cancel = true;
    }

    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_whitespace(p, args_nbytes);
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    // Validate.

    if ((gpio_pin < 0) || (gpio_pin >= HARDWARE_GPIO_PIN_COUNT)) {
        log_writeln(F("Invalid GPIO pin number %d. GPIO pin number should be in the range [0, %d]."), gpio_pin, HARDWARE_GPIO_PIN_COUNT - 1);
        cancel = true;
    }

    if (cancel) {
        log_writeln(F("Canceling."));
        return p - args;
    }

    // Execute.

    config_set_gpio_pin_mode(gpio_pin, gpio_mode);
    log_writeln(F("GPIO pin configuration:"));
    log_write(F("  "));
    config_print_one_gpio_pin_config(gpio_pin);
    log_writeln();

    return p - args;
}

int command_config_home_encoder(char *args, size_t args_nbytes)
{
    assert(args);

    int encoder = 0;

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_int(p, args_nbytes, &encoder);
    if (nbytes <= 0)
        return -1;
    p += nbytes;
    args_nbytes -= nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    config_set_motor_home_encoder(motor_id, encoder);
    motor_set_home_encoder(motor_id, encoder);
    config_print_one_motor(motor_id);

    return p - args;
}

int command_config_invert_motor_orientation(char *args, size_t args_nbytes)
{
    assert(args);

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_writeln(F("Changing orientation of motor %c to '%sinverted'."),
                'A' + motor_id, config.motor[motor_id].orientation == MOTOR_ORIENTATION_INVERTED ? "not " : "");

    config_set_motor_orientation(motor_id, (motor_orientation_t)(config.motor[motor_id].orientation * -1));
    config_print_one_motor(motor_id);

    return p - args;
}

int command_config_min_max_encoders(char *args, size_t args_nbytes)
{
    assert(args);

    int min_encoder = 0;
    int max_encoder = 0;

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_int(p, args_nbytes, &min_encoder);
    if (nbytes <= 0)
        return -1;
    p += nbytes;
    args_nbytes -= nbytes;

    nbytes = parse_int(p, args_nbytes, &max_encoder);
    if (nbytes <= 0)
        return -1;
    p += nbytes;
    args_nbytes -= nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    config_set_motor_min_max_encoders(motor_id, min_encoder, max_encoder);
    config_print_one_motor(motor_id);

    return p - args;
}

int command_config_robot_id(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;

    size_t nbytes = parse_whitespace(p, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes == 0) {
        log_write(F("Maintaining robot ID as '"));
        log_write((const __FlashStringHelper *)config_robot_name_by_id[config.robot_id]);
        log_writeln(F("'."));
        return p - args;
    }

    config_robot_id_t robot_id = (config_robot_id_t)-1;

    nbytes = parse_int(p, args_nbytes, (int *)(&robot_id));
    args_nbytes -= nbytes;
    p += nbytes;

    if ((robot_id < 0) || (robot_id > CONFIG_ROBOT_ID_COUNT)) {
        log_writeln(F("ERROR: Invalid robot ID."));
        return 0;
    }

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_write(F("Setting robot ID to '"));
    log_write((const __FlashStringHelper *)config_robot_name_by_id[robot_id]);
    log_writeln(F("'."));

    config_set_robot_id(robot_id);

    return p - args;
}

int command_config_robot_name(char *args, size_t args_nbytes)  // TODO: should these return a size_t?
{
    assert(args);

    char *p = args;

    size_t nbytes = parse_whitespace(p, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes == 0) {
        log_writeln(F("Maintaining robot name '%s'."), config.robot_name);
        return p - args;
    }

    char robot_name[CONFIG_ROBOT_NAME_NBYTES];

    nbytes = parse_string(p, args_nbytes, robot_name, CONFIG_ROBOT_NAME_NBYTES);
    if (nbytes == 0)
        return -1;
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_writeln(F("Setting robot name to '%s'."), robot_name);

    config_set_robot_name(robot_name);

    return p - args;
}

int command_config_robot_serial(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;

    size_t nbytes = parse_whitespace(p, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes == 0) {
        log_writeln(F("Maintaining robot serial '%s'."), config.robot_serial);
        return p - args;
    }

    char robot_serial[CONFIG_ROBOT_SERIAL_NBYTES];

    nbytes = parse_string(p, args_nbytes, robot_serial, CONFIG_ROBOT_SERIAL_NBYTES);
    if (nbytes == 0)
        return -1;
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_writeln(F("Setting robot serial to '%s'."), robot_serial);

    config_set_robot_serial(robot_serial);

    return p - args;
}

int command_config_stall_current_threshold(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;
    motor_id_t motor_id = MOTOR_ID_A;
    size_t nbytes = parse_whitespace(p, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes == 0) {
        log_writeln(F("Maintaining stall current thresholds at current values."));
        return p - args;
    }

    nbytes = parse_motor_id(p, args_nbytes, &motor_id);
    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    int stall_current_threshold = config.motor[motor_id].stall_current_threshold;

    nbytes = parse_int(p, args_nbytes, (int *)(&stall_current_threshold));
    if (nbytes == 0)
        return -1;

    args_nbytes -= nbytes;
    p += nbytes;

    if ((stall_current_threshold < 0) || (stall_current_threshold > 255)) {
        log_writeln(F("ERROR: Invalid stall current threshold. Please enter a value between 0-255, where 0 disables."));
        return -1;
    }

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_writeln(F("Setting motor %c stall current threshold to %d."), 'A' + motor_id, stall_current_threshold);

    config_set_motor_stall_current_threshold(motor_id, stall_current_threshold);
    config_print_one_motor(motor_id);

    return p - args;
}

int command_config_write(char *args, size_t args_nbytes)
{
    assert(args);

    // Confirm arguments are empty.
    size_t nbytes = parse_whitespace(args, args_nbytes);

    if (nbytes != args_nbytes)
        return -1;

    assert(config_write());
    log_writeln(F("%d bytes of configuration data written to Arduino EEPROM."), sizeof(config_t));
    return 0;
}

int command_reboot(char *args, size_t args_nbytes)
{
    assert(args);

    int entry_num = -1;
    char *reboot_table[] = { "reboot" };

    size_t nbytes = parse_string_in_table(args, args_nbytes, reboot_table, 1, &entry_num);

    args += nbytes;
    args_nbytes -= nbytes;
    if ((args_nbytes > 0) || (entry_num != 0))
        return -1;

    motor_disable_all();
    log_writeln(F("Rebooting."));
    hardware_reboot();  // TODO: Test that this works with and without whitespace after REBOOT.

    return 0;
}

static int calibrate_home_and_or_limits(char *args, size_t args_nbytes, bool calibrate_limits)
{
    assert(args);

    int motor_ids_mask = 0;
    char *p = args;
    bool cancel = false;
    int max_velocity_percent = 50;

    // Read.

    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);  // parse_motor_ids will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (motor_ids_mask == -1)
        return nbytes;

    if (motor_ids_mask == 0)
        motor_ids_mask = motor_get_enabled_mask();  // If user didn't specify, select all enabled motors.

    if (motor_ids_mask != 0) {
        nbytes = parse_int(p, args_nbytes, &max_velocity_percent);
        args_nbytes -= nbytes;
        p += nbytes;
    }

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    // Validate.

    if ((max_velocity_percent < 0) || (max_velocity_percent > 100)) {
        log_writeln(F("  Max velocity percent must be between 0 and 100."));
        cancel = true;
    }

    if (motor_ids_mask == 0) {
        log_writeln(F("  No motors selected."));
        cancel = true;
    }

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (((1 << i) & motor_ids_mask) && !motor_get_enabled(i)) {
            log_writeln(F("  Motor %c not enabled."), 'A' + i);
            cancel = true;
        }
    }

    if (!cancel && sm_get_state().run != sm_motors_on_execute) {
        log_writeln(F("  Motors can not be actuated in the current state."));
        cancel = true;
    }

    if (cancel) {
        log_writeln(F("Canceling."));
        return p - args;
    }

    // Execute.

    if (calibrate_limits)
        log_write(F("Calibrating home switch position and encoder limits for motor(s) "));
    else
        log_write(F("Calibrating home switch position for motor(s) "));

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (motor_ids_mask & (1 << i))
            log_write(F("%c"), 'A' + i);
    }
    log_writeln(F(" at %d%% of maximum velocity."), max_velocity_percent);

    if (calibrate_limits)
        calibrate_home_switch_and_limits(motor_ids_mask, max_velocity_percent);
    else
        calibrate_home_switch(motor_ids_mask, max_velocity_percent);

    return p - args;
}

int command_calibrate_home_and_limits(char *args, size_t args_nbytes)
{
    return calibrate_home_and_or_limits(args, args_nbytes, true);
}

int command_calibrate_home(char *args, size_t args_nbytes)
{
    return calibrate_home_and_or_limits(args, args_nbytes, false);
}

int command_set_enabled_motors(char *args, size_t args_nbytes)
{
    assert(args);

    int motor_ids_mask = 0;
    char *p = args;

    // Read.

    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);  // parse_motor_ids will emit message if error.

    if (motor_ids_mask == -1)
        return nbytes;                 // parse_motors_ids prints an error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    // Validate.

    if ((sm_get_state().run != sm_motors_off_execute) &&
        (sm_get_state().run != sm_motors_on_execute)) {
        log_writeln(F("ERROR: Motors can not be enabled in the current state."));
        motor_disable_all();
        return nbytes;
    }

    // Execute.

    sm_set_enabled_motors_mask(motor_ids_mask);

    if (motor_ids_mask == 0)
        sm_set_next_state(sm_state_motors_off_enter);
    else if (motor_ids_mask <= 0x3f)
        sm_set_next_state(sm_state_motors_on_enter);

    return p - args;
}

static void go_home_break_handler(void)
{
    log_writeln(F("Break detected. Stopping motors."));

    motor_disable_all();
    sm_set_next_state(exit_to_state);
}

static int go_home_motor_ids_mask = 0;

static void go_home(void)
{
    bool all_home = true;
    static long previous_status_time_millis = 0;
    long current_time_millis = millis();
    // TODO: Signal failure if motors stall.
    status_t status;

    if (current_time_millis - previous_status_time_millis > 250) {
        gather_status(&status);
        print_status(&status);
        log_writeln();
        previous_status_time_millis = millis();
    }

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        bool selected = ((go_home_motor_ids_mask & (1 << i)) != 0);
        if (!selected)
            continue;

        if (config.motor[i].min_encoder > 0) {
            log_writeln(F("Motor %c min encoder limit %d > 0. Skipping."), 'A' + i, config.motor[i].min_encoder);
            continue;
        }

        if (motor_get_target_encoder((motor_id_t)i) != 0)
            motor_set_target_encoder((motor_id_t)i, 0);
        if (motor_get_encoder((motor_id_t)i) != 0)
            all_home = false;
    }

    if (all_home) {
        for (int i = 0; i < MOTOR_ID_COUNT; i++) {
            bool selected = ((go_home_motor_ids_mask & (1 << i)) != 0);
            if (selected && (!motor_home_is_pressed_debounced((motor_id_t)i)))
                log_writeln(F("WARNING: Motor %c arrived home, but home switch not triggered."), 'A' + i);
        }

        log_writeln(F("Go home completed."));
        sm_set_next_state(exit_to_state);
    }
}

int command_go_home_or_open_gripper(char *args, size_t args_nbytes)
{
    assert(args);
    go_home_motor_ids_mask = 0;
    char *p = args;
    bool cancel = false;

    // Read.

    size_t nbytes = parse_motor_ids(p, args_nbytes, &go_home_motor_ids_mask);

    args_nbytes -= nbytes;
    p += nbytes;

    if (go_home_motor_ids_mask == -1)
        return nbytes;                 // parse_motors_ids prints an error.

    if (go_home_motor_ids_mask == 0)
        go_home_motor_ids_mask = motor_get_enabled_mask();  // If user didn't specify, select all enabled motors.

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    // Validate.

    if (go_home_motor_ids_mask == 0) {
        log_writeln(F("  No motors selected."));
        cancel = true;
    }

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (((1 << i) & go_home_motor_ids_mask) && !motor_get_enabled(i)) {
            log_writeln(F("  Motor %c not enabled."), 'A' + i);
            cancel = true;
        }
    }

    if (!cancel && sm_get_state().run != sm_motors_on_execute) {
        log_writeln(F("  Motors can not be actuated in the current state."));
        cancel = true;
    }

    if (cancel) {
        log_writeln(F("Canceling."));
        return p - args;
    }

    // Execute.

    exit_to_state = sm_get_state();
    assert(exit_to_state.run != go_home);

    static const char state_go_home_name[] PROGMEM = "go home";
    sm_state_t s = { .run = go_home, .break_handler = go_home_break_handler, .process_break_only = true, .name = state_go_home_name };

    sm_set_next_state(s);

    return p - args;
}

int command_print_motor_status(char *args, size_t args_nbytes)
{
    assert(args);

    int old_motor_ids_mask = 0;
    int motor_ids_mask = 0;
    char *p = args;
    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);

    if ((motor_ids_mask == -1) || (args_nbytes != nbytes))
        return nbytes;                 // parse_motors_ids prints an error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (motor_ids_mask == 0)
        motor_ids_mask = MOTOR_IDS_MASK;

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if ((motor_ids_mask & (1 << i)) == 0)
            continue;

        motor_dump(i);
    }

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    return p - args;
}

int command_set_motor_angle(char *args, size_t args_nbytes)
{
    assert(args);

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    bool cancel = false;

    sm_set_display_mode(SM_DISPLAY_MODE_ANGLE);

    // Read.

    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    float angle = motor_get_angle(motor_id);

    nbytes = parse_motor_position(p, args_nbytes, &angle);
    if (nbytes == 0)
        return -1;                     // parse_motor_position will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    // Validate.

    if (!motor_get_enabled(motor_id)) {
        log_writeln(F("  Motor %c not enabled."), 'A' + motor_id);
        cancel = true;
    }

    if (config.motor[motor_id].encoders_per_degree == 0) {
        log_writeln(F("  Motor %c angles/degree not configured."), 'A' + motor_id);
        cancel = true;
    }

    if (!cancel && sm_get_state().run != sm_motors_on_execute) {
        log_writeln(F("  Motors can not be actuated in the current state."));
        cancel = true;
    }

    if (cancel) {
        log_writeln(F("Canceling."));
        return p - args;
    }

    // Execute.

    float min_angle = config_motor_encoders_to_angle(motor_id, config.motor[motor_id].min_encoder);
    float max_angle = config_motor_encoders_to_angle(motor_id, config.motor[motor_id].max_encoder);

    if ((angle < min_angle) || (angle > max_angle)) {
        char str1[15] = {};
        char str2[15] = {};
        char str3[15] = {};

        dtostrf(angle, 3, 2, str1);
        dtostrf(min_angle, 3, 2, str2);
        dtostrf(max_angle, 3, 2, str3);

        log_writeln(F("Clamping motor angle %s to [%s, %s]"), str1, str2, str3);
        angle = fmax(min_angle, fmin(max_angle, angle));
    }

    char angle_str[15] = {};

    dtostrf(angle, 3, 2, angle_str);
    log_writeln(F("Move Motor %c to an angle of %s degrees."), 'A' + motor_id, angle_str);
    motor_set_target_angle(motor_id, angle);

    return p - args;
}

int command_set_motor_encoder(char *args, size_t args_nbytes)
{
    assert(args);

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    bool cancel = false;

    sm_set_display_mode(SM_DISPLAY_MODE_ENCODER);

    // Read.

    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    float encoder = motor_get_encoder(motor_id);

    nbytes = parse_motor_position(p, args_nbytes, &encoder);
    if (nbytes == 0)
        return -1;

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    // Validate.

    if (!motor_get_enabled(motor_id)) {
        log_writeln(F("  Motor %c not enabled."), 'A' + motor_id);
        cancel = true;
    }

    if (!cancel && sm_get_state().run != sm_motors_on_execute) {
        log_writeln(F("  Motors can not be actuated in the current state."));
        cancel = true;
    }

    if (cancel) {
        log_writeln(F("Canceling."));
        return p - args;
    }

    // Execute.

    int min_encoder = config.motor[motor_id].min_encoder;
    int max_encoder = config.motor[motor_id].max_encoder;

    if ((encoder < min_encoder) || (encoder > max_encoder)) {
        log_writeln(F("Clamping motor encoder %d to [%d, %d]"), (int)encoder, min_encoder, max_encoder);
        encoder = max(min_encoder, min(max_encoder, encoder));
    }

    char encoder_str[15] = {};

    dtostrf(encoder, 3, 2, encoder_str);
    log_writeln(F("Move Motor %c to encoder %s."), 'A' + motor_id, encoder_str);
    motor_set_target_encoder(motor_id, encoder);

    return p - args;
}

int command_set_motor_percent(char *args, size_t args_nbytes)
{
    assert(args);

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    bool cancel = false;

    sm_set_display_mode(SM_DISPLAY_MODE_PERCENT);

    // Read.

    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    float percent = motor_get_percent(motor_id);

    nbytes = parse_motor_position(p, args_nbytes, &percent);
    if (nbytes == 0)
        return -1;                     // parse_motor_position will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    // Validate.

    if (!motor_get_enabled(motor_id)) {
        log_writeln(F("  Motor %c not enabled."), 'A' + motor_id);
        cancel = true;
    }

    if (!cancel && sm_get_state().run != sm_motors_on_execute) {
        log_writeln(F("  Motors can not be actuated in the current state."));
        cancel = true;
    }

    if ((config.motor[motor_id].min_encoder == INT_MIN) || (config.motor[motor_id].max_encoder == INT_MAX)) {
        log_writeln(F("  Motor percent cannot be set until both min and max encoders have been calibrated."));
        cancel = true;
    }

    if (cancel) {
        log_writeln(F("Canceling."));
        return p - args;
    }

    // Execute.

    if ((percent < 0.0) || (percent > 100.0)) {
        log_writeln(F("Clamping motor position %d%% to [0.0, 100.0]"), (int)percent);
        percent = fmax(0.0, fmin(100.0, percent));
    }

    char percent_str[15] = {};

    dtostrf(percent, 3, 2, percent_str);
    log_writeln(F("Move Motor %c to %s%%."), 'A' + motor_id, percent_str);
    motor_set_target_percent(motor_id, percent);

    return p - args;
}

int command_set_gpio_pin_output(char *args, size_t args_nbytes)
{
    assert(args);

    int gpio_pin = 0;
    bool is_high = false;
    bool cancel = false;
    char *p = args;

    // Read.

    size_t nbytes = parse_int(p, args_nbytes, &gpio_pin);

    if (nbytes <= 0)
        return -1;
    p += nbytes;
    args_nbytes -= nbytes;

    nbytes = parse_bool(p, args_nbytes, &is_high);
    if (nbytes <= 0)
        return -1;
    p += nbytes;
    args_nbytes -= nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    // Validate.

    if ((gpio_pin < 0) || (gpio_pin >= HARDWARE_GPIO_PIN_COUNT)) {
        log_writeln(F("Invalid GPIO pin number %d. GPIO pin number should be in the range [0, %d]."), gpio_pin, HARDWARE_GPIO_PIN_COUNT - 1);
        cancel = true;
    }

    if (hardware_get_gpio_pin_mode(gpio_pin) != HARDWARE_GPIO_PIN_MODE_OUTPUT) {
        log_writeln(F("GPIO pin %d is not configured for output."), gpio_pin);
        cancel = true;
    }

    if (cancel) {
        log_writeln(F("Canceling."));
        return p - args;
    }

    // Execute.

    hardware_set_gpio_pin_output(gpio_pin, is_high);

    return p - args;
}

static void poll_gpio_pins_break_handler(void)
{
    log_writeln(F("Break detected. Stopping polling of GPIO pins."));

    sm_set_next_state(exit_to_state);

    for (int i = 0; i < 8; i++) {  // TODO: Fix hard-coded 8.
        poll_pins_prev_val[i] = false;
    }
}

static void poll_gpio_inputs(void)
{
    for (int i = 0; i < HARDWARE_GPIO_PIN_COUNT; i++) {
        if (hardware_get_gpio_pin_mode(i) == HARDWARE_GPIO_PIN_MODE_OUTPUT)
            continue;

        bool val = hardware_get_gpio_pin_pressed(i);
        if (val != poll_pins_prev_val[i]) {
            poll_pins_prev_val[i] = val;
            log_writeln(F("GPIO index %d is %s"), i, val ? "button pressed" : "button not pressed");
        }
    }
}

int command_poll_gpio_pin_inputs(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;
    size_t nbytes = parse_whitespace(p, args_nbytes);

    if (nbytes != args_nbytes)
        return -1;

    log_writeln(F("Attach device(s) to IO pins and test. Output will print here when polarity changes. Press <CTRL+C> when done."));

    exit_to_state = sm_get_state();

    static const char state_poll_gpio_name[] PROGMEM = "poll gpio";
    const sm_state_t s = { .run = poll_gpio_inputs, .break_handler = poll_gpio_pins_break_handler, .process_break_only = true, .name = state_poll_gpio_name };

    sm_set_next_state(s);

    return p - args;
}

int command_run_test_sequence(char *args, size_t args_nbytes)
{
    assert(args);

    // TODO: Implement command_run_test_sequence().
    assert(false);

    return -1;
}

int command_test_motors(char *args, size_t args_nbytes)
{
    assert(args);

    int motor_ids_mask = 0;
    char *p = args;
    bool cancel = false;

    // Read.

    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);

    if ((motor_ids_mask == -1) || (args_nbytes != nbytes))
        return nbytes;                 // parse_motors_ids prints an error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    if (motor_ids_mask == 0)
        motor_ids_mask = motor_get_enabled_mask();

    // Validate.

    if (motor_ids_mask == 0) {
        log_writeln(F("  No motors to test."));
        cancel = true;
    }

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (((1 << i) & motor_ids_mask) && !motor_get_enabled(i)) {
            log_writeln(F("  Motor %c not enabled."), 'A' + i);
            cancel = true;
        }
    }

    if (!cancel && sm_get_state().run != sm_motors_on_execute) {
        log_writeln(F("  Motors can not be actuated in the current state."));
        cancel = true;
    }

    if (cancel) {
        log_writeln(F("Canceling."));
        return p - args;
    }

    // Execute.

    motor_test_mask(motor_ids_mask, sm_get_state());

    return nbytes;
}

int command_close_gripper(char *args, size_t args_nbytes)
{
    assert(args);
    int old_motor_ids_mask = 0;
    int motor_ids_mask = 0;
    char *p = args;
    bool cancel = false;

    // Read.

    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);

    if (motor_ids_mask == -1)
        return nbytes;                 // parse_motors_ids prints an error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    if (motor_ids_mask == 0)
        motor_ids_mask = motor_get_enabled_mask();

    // Validate.

    if (motor_ids_mask == 0) {
        log_writeln(F("  No motors to actuate."));
        cancel = true;
    }

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (((1 << i) & motor_ids_mask) && !motor_get_enabled(i)) {
            log_writeln(F("  Motor %c not enabled."), 'A' + i);
            cancel = true;
        }
    }

    if (!cancel && sm_get_state().run != sm_motors_on_execute) {
        log_writeln(F("  Motors can not be actuated in the current state."));
        cancel = true;
    }

    if (cancel) {
        log_writeln(F("Canceling."));
        return p - args;
    }

    // Execute.

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if ((motor_ids_mask & (1 << i)) == 0)
            continue;

        if (!config.motor[i].is_gripper) {
            log_writeln(F("Motor %c is not a gripper. Skipping."), 'A' + i);
            continue;
        }

        log_writeln(F("Move Motor %c to encoder %d."), 'A' + i, config.motor[i].gripper_close_encoder);
        motor_set_target_encoder(i, config.motor[i].gripper_close_encoder);
    }

    return nbytes;
}

int command_print_software_version(char *args, size_t args_nbytes)
{
    assert(args);
    char *p = args;

    if (args_nbytes >= command_args_max_nbytes)
        return -1;

    size_t nbytes = parse_whitespace(p, args_nbytes);

    if (nbytes != args_nbytes)
        return -1;

    char version_number_string[10] = {};

    dtostrf(software_version, 3, 2, version_number_string);
    log_writeln(F("Version: %s (%s %s)."), version_number_string, __DATE__, __TIME__);

    return nbytes;
}

int command_waypoint_run(char *args, size_t args_nbytes)
{
    assert(args);
    char *p = args;
    int start_step = 0;
    int count = -1;

    if (args_nbytes > 0) {
        // Parse optional start-step integer parameter.
        size_t nbytes = parse_int(p, args_nbytes, &start_step);
        args_nbytes -= nbytes;
        p += nbytes;

        if ((start_step < -1) || (start_step >= waypoint_get_max_count())) {
            log_writeln(F("ERROR: Step %d is outside of range [-1, %d]."), start_step, waypoint_get_max_count() - 1);
            return -1;
        }

        if (args_nbytes > 0) {
            // Parse optional count parameter.
            nbytes = parse_int(p, args_nbytes, &count);
            args_nbytes -= nbytes;
            p += nbytes;

            if (count <= 0) {
                log_writeln(F("ERROR: Invalid count %d. Count should be > 0."), count);
                return -1;
            }
        }
    }

    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_writeln(F("Running waypoint sequence. Type <CTRL+C> to stop."));

    waypoint_run(start_step, count);

    return p - args;
}

static size_t parse_step_and_waypoint(char *args, size_t args_nbytes, int *step, waypoint_t *waypoint)
{
    assert(args);
    assert(step);
    assert(waypoint);

    char *p = args;

    *step = -1;
    memset(waypoint, 0, sizeof(waypoint_t));

    size_t nbytes = parse_int(p, args_nbytes, step);

    if (nbytes <= 0)
        return 0;
    if ((*step < 0) || (*step >= waypoint_get_max_count()))
        return 0;
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_waypoint(p, args_nbytes, waypoint);
    if (nbytes <= 0)
        return nbytes;

    args_nbytes -= nbytes;
    p += nbytes;
    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    return p - args;
}

int command_waypoint_set(char *args, size_t args_nbytes)
{
    assert(args);
    char *p = args;
    int step = -1;
    waypoint_t waypoint = { 0 };

    size_t nbytes = parse_step_and_waypoint(p, args_nbytes, &step, &waypoint);

    if (nbytes <= 0)
        return nbytes;

    args_nbytes -= nbytes;
    p += nbytes;
    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_writeln(F("Setting waypoint %d."), step);
    waypoint_set(step, waypoint);
    waypoint_print(step);

    return nbytes;
}

int command_waypoint_insert_before(char *args, size_t args_nbytes)
{
    assert(args);
    char *p = args;

    int step = -1;
    waypoint_t waypoint = { 0 };

    size_t nbytes = parse_step_and_waypoint(p, args_nbytes, &step, &waypoint);

    if (nbytes <= 0)
        return nbytes;

    args_nbytes -= nbytes;
    p += nbytes;
    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_writeln(F("Inserting waypoint before step %d."), step);
    waypoint_insert_before(step, waypoint);
    waypoint_print(step);

    return nbytes;
}

int command_waypoint_append(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;
    waypoint_t waypoint = { 0 };

    size_t nbytes = parse_waypoint(p, args_nbytes, &waypoint);

    if (nbytes <= 0)
        return nbytes;                 // parse_waypoint emits error message.

    args_nbytes -= nbytes;
    p += nbytes;
    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_writeln(F("Appending waypoint to end of list."));
    int step = waypoint_append(waypoint);

    if (step < 0) {
        log_writeln(F("ERROR: Waypoint append failed."));
        return -1;
    }

    waypoint_print(step);

    return nbytes;
}

int command_waypoint_delete(char *args, size_t args_nbytes)
{
    assert(args);

    int step = 0;
    char *p = args;

    size_t nbytes = parse_int(p, args_nbytes, &step);

    args_nbytes -= nbytes;
    p += nbytes;
    if (args_nbytes > 0)
        return -1;                     // Extraneous input.

    log_writeln(F("Deleting waypoint %d."), step);

    waypoint_delete(step);

    return p - args;
}

int command_waypoint_print(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;
    size_t nbytes = parse_whitespace(p, args_nbytes);

    if (args_nbytes != nbytes)
        return -1;

    waypoint_print_all_used();

    return p - args;
}

int command_factory_reset(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;

    int entry_num = -1;
    char *reset_table[] = { "reset" };

    size_t nbytes = parse_string_in_table(args, args_nbytes, reset_table, 1, &entry_num);

    args_nbytes -= nbytes;
    p += nbytes;
    if ((args_nbytes > 0) || (entry_num != 0))
        return -1;

    hardware_factory_reset();
    hardware_reboot();

    return 0;
}

int command_emergency_stop(char *args, size_t args_nbytes)
{
    assert(args);

    // Confirm arguments are empty.
    size_t nbytes = parse_whitespace(args, args_nbytes);

    if (args_nbytes != nbytes)
        return -1;

    motor_disable_all();
    hardware_halt();

    return 0;
}

int command_print_help(char *args, size_t args_nbytes)
{
    assert(args);

    // Confirm arguments are empty.
    size_t nbytes = parse_whitespace(args, args_nbytes);

    if (nbytes != args_nbytes)
        return -1;

    menu_help();
    return 0;
}
