/*
 * Implementation for command processing functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <stdlib.h>
#include "calibrate.h"
#include "command.h"
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "menu.h"
#include "motor.h"
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

    config_robot_id_t robot_id = CONFIG_ROBOT_ID_FIRST - 1;

    nbytes = parse_int(p, args_nbytes, (int *)(&robot_id));
    args_nbytes -= nbytes;
    p += nbytes;

    if ((robot_id < CONFIG_ROBOT_ID_FIRST) || (robot_id > CONFIG_ROBOT_ID_LAST)) {
        log_writeln(F("ERROR: Invalid robot ID."));
        return 0;
    }

    if (args_nbytes > 0)
        goto error;

    log_write(F("Setting robot ID to '"));
    log_write((const __FlashStringHelper *)config_robot_name_by_id[robot_id]);
    log_writeln(F("'."));

    config_set_robot_id(robot_id);

    return p - args;

error:
    return -1;
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
        goto error;

    log_writeln(F("Setting robot serial to '%s'."), robot_serial);

    config_set_robot_serial(robot_serial);

    return p - args;

error:
    return -1;
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
        goto error;

    log_writeln(F("Setting robot name to '%s'."), robot_name);

    config_set_robot_name(robot_name);

    return p - args;

error:
    return -1;
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
    char *reboot_table[] = { "REBOOT" };

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

int command_calibrate_home_and_limits(char *args, size_t args_nbytes)
{
    assert(args);

    int motor_ids_mask = 0;
    char *p = args;
    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);  // parse_motor_ids will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (motor_ids_mask == -1)
        return nbytes;

    int max_speed_percent = 75;

    if (motor_ids_mask != 0) {
        nbytes = parse_int(p, args_nbytes, &max_speed_percent);
        args_nbytes -= nbytes;
        p += nbytes;
    }


    if (motor_ids_mask == 0)
        motor_ids_mask = motor_get_enabled_mask();

    if (motor_ids_mask == 0) {
        log_writeln(F("No motors enabled and no motors specified. Skipping calibration."));
        return args - p;
    }

    if ((max_speed_percent < 0) || (max_speed_percent > 100)) {
        log_writeln(F("Max speed percent must be between 0 and 100. Skipping calibration."));
        return args - p;
    }

    log_writeln(F("Calibrating motors %d"), motor_ids_mask);

    calibrate_home_switch_and_limits(motor_ids_mask, max_speed_percent);

    return p - args;
}

int command_calibrate_home(char *args, size_t args_nbytes)
{
    assert(args);

    int motor_ids_mask = 0;
    char *p = args;
    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);  // parse_motor_ids will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (motor_ids_mask == -1)
        return nbytes;

    int max_speed_percent = 75;

    if (motor_ids_mask != 0) {
        nbytes = parse_int(p, args_nbytes, &max_speed_percent);
        args_nbytes -= nbytes;
        p += nbytes;
    }

    if (args_nbytes > 0)
        return -1;

    if (motor_ids_mask == 0)
        motor_ids_mask = motor_get_enabled_mask();

    if (motor_ids_mask == 0) {
        log_writeln(F("No motors enabled and no motors specified. Skipping calibration."));
        return args - p;
    }

    if ((max_speed_percent < 0) || (max_speed_percent > 100)) {
        log_writeln(F("Max speed percent must be between 0 and 100. Skipping calibration."));
        return args - p;
    }

    calibrate_home_switch(motor_ids_mask, max_speed_percent);

    return p - args;
}

int command_set_enabled_motors(char *args, size_t args_nbytes)
{
    assert(args);

    int motor_ids_mask = 0;
    char *p = args;
    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);  // parse_motor_ids will emit message if error.

    if ((motor_ids_mask == -1) || (args_nbytes != nbytes))
        return nbytes;                 // parse_motors_ids prints an error.

    args_nbytes -= nbytes;
    p += nbytes;

    if ((sm_get_state().run != sm_motors_off_execute) &&
        (sm_get_state().run != sm_motors_on_execute)) {
        log_writeln(F("ERROR: Motors can not be turned on or off in the current state."));
        goto error;
    }

    sm_set_enabled_motors_mask(motor_ids_mask);

    if (motor_ids_mask == 0) {
        sm_state_t s = { .run = sm_motors_off_enter, .break_handler = NULL, .name = F("sm_motors_off_enter"), .data = NULL };
        sm_set_next_state(s);
    } else if (motor_ids_mask <= 0x3f) {
        sm_state_t s = { .run = sm_motors_on_enter, .break_handler = NULL, .name = F("sm_motors_on_enter"), .data = NULL };
        sm_set_next_state(s);
    }

error:
    return nbytes;
}

int command_set_gripper_position(char *args, size_t args_nbytes)
{
    // TODO: Implement command_set_gripper_position.
    assert(false);

    return -1;
}

static void go_home_break_handler(void)
{
    log_writeln(F("Break detected. Stopping motors."));

    motor_disable_all();
    sm_set_next_state(exit_to_state);
}

static void go_home(void)
{
    bool all_home = true;

    // TODO: Signal failure if motors get stuck.

    int enabled_motors = motor_get_enabled_mask();

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        bool enabled = ((enabled_motors & (1 << i)) != 0);
        if (enabled) {
            if (motor_get_target_encoder(i) != 0)
                motor_set_target_encoder(i, 0);
            if ((motor_get_encoder(i) != 0) || (!motor_is_home_triggered_debounced(i)))
                all_home = false;
        }
    }

    if (all_home) {
        log_writeln(F("Motors at home position."));
        sm_set_next_state(exit_to_state);
    }
}

int command_go_home(char *args, size_t args_nbytes)
{
    assert(args);

    int motor_ids_mask = 0;
    char *p = args;
    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);  // parse_motor_ids will emit message if error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (motor_ids_mask == -1)
        return nbytes;

    if (motor_ids_mask == 0)
        motor_ids_mask = motor_get_enabled_mask();

    if (motor_ids_mask == 0) {
        log_writeln(F("No motors enabled and no motors specified. Skipping."));
        return args - p;
    }

    exit_to_state = sm_get_state();
    sm_state_t s = { .run = go_home, .break_handler = go_home_break_handler, .name = F("go_home"), .data = NULL };

    sm_set_next_state(s);
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
        motor_ids_mask = 0x3f;

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if ((motor_ids_mask & (1 << i)) == 0)
            continue;

        char angle_str[15] = {};
        dtostrf(motor_get_angle((motor_id_t)i), 3, 2, angle_str);
        log_writeln(F("%c%s: home:%d sta:%d enc:%d tar:%d err:%d spd:%d PWM:%d cur:%d hs:%d,%d,%d,%d->%d angle:%s"),
                    'A' + i,
                    ((motor_get_enabled_mask() & (1 << i)) == 0) ? " [not enabled]" : "",
                    motor_state[i].home_triggered_debounced,  // home switch.
                    motor_state[i].progress,  // sta. Report whether or not the Motor has reached the target location.
                    /* motor_get_encoder(iMotor), */  // pos.
                    motor_get_encoder(i) * motor_state[i].logic,  // enc.
                    motor_state[i].target_encoder * motor_state[i].logic,  // tar.
                    motor_state[i].pid_perror * motor_state[i].logic,  // err.
                    motor_state[i].speed * motor_state[i].logic,  // spd.
                    /* motor_state[i].target_speed, */  // tspd.
                    motor_state[i].pwm,
                    motor_state[i].current_draw,  // cur.
                    motor_state[i].home_reverse_off_encoder,  // hs.
                    motor_state[i].home_forward_on_encoder,  // hs.
                    motor_state[i].home_reverse_on_encoder,  // hs.
                    motor_state[i].home_forward_off_encoder,  // hs.
                    (motor_state[i].home_forward_off_encoder + motor_state[i].home_reverse_on_encoder + motor_state[i].home_forward_on_encoder + motor_state[i].home_reverse_off_encoder) / 4,  // hs.
                    angle_str);
    }

    if (args_nbytes > 0)
        return -1;

    return p - args;
}

int command_set_motor_angle(char *args, size_t args_nbytes)
{
    assert(args);

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    float angle = motor_get_angle(motor_id);

    nbytes = parse_motor_angle_or_encoder(p, args_nbytes, &angle);
    if (nbytes == 0)
        return -1;                     // parse_motor_angle_or_encoder will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;

    char angle_str[15] = {};

    dtostrf(angle, 3, 2, angle_str);

    if (motor_get_enabled(motor_id)) {
        log_writeln(F("Move Motor %c to an angle of %s degrees."), 'A' + motor_id, angle_str);
        motor_set_target_angle(motor_id, angle);
    } else {
        log_writeln(F("ERROR: Motor %c not enabled."), 'A' + motor_id);
        // TODO: error state?
    }
    return p - args;

error:
    return -1;
}

static void poll_pins_break_handler(void)
{
    log_writeln(F("Break detected. Stopping polling of header pins."));

    sm_set_next_state(exit_to_state);

    for (int i = 0; i < 8; i++) {  // TODO: Fix hard-coded 8.
        poll_pins_prev_val[i] = false;
    }
}

static void poll_pins(sm_state_t *state)
{
    assert(state);

    int npins = hardware_get_num_header_pins();

    for (int i = 0; i < npins; i++) {
        bool val = hardware_get_header_pin_pressed(i);
        if (val != poll_pins_prev_val[i]) {
            poll_pins_prev_val[i] = val;
            log_writeln(F("Pin index %d is %s"), i, val ? "button pressed" : "button not pressed");
        }
    }
}

int command_poll_pins(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;
    size_t nbytes = parse_whitespace(p, args_nbytes);

    if (nbytes != args_nbytes)
        return -1;

    log_writeln(F("Attach device(s) to IO pins and test. Output will print here when polarity changes. Press <CTRL+C> when done."));

    exit_to_state = sm_get_state();

    sm_state_t s = { .run = poll_pins, .break_handler = poll_pins_break_handler, .name = F("poll_pins"), .data = NULL };

    sm_set_next_state(s);

    return p - args;
}

int command_set_motor_encoder(char *args, size_t args_nbytes)
{
    assert(args);

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        return -1;                     // parse_motor_id will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    float encoder = motor_get_encoder(motor_id);

    nbytes = parse_motor_angle_or_encoder(p, args_nbytes, &encoder);
    if (nbytes == 0)
        return -1;
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;

    char encoder_str[15] = {};

    dtostrf(encoder, 3, 2, encoder_str);

    if (motor_get_enabled(motor_id)) {
        log_writeln(F("Move Motor %c to encoder %s."), 'A' + motor_id, encoder_str);
        motor_set_target_encoder(motor_id, encoder);
    } else {
        log_writeln(F("ERROR: Motor %c not enabled."), 'A' + motor_id);
        // TODO: error state?
    }

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
    int old_motor_ids_mask = 0;
    int motor_ids_mask = 0;
    char *p = args;
    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);

    if ((motor_ids_mask == -1) || (args_nbytes != nbytes))
        return nbytes;                 // parse_motors_ids prints an error.

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        goto error;

    if ((sm_get_state().run != sm_motors_off_execute) &&
        (sm_get_state().run != sm_motors_on_execute)) {
        log_writeln(F("ERROR: Motors can not be turned on or off in the current state."));
        goto error;
    }

    old_motor_ids_mask = motor_get_enabled_mask();

    if (motor_ids_mask == 0) {
        // No args specified, so test all enabled motors.
        if (old_motor_ids_mask == 0) {
            log_writeln(F("No motors enabled. Skipping test."));
            return nbytes;
        }
        motor_test_enabled();
    } else {
        motor_set_enabled_mask(motor_ids_mask);
        motor_test_enabled();
        motor_set_enabled_mask(old_motor_ids_mask);
    }

    return nbytes;

error:
    return -1;
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
        return -1;

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
        return -1;

    return p - args;
}

int command_waypoint_set(char *args, size_t args_nbytes)
{
    assert(args);
    char *p = args;
    int step = -1;
    waypoint_t waypoint = { 0 };

    size_t nbytes = parse_step_and_waypoint(p, args_nbytes, &step, &waypoint);

    args_nbytes -= nbytes;
    p += nbytes;
    if (args_nbytes > 0)
        return -1;

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

    args_nbytes -= nbytes;
    p += nbytes;
    if (args_nbytes > 0)
        return -1;

    log_writeln(F("Inserting waypoint before step %d."), step);
    waypoint_insert_before(step, waypoint);
    waypoint_print(step);

    return nbytes;
}

int command_waypoint_append(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;
    int step = -1;
    waypoint_t waypoint = { 0 };

    size_t nbytes = parse_waypoint(p, args_nbytes, &waypoint);

    args_nbytes -= nbytes;
    p += nbytes;
    if (args_nbytes > 0)
        return -1;

    log_writeln(F("Appending waypoint to end of list."));
    waypoint_append(waypoint);
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
        goto error;

    log_writeln(F("Deleting waypoint %d."), step);

    waypoint_delete(step);

    return p - args;

error:
    return -1;
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
    char *reset_table[] = { "RESET" };

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
