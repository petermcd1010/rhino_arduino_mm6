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
        log_writeln(F("Maintaining robot ID as '%s'."), config_robot_name_by_id[config.robot_id]);
        return p - args;
    }

    config_robot_id_t robot_id = CONFIG_ROBOT_ID_FIRST - 1;

    nbytes = parse_int(p, args_nbytes, (int *)(&robot_id));
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_whitespace(p, args_nbytes);
    args_nbytes -= nbytes;
    p += nbytes;

    if ((robot_id < CONFIG_ROBOT_ID_FIRST) || (robot_id > CONFIG_ROBOT_ID_LAST)) {
        log_writeln(F("ERROR: Invalid robot ID."));
        goto error;
    }

    if (args_nbytes > 0)
        goto error;

    log_writeln(F("Setting robot ID to '%s'."), config_robot_name_by_id[robot_id]);

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

    size_t nbytes = parse_whitespace(args, args_nbytes);

    args += nbytes;
    args_nbytes -= nbytes;
    if (args_nbytes == 0)
        return -1;

    int entry_num = -1;
    char *reboot_table[] = { "REBOOT" };

    nbytes = parse_string_in_table(args, args_nbytes, reboot_table, 1, &entry_num);
    if (entry_num != 0)
        return -1;
    args += nbytes;
    args_nbytes -= nbytes;

    nbytes = parse_whitespace(args, args_nbytes);
    args += nbytes;
    args_nbytes -= nbytes;

    if (nbytes != args_nbytes)
        return -1;

    motor_disable_all();
    log_writeln(F("Rebooting."));
    hardware_reboot();  // TODO: Test that this works with and without whitespace after REBOOT.

    return 0;
}

int command_calibrate_motors(char *args, size_t args_nbytes)
{
    assert(args);

    int motor_ids_mask = motor_get_enabled_mask();
    char *p = args;
    size_t nbytes = parse_motor_ids(p, args_nbytes, &motor_ids_mask);  // parse_motor_ids will emit message if error.

    if ((motor_ids_mask == -1) || (args_nbytes != nbytes))
        return nbytes;                 // parse_motors_ids prints an error.

    log_writeln(F("Calibrating motors %d"), motor_ids_mask);

    calibrate_init(motor_ids_mask);
    sm_set_next_state(calibrate_begin);

error:
    return nbytes;
}

int command_pid_mode(char *args, size_t args_nbytes)
{
    // TODO: Implement command_pid_mode().
    assert(false);

    return -1;
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

    if ((sm_get_state() != sm_motors_off_execute) &&
        (sm_get_state() != sm_motors_on_execute)) {
        log_writeln(F("ERROR: Motors can not be turned on or off in the current state."));
        goto error;
    }

    sm_set_enabled_motors_mask(motor_ids_mask);

    if (motor_ids_mask == 0)
        sm_set_next_state(sm_motors_off_enter);
    else if (motor_ids_mask <= 0x3f)
        sm_set_next_state(sm_motors_on_enter);

error:
    return nbytes;
}

int command_set_gripper_position(char *args, size_t args_nbytes)
{
    // TODO: Implement command_set_gripper_position.
    assert(false);

    return -1;
}

int command_set_home_position(char *args, size_t args_nbytes)
{
    assert(args);
    size_t nbytes = parse_whitespace(args, args_nbytes);

    if (nbytes != args_nbytes)
        return -1;

    LOG_ERROR(F("TODO"));

    // motor_exec_all(motor_set_position_to_home);

    return nbytes;
}

int command_print_motor_status(char *args, size_t args_nbytes)
{
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

    for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
        if ((motor_ids_mask & (1 << i)) == 0)
            continue;

        char angle_str[15] = {};
        dtostrf(motor_get_angle((motor_id_t)i), 3, 2, angle_str);
        log_writeln(F("%c%s: home:%d sta:%d enc:%d tar:%d err:%d spd:%d PWM:%d cur:%d hs:%d,%d,%d,%d->%d angle:%s"),
                    'A' + i,
                    ((motor_get_enabled_mask() & (1 << i)) == 0) ? " [not enabled]" : "",
                    motor_state[i].switch_previously_triggered,  // home.
                    motor_state[i].progress,  // sta. Report whether or not the Motor has reached the target location.
                    /* motor_get_encoder(iMotor), */  // pos.
                    motor_get_encoder(i) * motor_state[i].logic,  // enc.
                    motor_state[i].target_encoder * motor_state[i].logic,  // tar.
                    motor_state[i].pid_perror * motor_state[i].logic,  // err.
                    motor_state[i].speed * motor_state[i].logic,  // spd.
                    /* motor_state[i].target_speed, */  // tspd.
                    motor_state[i].pwm,
                    motor_state[i].current_draw,  // cur.
                    motor_state[i].switch_reverse_off,  // hs.
                    motor_state[i].switch_forward_on,  // hs.
                    motor_state[i].switch_reverse_on,  // hs.
                    motor_state[i].switch_forward_off,  // hs.
                    (motor_state[i].switch_forward_off + motor_state[i].switch_reverse_on + motor_state[i].switch_forward_on + motor_state[i].switch_reverse_off) / 4,  // hs.
                    angle_str);
    }

    return nbytes;
}

int command_set_motor_angle(char *args, size_t args_nbytes)
{
    assert(args);

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        goto error;                    // parse_motor_id will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_whitespace(p, args_nbytes);
    args_nbytes -= nbytes;
    p += nbytes;

    float angle = motor_get_angle(motor_id);

    nbytes = parse_motor_angle_or_encoder(p, args_nbytes, &angle);
    if (nbytes == 0)
        goto error;                    // parse_motor_angle_or_encoder will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_whitespace(p, args_nbytes);
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        goto error;

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
    LOG_ERROR(F(""));
    return -1;
}

int command_set_motor_encoder(char *args, size_t args_nbytes)
{
    assert(args);

    motor_id_t motor_id = MOTOR_ID_A;
    char *p = args;
    size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);

    if (nbytes == 0)
        goto error;                    // parse_motor_id will emit message if error.
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_whitespace(p, args_nbytes);
    args_nbytes -= nbytes;
    p += nbytes;

    float encoder = motor_get_encoder(motor_id);

    nbytes = parse_motor_angle_or_encoder(p, args_nbytes, &encoder);
    if (nbytes == 0)
        return -1;
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_whitespace(p, args_nbytes);
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        goto error;

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

error:
    LOG_ERROR(F(""));
    return -1;
}

int command_run_test_sequence(char *args, size_t args_nbytes)
{
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

    if ((sm_get_state() != sm_motors_off_execute) &&
        (sm_get_state() != sm_motors_on_execute)) {
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
    if (args_nbytes >= command_args_max_nbytes)
        return -1;

    char *p = args;
    size_t nbytes = parse_whitespace(p, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        return -1;

    char version_number_string[10] = {};

    dtostrf(software_version, 3, 2, version_number_string);
    log_writeln(F("Version: %s (%s %s)."), version_number_string, __DATE__, __TIME__);

    return 0;
}

int command_waypoint_run(char *args, size_t args_nbytes)
{
    // w r [start-step].
    assert(false);
    return -1;
}

int command_waypoint_set(char *args, size_t args_nbytes)
{
    // TODO: Add commands other than move motors to current position.
    // w s [step].
    assert(args);

    static int step = 0;

    char *p = args;

    size_t nbytes = parse_whitespace(p, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0) {
        nbytes = parse_int(p, args_nbytes, &step);
        args_nbytes -= nbytes;
        p += nbytes;
    }

    nbytes = parse_whitespace(p, args_nbytes);
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        goto error;

    log_writeln(F("Setting waypoint %d to current position."), step);

    waypoint_t waypoint = { 0 };

    waypoint.step = step;
    waypoint.command = WAYPOINT_COMMAND_MOVE_AT;
    waypoint.motor.a = motor_get_encoder(MOTOR_ID_A);
    waypoint.motor.b = motor_get_encoder(MOTOR_ID_B);
    waypoint.motor.c = motor_get_encoder(MOTOR_ID_C);
    waypoint.motor.d = motor_get_encoder(MOTOR_ID_D);
    waypoint.motor.e = motor_get_encoder(MOTOR_ID_E);
    waypoint.motor.f = motor_get_encoder(MOTOR_ID_F);

    waypoint_set(step, waypoint);

    step++;  // Advance to next step.

    return p - args;

error:
    return -1;
}

int command_waypoint_insert_before(char *args, size_t args_nbytes)
{
    // w i step command [args].
    assert(false);
    return -1;
}

int command_waypoint_delete(char *args, size_t args_nbytes)
{
    // w d step.
    static int step = 0;

    char *p = args;

    size_t nbytes = parse_whitespace(p, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0) {
        nbytes = parse_int(p, args_nbytes, &step);
        args_nbytes -= nbytes;
        p += nbytes;
    }

    nbytes = parse_whitespace(p, args_nbytes);
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        goto error;

    log_writeln(F("Deleting waypoint %d."), step);

    waypoint_delete(step);

    step++;  // Advance to next step.

    return p - args;

error:
    return -1;
}

int command_waypoint_append(char *args, size_t args_nbytes)
{
    // w a command [args].
    assert(false);
    return -1;
}

int command_waypoint_print(char *args, size_t args_nbytes)
{
    // w p [step [count]]

    char *p = args;
    size_t nbytes = parse_whitespace(p, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;

    for (int i = 0; i < waypoint_get_max_count(); i++) {
        waypoint_t waypoint = waypoint_get(i);
        if (waypoint.step != -1)
            waypoint_print(waypoint);
    }

    return p - args;
}

int command_waypoint_execute_single(char *args, size_t args_nbytes)
{
    // w x step.
    assert(false);
    return -1;
}

int command_factory_reset(char *args, size_t args_nbytes)
{
    assert(args);

    char *p = args;

    size_t nbytes = parse_whitespace(args, args_nbytes);

    args_nbytes -= nbytes;
    p += nbytes;
    if (args_nbytes == 0)
        return -1;

    int entry_num = -1;
    char *reboot_table[] = { "RESET" };

    nbytes = parse_string_in_table(args, args_nbytes, reboot_table, 1, &entry_num);
    args_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_whitespace(p, args_nbytes);
    args_nbytes -= nbytes;
    p += nbytes;

    if (args_nbytes > 0)
        goto error;

    assert(entry_num == 0);
    hardware_factory_reset();
    hardware_reboot();

    return 0;

error:

    return -1;
}

int command_emergency_stop(char *args, size_t args_nbytes)
{
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
