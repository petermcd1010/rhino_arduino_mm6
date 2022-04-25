#pragma once
/*
 * Declarations for command processing functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

// Returns number of bytes processed on success, or -1 on error.
int command_print_config(char *pargs, size_t args_nbytes);
int command_config_robot_id(char *pargs, size_t args_nbytes);
int command_config_robot_serial(char *pargs, size_t args_nbytes);
int command_config_robot_name(char *pargs, size_t args_nbytes);
int command_config_write(char *pargs, size_t args_nbytes);
int command_reboot(char *pargs, size_t args_nbytes);
int command_calibrate_motors(char *pargs, size_t args_nbytes);
int command_pid_mode(char *pargs, size_t args_nbytes);
int command_set_enabled_motors(char *pargs, size_t args_nbytes);
int command_set_gripper_position(char *pargs, size_t args_nbytes);
int command_set_home_position(char *pargs, size_t args_nbytes);
int command_print_motor_status(char *pargs, size_t args_nbytes);
int command_set_motor_angle(char *pargs, size_t args_nbytes);
int command_set_motor_encoder(char *pargs, size_t args_nbytes);
int command_run_test_sequence(char *pargs, size_t args_nbytes);
int command_test_motors(char *pargs, size_t args_nbytes);
int command_print_software_version(char *pargs, size_t args_nbytes);
int command_waypoint_run(char *pargs, size_t args_nbytes);  // w r [start-step].
int command_waypoint_set(char *pargs, size_t args_nbytes);  // w s step command [args].
int command_waypoint_insert_before(char *pargs, size_t args_nbytes);  // w i step command [args].
int command_waypoint_delete(char *pargs, size_t args_nbytes);  // w d step.
int command_waypoint_append(char *pargs, size_t args_nbytes);  // w a command [args].
int command_waypoint_print(char *pargs, size_t args_nbytes);  // w p [step [count]]
int command_waypoint_execute_single(char *pargs, size_t args_nbytes);  // w x step.
int command_factory_reset(char *pargs, size_t args_nbytes);
int command_emergency_stop(char *pargs, size_t args_nbytes);
int command_print_help(char *pargs, size_t args_nbytes);
