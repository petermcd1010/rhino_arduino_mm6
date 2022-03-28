#pragma once

/*
 * Declarations for command processing functions.
 */

#include <Arduino.h>

// Returns number of bytes processed on success, or -1 on error.
int command_print_config(char *pargs, size_t args_nbytes);
int command_config_robot_id(char *pargs, size_t args_nbytes);
int command_config_robot_serial(char *pargs, size_t args_nbytes);
int command_config_robot_name(char *pargs, size_t args_nbytes);
int command_config_write(char *pargs, size_t args_nbytes);
int command_run_calibration(char *pargs, size_t args_nbytes);
int command_pid_mode(char *pargs, size_t args_nbytes);
int command_emergency_stop(char *pargs, size_t args_nbytes);
int command_set_gripper_position(char *pargs, size_t args_nbytes);
int command_set_home_position(char *pargs, size_t args_nbytes);
int command_print_motor_status(char *pargs, size_t args_nbytes);
int command_set_motor_angle(char *pargs, size_t args_nbytes);
int command_set_motor_encoder(char *pargs, size_t args_nbytes);
int command_run_test_sequence(char *pargs, size_t args_nbytes);
int command_start_stop_motors(char *pargs, size_t args_nbytes);
int command_test_motors(char *pargs, size_t args_nbytes);
int command_print_software_version(char *pargs, size_t args_nbytes);
int command_waypoint(char *pargs, size_t args_nbytes);
int command_factory_reset(char *pargs, size_t args_nbytes);
int command_reboot(char *pargs, size_t args_nbytes);
int command_print_help(char *pargs, size_t args_nbytes);
