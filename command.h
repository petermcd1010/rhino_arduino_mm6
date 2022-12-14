#pragma once
/*
 * Declarations for command processing functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

// Returns number of bytes processed on success, or -1 on error.
int command_print_config(char *args, size_t args_nbytes);
int command_config_angle_offset(char *args, size_t args_nbytes);
int command_config_boot_mode(char *args, size_t args_nbytes);
int command_config_encoders_per_degree(char *args, size_t args_nbytes);
int command_config_gpio_pin_mode(char *args, size_t args_nbytes);
int command_config_home_encoder(char *args, size_t args_nbytes);
int command_config_invert_motor_orientation(char *args, size_t args_nbytes);
int command_config_min_max_encoders(char *args, size_t args_nbytes);
int command_config_robot_id(char *args, size_t args_nbytes);
int command_config_robot_name(char *args, size_t args_nbytes);
int command_config_robot_serial(char *args, size_t args_nbytes);
int command_config_stall_current_threshold(char *args, size_t args_nbytes);
int command_config_write(char *args, size_t args_nbytes);
int command_reboot(char *args, size_t args_nbytes);
int command_calibrate_home_and_limits(char *args, size_t args_nbytes);
int command_calibrate_home(char *args, size_t args_nbytes);
int command_calibrate_motors(char *args, size_t args_nbytes);
int command_set_enabled_motors(char *args, size_t args_nbytes);
int command_go_home_or_open_gripper(char *args, size_t args_nbytes);
int command_print_motor_status(char *args, size_t args_nbytes);
int command_set_motor_angle(char *args, size_t args_nbytes);
int command_set_motor_encoder(char *args, size_t args_nbytes);
int command_set_motor_percent(char *args, size_t args_nbytes);
int command_set_gpio_pin_output(char *args, size_t args_nbytes);
int command_poll_gpio_pin_inputs(char *args, size_t args_nbytes);
int command_run_test_sequence(char *args, size_t args_nbytes);
int command_test_motors(char *args, size_t args_nbytes);
int command_close_gripper(char *args, size_t args_nbytes);
int command_print_software_version(char *args, size_t args_nbytes);
int command_waypoint_run(char *args, size_t args_nbytes);
int command_waypoint_set(char *args, size_t args_nbytes);
int command_waypoint_insert_before(char *args, size_t args_nbytes);
int command_waypoint_append(char *args, size_t args_nbytes);
int command_waypoint_delete(char *args, size_t args_nbytes);
int command_waypoint_print(char *args, size_t args_nbytes);
int command_factory_reset(char *args, size_t args_nbytes);
int command_emergency_stop(char *args, size_t args_nbytes);
int command_print_help(char *args, size_t args_nbytes);
