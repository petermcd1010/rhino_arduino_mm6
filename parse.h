#pragma once
/*
 * Declarations for input parsing functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>
#include "waypoint.h"  // waypoint_t.
#include "motor.h"  // motor_id_t.

size_t parse_whitespace(char *buf, size_t buf_nbytes);
size_t parse_bool(char *buf, size_t buf_nbytes, bool *out_bool);
size_t parse_char(char *buf, size_t buf_nbytes, char *out_char);
size_t parse_int(char *buf, size_t buf_nbytes, int *out_int);
size_t parse_float(char *buf, size_t buf_nbytes, float *out_float);
size_t parse_string(char *buf, size_t buf_nbytes, char *out_string, size_t out_string_nbytes);
size_t parse_string_in_table(char *buf, size_t buf_nbytes, char *table[], int ntable_entries, int *out_entry_num);
size_t parse_motor_ids(char *buf, size_t buf_nbytes, int *out_mask);  // *out_mask unchanged if buffer is emapty or only whitespace. *out_mask == -1 on error.
size_t parse_motor_id(char *buf, size_t buf_nbytes, motor_id_t *out_motor_id);
size_t parse_motor_angle_or_encoder(char *buf, size_t buf_nbytes, float *out_value);
size_t parse_waypoint(char *buf, size_t buf_nbytes, waypoint_t *out_waypoint);
bool parse_test();
