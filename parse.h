#pragma once

/*
 * Declarations for input parsing functions.
 */

#include <Arduino.h>

size_t parse_whitespace(char *pbuf, size_t buf_nbytes);
size_t parse_bool(char *pbuf, size_t buf_nbytes, bool *pout_bool);
size_t parse_char(char *pbuf, size_t buf_nbytes, char *pout_char);
size_t parse_int(char *pbuf, size_t buf_nbytes, int *pout_int);
size_t parse_string(char *pbuf, size_t buf_nbytes, char *pout_string, size_t out_string_nbytes);
size_t parse_float(char *pbuf, size_t buf_nbytes, float *pout_float);
size_t parse_motor_id(char *pbuf, size_t buf_nbytes,  motor_id_t *pout_motor_id);
size_t parse_string_in_table(char *pbuf, size_t buf_nbytes, char *ptable[], int ntable_entries, int *pout_entry_num);
size_t parse_motor_angle_or_encoder(char *pargs, size_t args_nbytes, float *pvalue);
bool test_parse();









