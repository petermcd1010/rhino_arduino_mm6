#pragma once
/*
 * Declarations for calibration.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "motor.h"

typedef struct {
    int min_encoder;
    int max_encoder;
    int switch_forward_on;  // Home switch forward direction high value.
    int switch_forward_off;  // Home switch forward direction low value.
    int switch_reverse_on;  // Home switch reverse direction high value.
    int switch_reverse_off;  // Home switch reverse direction low value.
} calibrate_data_t;

void calibrate_home_switch_and_limits(int motor_ids_mask, int max_speed_percent);
void calibrate_home_switch(int motor_ids_mask, int max_speed_percent);
