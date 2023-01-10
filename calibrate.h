#pragma once
/*
 * Declarations for calibration.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "motor.h"

void calibrate_home_switch_and_limits(int motor_ids_mask, int max_velocity_percent);
void calibrate_home_switch(int motor_ids_mask, int max_velocity_percent);
