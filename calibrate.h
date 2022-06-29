#pragma once
/*
 * Declarations for calibration.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "motor.h"

void calibrate_gripper(motor_id_t motor_id, int max_speed_percent);
void calibrate_home_switch_and_limits(int motor_ids_mask, int max_speed_percent);
void calibrate_home_switch(int motor_ids_mask, int max_speed_percent);
