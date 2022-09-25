#pragma once
/*
 * Declarations for motor testing.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "motor.h"
#include "sm.h"

void motor_test(motor_id_t motor_id, sm_state_t exit_to_state);
void motor_test_mask(int motor_ids_mask, sm_state_t exit_to_state);
bool motor_test_passed(motor_id_t motor_id);
