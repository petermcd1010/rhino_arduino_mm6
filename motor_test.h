#pragma once
/*
 * Declarations for motor testing.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

bool motor_test(motor_id_t motor_id);  // Returns true if passed, false if failed.
void motor_test_mask(int motorids_mask);  // TODO: Return true if all pass, false otherwise.
