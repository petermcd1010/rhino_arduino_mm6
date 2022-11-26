#pragma once
/*
 * Declarations for MegaMotor6 hardware functionality.
 * See https://www.ti.com/lit/gpn/lmd18200 for the MegaMotor6's motor drivers.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

typedef enum {
    MOTOR_ID_A = 0,
    MOTOR_ID_B,
    MOTOR_ID_C,
    MOTOR_ID_D,
    MOTOR_ID_E,
    MOTOR_ID_F,
    MOTOR_ID_COUNT,
} motor_id_t;

// Sum of all powers of 2 of an n-bit number is 2^n-1.
#define MOTOR_IDS_MASK ((1 << MOTOR_ID_COUNT) - 1)

typedef enum {
    MOTOR_PROGRESS_AT_TARGET = 0,
    MOTOR_PROGRESS_BESIDE_TARGET,  // Within 1 click.
    MOTOR_PROGRESS_NEAR_TARGET,  // between 2 and 30 clicks.
    MOTOR_PROGRESS_APPROACHING_TARGET,  // between 30 and 200 clicks.
    MOTOR_PROGRESS_ON_WAY_TO_TARGET,  // More than 200 clicks away.
} motor_progress_t;

typedef enum {
    MOTOR_ERROR_FLAG_USER_FLAG                                   = 1 << 0, // Hacky way to indicate the state machine is in ERROR, so LED blinks accordingly.
    MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED                   = 1 << 1,
    MOTOR_ERROR_FLAG_INVALID_ENCODER_TRANSITION                  = 1 << 2, // only 0->1->3->2 and 0->2->3->1 are valid.
    MOTOR_ERROR_FLAG_OPPOSITE_DIRECTION                          = 1 << 3,
    MOTOR_ERROR_FLAG_UNEXPECTED_HOME_SWITCH_ENCODER              = 1 << 4,
    MOTOR_ERROR_FLAG_UNEXPECTED_STALL_CURRENT_THRESHOLD_EXCEEDED = 1 << 5,
} motor_error_flag_t;

// Mechanical orientation based on motor installation side.
typedef enum {
    MOTOR_ORIENTATION_INVERTED     = -1,
    MOTOR_ORIENTATION_NOT_INVERTED = 1
} motor_orientation_t;

typedef struct {
    bool               enabled;
    int                max_speed;
    int                target_speed;
    int                speed;
    int                pwm;
    int                previous_direction;
    int                pid_dvalue;
    int                pid_perror; // Proportional Error (Difference between Current and Target)
    int                target_encoder;
    int                current; // TODO: units? counts?
    bool               stall_triggered; // If current >= stall_current once, remains true until cleared.
    motor_progress_t   progress;
    int                encoders_per_second;  // Updated 3x/second.
    int                encoders_per_second_counts;
    int                encoders_per_second_start_encoder;
    bool               prev_home_triggered;  // Switch value last time transition detected.
    unsigned long      prev_home_triggered_millis;  // Time last transition detected.
    int                prev_home_triggered_encoder;  // Encoder value last time transition detected.
    bool               home_triggered_debounced; // Debounced switch value.
    int                home_forward_on_encoder; // Home switch forward direction high value.
    int                home_forward_off_encoder; // Home switch forward direction low value.
    int                home_reverse_on_encoder; // Home switch reverse direction high value.
    int                home_reverse_off_encoder; // Home switch reverse direction low value.
    motor_error_flag_t error_flags;  // Once set, error flags must be cleared by user code.
} motor_state_t;

extern motor_state_t motor_state[MOTOR_ID_COUNT];

void motor_clear_ram_data(void);  // Clears data cached in RAM between boots.

void motor_init_all(void);
bool motor_get_thermal_overload_detected(motor_id_t motor_id);
bool motor_get_thermal_overload_detected();
void motor_clear_thermal_overload(motor_id_t motor_id);
int motor_get_current(motor_id_t motor_id);
bool motor_stall_triggered(motor_id_t motor_id);
void motor_clear_stall(motor_id_t motor_id);
void motor_disable_all(void);
void motor_set_enabled(motor_id_t motor_id, bool enable);
bool motor_get_enabled(motor_id_t motor_id);
int motor_get_enabled_mask(void);
void motor_set_enabled_mask(int mask);
void motor_set_home_encoder(motor_id_t motor_id, int home_encoder);  // Home_encoder will become 0.
float motor_get_encoder_steps_per_degree(motor_id_t motor_id);
int motor_get_encoder(motor_id_t motor_id);
void motor_set_target_encoder(motor_id_t motor_id, int encoder);
int motor_get_target_encoder(motor_id_t motor_id);
int motor_angle_to_encoder(motor_id_t motor_id, float angle);
float motor_get_angle(motor_id_t motor_id);
void motor_set_target_angle(motor_id_t motor_id, float angle);
float motor_get_percent(motor_id_t motor_id);
void motor_set_target_percent(motor_id_t motor_id, float percent);
bool motor_is_moving(motor_id_t motor_id);
void motor_set_speed(motor_id_t motor_id, int speed);  // For speed in [motor_min_speed, motor_max_speed]. Sets to 0 if not enabled.
void motor_set_max_speed_percent(motor_id_t motor_id, int max_speed_percent);
int motor_get_max_speed_percent(motor_id_t motor_id);
bool motor_is_home_triggered(motor_id_t motor_id);
bool motor_is_home_triggered_debounced(motor_id_t motor_id);
void motor_dump(motor_id_t motor_id);
void motor_set_user_error(bool enable);  // Will blink LED quickly to notify user of issue.
void motor_log_errors(motor_id_t motor_id);
