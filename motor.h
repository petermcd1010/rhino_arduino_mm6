#pragma once
/*
 * Declarations for MegaMotor6 hardware functionality.
 * See https://www.ti.com/lit/gpn/lmd18200 for the MegaMotor6's motor drivers.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

typedef enum {
    MOTOR_ID_FIRST = 0,
    MOTOR_ID_A     = MOTOR_ID_FIRST,
    MOTOR_ID_B,
    MOTOR_ID_C,
    MOTOR_ID_D,
    MOTOR_ID_E,
    MOTOR_ID_F,
    MOTOR_ID_COUNT,
    MOTOR_ID_LAST = MOTOR_ID_COUNT - 1
} motor_id_t;

typedef enum {
    MOTOR_PROGRESS_AT_TARGET = 0,
    MOTOR_PROGRESS_BESIDE_TARGET,  // Within 1 click.
    MOTOR_PROGRESS_NEAR_TARGET,  // between 2 and 30 clicks.
    MOTOR_PROGRESS_APPROACHING_TARGET,  // between 30 and 200 clicks.
    MOTOR_PROGRESS_ON_WAY_TO_TARGET,  // More than 200 clicks away.
} motor_progress_t;

typedef enum {
    MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED  = 1 << 0,
    MOTOR_ERROR_FLAG_OVERCURRENT_DETECTED       = 1 << 1,
    MOTOR_ERROR_FLAG_INVALID_ENCODER_TRANSITION = 1 << 2,  // only 0->1->3->2 and 0->2->3->1 are valid.
    MOTOR_ERROR_FLAG_OPPOSITE_DIRECTION         = 1 << 3,
    MOTOR_ERROR_FLAG_UNEXPECTED_SWITCH_ENCODER  = 1 << 4,
    MOTOR_ERROR_FLAG_ENCODER_OVERFLOW           = 1 << 5,
    MOTOR_ERROR_FLAG_ENCODER_UNDERFLOW          = 1 << 6,
} motor_error_flag_t;

// Mechanical orientation based on motor installation side.
typedef enum {
    MOTOR_ORIENTATION_INVERTED     = -1,
    MOTOR_ORIENTATION_NOT_INVERTED = 1
} motor_orientation_t;

// Motor wiring polarity. Corrects for motors being wired backwards.
typedef enum {
    MOTOR_POLARITY_REVERSED     = -1,
    MOTOR_POLARITY_NOT_REVERSED = 1,
} motor_polarity_t;

typedef struct {
    bool               enabled;
    int                speed;
    int                target_speed;
    int                pwm;
    int                logic; // -1 or +1.
    int                previous_direction;
    int                pid_dvalue;
    int                pid_perror; // Proportional Error (Difference between Current and Target)
    int                target_encoder;
    int                current_draw; // TODO: units? counts?
    motor_progress_t   progress;

    bool               switch_previously_triggered; // Alignment switch previous value used for debounce.
    bool               switch_triggered; // Alignment switch previous value used for debounce.
    int                switch_forward_on; // Alighment switch forward direction high value.
    int                switch_forward_off; // Alignment switch forward direction low value.
    int                switch_reverse_on; // Alignment switch reverse direction high value.
    int                switch_reverse_off; // Alignment switch reverse direction low value.

    motor_error_flag_t error_flags;  // Once set, error flags must be cleared by user code.
} motor_state_t;

extern motor_state_t motor_state[MOTOR_ID_COUNT];

void motor_clear_ram_data();  // Clears data cached in RAM between boots.

void motor_init_all();
bool motor_get_thermal_overload_detected(motor_id_t motor_id);
bool motor_get_thermal_overload_detected();
void motor_clear_thermal_overload(motor_id_t motor_id);
bool motor_get_overcurrent_detected(motor_id_t motor_id);
bool motor_get_overcurrent_detected();
void motor_clear_overcurrent(motor_id_t motor_id);
int motor_get_current_draw(motor_id_t motor_id);

void motor_disable_all(void);
void motor_set_enabled(motor_id_t motor_id, bool enable);
bool motor_get_enabled(motor_id_t motor_id);

int motor_get_enabled_mask();
void motor_set_enabled_mask(int mask);

void motor_set_target_encoder(motor_id_t motor_id, int encoder);
int motor_get_encoder(motor_id_t motor_id);
void motor_print_encoders();
float motor_get_encoder_steps_per_degree(motor_id_t motor_id);
int motor_angle_to_encoder(motor_id_t motor_id, float angle);
void motor_set_target_angle(motor_id_t motor_id, float angle);
float motor_get_angle(motor_id_t motor_id);
void motor_set_position_to_home(motor_id_t motor_id);
void motor_set_speed(motor_id_t motor_id, int speed);  // For speed in [motor_min_speed, motor_max_speed]. Sets speed to 0 if motor not enabled/configured.
bool motor_get_switch_triggered(motor_id_t motor_id);
void motor_test_enabled();
bool motor_calibrate(motor_id_t motor_id);
bool motor_calibrate_all();
void motor_dump(motor_id_t motor_id);
void motor_log_errors(motor_id_t motor_id);
