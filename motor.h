#pragma once
/*
 * Declarations for MegaMotor6 hardware functionality.
 * See https://www.ti.com/lit/gpn/lmd18200 for the MegaMotor6's motor drivers.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

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
    MOTOR_ERROR_THERMAL_OVERLOAD_DETECTED = 0,  // Motor drive chip reports 145°C (junction temperature) exceeded.
    MOTOR_ERROR_INVALID_ENCODER_TRANSITION,  // Quadrature encoder reading didn't match what the previous or next encoder reading should be.
    MOTOR_ERROR_UNEXPECTED_STALL_CURRENT_THRESHOLD_EXCEEDED,  // The motor is at its target, but the current threshold was exceeded.
    MOTOR_ERROR_COUNT
} motor_error_t;

extern const char *const motor_error_name_by_id[MOTOR_ERROR_COUNT];

// Mechanical orientation based on motor installation side.
typedef enum {
    MOTOR_ORIENTATION_INVERTED     = -1,
    MOTOR_ORIENTATION_NOT_INVERTED = 1
} motor_orientation_t;

typedef struct {
    bool          enabled;
    int           max_velocity;
    int           velocity;
    int           pwm;
    int           prev_direction;
    int           pid_dvalue;
    int           pid_perror;      // Proportional Error (Difference between Current and Target)
    int           target_encoder;
    int           current_draw;      // TODO: units? counts?
    bool          stall_triggered;      // If current >= stall_current once, remains true until cleared.
    bool          prev_home_is_pressed;       // Switch value last time inversion detected.
    unsigned long prev_home_millis;    // Time last inversion detected.
    int           prev_home_encoder;       // Encoder value last time inversion detected.
    bool          home_is_pressed_debounced;      // Debounced switch value.
    int           home_forward_on_encoder;      // Home switch forward direction high value.
    int           home_forward_off_encoder;      // Home switch forward direction low value.
    int           home_reverse_on_encoder;      // Home switch reverse direction high value.
    int           home_reverse_off_encoder;      // Home switch reverse direction low value.
    unsigned char error_flags_isr;    // Error flags from the latest execution of the ISR, bitwise or'd into error_flags.
    unsigned char error_flags;    // Cleared when motor_get_and_clear_error_flags() is called.
} motor_t;

extern motor_t motor[MOTOR_ID_COUNT];

void motor_clear_persistent_ram_data(void);  // Clears data cached in RAM between boots.

void motor_init(void);  // Initializes all motors, ISRs, etc.
int motor_get_current_draw(motor_id_t motor_id);
bool motor_stall_triggered(motor_id_t motor_id);
void motor_clear_stall(motor_id_t motor_id);
void motor_disable_all(void);
void motor_set_enabled(motor_id_t motor_id, bool enable);
bool motor_get_enabled(motor_id_t motor_id);
int motor_get_enabled_mask(void);
void motor_set_enabled_mask(int mask);
void motor_set_home_encoder(motor_id_t motor_id, int home_encoder);  // Home_encoder will become 0.
int motor_get_encoder(motor_id_t motor_id);
void motor_set_target_encoder(motor_id_t motor_id, int encoder);
int motor_get_target_encoder(motor_id_t motor_id);
float motor_get_angle(motor_id_t motor_id);
void motor_set_target_angle(motor_id_t motor_id, float angle);
float motor_get_percent(motor_id_t motor_id);
void motor_set_target_percent(motor_id_t motor_id, float percent);
bool motor_is_moving(motor_id_t motor_id);
void motor_set_max_velocity_percent(motor_id_t motor_id, int max_velocity_percent);
int motor_get_max_velocity_percent(motor_id_t motor_id);
bool motor_home_is_pressed(motor_id_t motor_id);
bool motor_home_is_pressed_debounced(motor_id_t motor_id);
void motor_dump(motor_id_t motor_id);
void motor_set_high_level_error(bool has_error);
int motor_get_error_flags(motor_id_t motor_id);  // Returns OR-mask of all errors active since last time called.
