#pragma once

/*
 * Declarations for MegaMotor6-specific hardware functionality.
 */

#include <Arduino.h>

// Motor IDs are used to select an element from the arrays that hold motor information.
typedef enum {
  MOTOR_ID_FIRST = 0,
  MOTOR_ID_A = MOTOR_ID_FIRST,
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
  MOTOR_PROGRESS_ON_WAY_TO_TARGET, // More than 200 clicks away.
} motor_progress_t;

typedef enum {
  MOTOR_ERROR_FLAG_INVALID_ENCODER_TRANSITION = 1 << 0,  // only 0->1->3->2 and 0->2->3->1 are valid.
  MOTOR_ERROR_FLAG_OPPOSITE_DIRECTION = 1 << 1,
  MOTOR_ERROR_FLAG_UNEXPECTED_SWITCH_ENCODER = 1 << 2,
} motor_error_flag_t;

typedef struct {
  bool enabled;
  int speed;
  int target_speed;
  int pwm;
  int logic;  // -1 or +1.
  int previous_direction;
  int pid_dvalue;
  int pid_perror;  // Proportional Error (Difference between Current and Target)
  int target_encoder;
  int current_draw;  // TODO: units? counts?
  motor_progress_t progress;

  bool switch_previously_triggered;  // Alignment switch previous value used for debounce.
  bool switch_triggered;  // Alignment switch previous value used for debounce.
  int switch_forward_on;  // Alighment switch forward direction high value.
  int switch_forward_off;  // Alignment switch forward direction low value.
  int switch_reverse_on;  // Alignment switch reverse direction high value.
  int switch_reverse_off;  // Alignment switch reverse direction low value.

  motor_error_flag_t error_flags;
} motor_state_t;

extern motor_state_t motor_state[MOTOR_ID_COUNT]; 

// Mechanical orientation based on motor installation side.
typedef enum {
  MOTOR_ORIENTATION_INVERTED = -1,
  MOTOR_ORIENTATION_NOT_INVERTED = 1
} motor_orientation_t;

// Motor wiring polarity. Corrects for motors being wired backwards.
typedef enum {
  MOTOR_POLARITY_REVERSED = -1,
  MOTOR_POLARITY_NOT_REVERSED = 1,
} motor_polarity_t;

// Configuration stored in RAM and saved across reset/reboot that don't include a power-cycle of the board.
const int noinit_data_version = 2;
const int noinit_data_magic = 0xABCD1234;
typedef struct {
  // nbytes, version, magic are used to verify valid data.
  size_t nbytes;
  int version;
  int magic;
  int previous_quadrature_encoder[MOTOR_ID_COUNT];
  int encoder[MOTOR_ID_COUNT];
} noinit_data_t;

extern noinit_data_t noinit_data;

extern bool mm6_pid_enabled;
extern const int motor_min_speed;

extern int Gripper_StallC;
extern int Gripper_StallE;
extern int Gripper_StallX;
extern int SyncMove_Status;

void mm6_init();
void mm6_set_brake(motor_id_t motor_id, bool enable);
bool mm6_enabled(motor_id_t motor_id);
void mm6_enable_all(bool enable);
bool mm6_get_switch_triggered(motor_id_t motor_id);
int mm6_get_encoder(motor_id_t motor_id);
void mm6_set_target_encoder(motor_id_t motor_id, int encoder);
void mm6_print_encoders();
int mm6_angle_to_encoder(motor_id_t motor_id, float angle);
float mm6_get_angle(motor_id_t motor_id);
void mm6_set_target_angle(motor_id_t motor_id, float angle);
bool mm6_get_thermal_overload_active(motor_id_t motor_id);
int mm6_get_current_draw(motor_id_t motor_id);
bool mm6_get_overcurrent_active(motor_id_t motor_id);
void mm6_test_all();
bool mm6_calibrate_all();
void mm6_pid_enable(bool enable);
void mm6_set_speed(motor_id_t motor_id, int speed);
void mm6_dump(motor_id_t motor_id);

