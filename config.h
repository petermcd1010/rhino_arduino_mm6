#pragma once
/*
 * Declarations for persistent configuration stored in EEPROM.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>
#include "motor.h"

#define CONFIG_ROBOT_SERIAL_NBYTES 15
#define CONFIG_ROBOT_NAME_NBYTES 20

typedef enum {
    CONFIG_ROBOT_ID_NOT_CONFIGURED = 0,
    CONFIG_ROBOT_ID_RHINO_XR_1,
    CONFIG_ROBOT_ID_RHINO_XR_2,
    CONFIG_ROBOT_ID_RHINO_XR_3,
    CONFIG_ROBOT_ID_RHINO_XR_4,
    CONFIG_ROBOT_ID_RHINO_SCARA,
    CONFIG_ROBOT_ID_RHINO_LINEAR_SLIDE_BASE,
    CONFIG_ROBOT_ID_RHINO_XY_SLIDE_BASE,
    CONFIG_ROBOT_ID_RHINO_ROTARY_CAROUSEL,
    CONFIG_ROBOT_ID_RHINO_TILT_CAROUSEL,
    CONFIG_ROBOT_ID_RHINO_CONVEYOR_BELT,
    CONFIG_ROBOT_ID_COUNT,
    CONFIG_ROBOT_ID_DEFAULT = CONFIG_ROBOT_ID_NOT_CONFIGURED,
} config_robot_id_t;

typedef enum {
    CONFIG_BOOT_MODE_WAIT_USER_INPUT = 0,
    CONFIG_BOOT_MODE_EXECUTE_WAYPOINT_SEQUENCE,
    CONFIG_BOOT_MODE_COUNT,
    CONFIG_BOOT_MODE_DEFAULT         = CONFIG_BOOT_MODE_WAIT_USER_INPUT
} config_boot_mode_t;

extern const char *const config_robot_name_by_id[CONFIG_ROBOT_ID_COUNT];
extern const char *const config_boot_mode_by_id[CONFIG_BOOT_MODE_COUNT];

typedef struct __attribute__((packed)) {
    bool is_configured;
    motor_orientation_t orientation;
    uint8_t forward_polarity;  // Defaults to LOW. If +/- wired backward, motor test sets to HIGH.
    int min_encoder;  // Minimum encoder limit found during calibration or set in config.
    int max_encoder;  // Maximum encoder limit found during calibration or set in config.
    float encoders_per_degree;  // Number of encoder counts per angle degree.
    float angle_offset;  // +/- angle offset to zero-degrees from home encoder 0.
    bool is_gripper;  // True if calibration detected this motor actuates a gripper.
    int gripper_close_encoder;  // Calibrated gripper close encoder value.
    int stall_current_threshold;  // Threshold to trigger stall condition. Defaults to 200, 0 disables.
} config_motor_t;

typedef struct __attribute__((packed)) {
    // nbytes, version, magic, and crc are used to verify a valid config.
    size_t nbytes;
    int version;
    uint32_t magic;
    uint32_t crc;

    config_boot_mode_t boot_mode;
    config_robot_id_t robot_id;
    char robot_serial[CONFIG_ROBOT_SERIAL_NBYTES];  // To help user confirm board/robot match.
    char robot_name[CONFIG_ROBOT_NAME_NBYTES];  // Optional robot name for user.
    config_motor_t motor[MOTOR_ID_COUNT];
} config_t;

extern config_t config;

void config_get_waypoint_eeprom_region(int *base_address, int *nbytes);
bool config_read(void);
bool config_write(void);
bool config_check(void);
bool config_modified(void);
void config_clear(void);
void config_set_boot_mode(config_boot_mode_t boot_mode);
void config_set_robot_id(config_robot_id_t robot_id);
void config_set_robot_serial(char robot_serial[CONFIG_ROBOT_SERIAL_NBYTES]);
void config_set_robot_name(char robot_name[CONFIG_ROBOT_NAME_NBYTES]);
void config_set_motor_orientation(motor_id_t motor_id, motor_orientation_t motor_orientation);
void config_set_motor_forward_polarity(motor_id_t motor_id, int low_or_high);
void config_set_motor_min_max_encoders(motor_id_t motor_id, int min_encoder, int max_encoder);
void config_set_motor_home_encoder(motor_id_t motor_id, int encoder);  // Sets encoder to zero and adjusts min, max limits.
void config_set_motor_encoders_per_degree(motor_id_t motor_id, float encoders_per_degree);
void config_set_motor_angle_offset(motor_id_t motor_id, float angle_offset);
void config_set_motor_stall_current_threshold(motor_id_t, int stall_current_threshold);
int config_motor_angle_to_encoders(motor_id_t motor_id, float angle);
float config_motor_encoders_to_angle(motor_id_t motor_id, int encoders);
void config_set_motor_gripper_close_encoder(motor_id_t motor_id, int gripper_close_encoder);
void config_set_motor_stall_current_threshold(motor_id_t motor_id, int stall_current_threshold);
void config_print_one(motor_id_t motor_id);
void config_print(void);
bool config_test(void);
