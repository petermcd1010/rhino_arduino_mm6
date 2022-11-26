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

extern const char * const config_robot_name_by_id[CONFIG_ROBOT_ID_COUNT];

typedef struct __attribute__((packed)) {
    bool is_configured;
    int angle_offset;
    motor_orientation_t orientation;
    uint8_t forward_polarity;  // Defaults to LOW. If motor is wired backward, set to HIGH.
    int min_encoder;  // Minimum encoder limit found during calibration.
    int max_encoder;  // Maximum encoder limit found during calibration.
    bool is_gripper;
    struct {
        int gripper_open_encoder;
        int gripper_close_encoder;
    };
    int stall_current_threshold;  // Threshold to trigger stall condition. Defaults to 200.
} config_motor_t;

typedef struct __attribute__((packed)) {
    // nbytes, version, magic, and crc are used to verify a valid config.
    size_t nbytes;
    int version;
    uint32_t magic;
    uint32_t crc;

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
void config_set_robot_id(config_robot_id_t robot_id);
void config_set_robot_serial(char robot_serial[CONFIG_ROBOT_SERIAL_NBYTES]);
void config_set_robot_name(char robot_name[CONFIG_ROBOT_NAME_NBYTES]);
void config_set_motor_orientation(motor_id_t motor_id, motor_orientation_t motor_orientation);
void config_set_motor_forward_polarity(motor_id_t motor_id, int low_or_high);
void config_set_motor_angle_offsets(int B, int C, int D, int E, int F);
void config_set_motor_min_max_encoders(motor_id_t motor_id, int min_encoder, int max_encoder);
void config_set_motor_home_encoder(motor_id_t motor_id, int encoder);  // Sets encoder to zero and adjusts min, max limits.
void config_set_motor_gripper_open_close_encoders(motor_id_t motor_id, int gripper_open_encoder, int gripper_close_encoder);
void config_set_motor_stall_current_threshold(motor_id_t motor_id, int stall_current_threshold);
void config_print_one(motor_id_t motor_id);
void config_print(void);
bool config_test(void);
