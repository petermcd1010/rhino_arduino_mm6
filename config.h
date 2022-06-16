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
    CONFIG_ROBOT_ID_FIRST          = 0,
    CONFIG_ROBOT_ID_NOT_CONFIGURED = CONFIG_ROBOT_ID_FIRST,
    CONFIG_ROBOT_ID_RHINO_XR_1,
    CONFIG_ROBOT_ID_RHINO_XR_2,
    CONFIG_ROBOT_ID_RHINO_XR_3,
    CONFIG_ROBOT_ID_RHINO_XR_4,
    CONFIG_ROBOT_ID_RHINO_SCARA,
    CONFIG_ROBOT_ID_RHINO_LINEAR_SLIDE_TABLE,
    CONFIG_ROBOT_ID_RHINO_XY_SLIDE_TABLE,
    CONFIG_ROBOT_ID_RHINO_TILT_CAROUSEL,
    CONFIG_ROBOT_ID_RHINO_CONVEYOR_BELT,
    CONFIG_ROBOT_ID_COUNT,
    CONFIG_ROBOT_ID_LAST    = CONFIG_ROBOT_ID_COUNT - 1,
    CONFIG_ROBOT_ID_DEFAULT = CONFIG_ROBOT_ID_NOT_CONFIGURED,
} config_robot_id_t;

extern const char * const config_robot_name_by_id[CONFIG_ROBOT_ID_COUNT];

typedef struct __attribute__((packed)) {
    int angle_offset;
    motor_orientation_t orientation;
    motor_polarity_t polarity;  // Easy to wire motors backwards.
    int min_encoder;  // Minimum encoder limit found during calibration.
    int max_encoder;  // Maximum encoder limit found during calibration.
    int home_forward_on_encoder;  // Home switch forward direction triggered value.
    int home_forward_off_encoder;  // Home switch forward direction not triggered value.
    int home_reverse_on_encoder;  // Home switch reverse direction triggered value.
    int home_reverse_off_encoder;  // Home switch reverse direction not triggered value.
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
    int gripper_open_encoder;
    int gripper_close_encoder;
} config_t;

extern config_t config;

void config_get_waypoint_eeprom_region(int *base_address, int *nbytes);
bool config_read(void);
bool config_write(void);
bool config_check(void);
void config_clear(void);
void config_set_robot_id(config_robot_id_t robot_id);
void config_set_robot_serial(char robot_serial[CONFIG_ROBOT_SERIAL_NBYTES]);
void config_set_robot_name(char robot_name[CONFIG_ROBOT_NAME_NBYTES]);
void config_set_motor_configured(motor_id_t motor_id, bool configured);
void config_set_motor_orientation(motor_id_t motor_id, motor_orientation_t motor_orientation);
void config_set_motor_polarity(motor_id_t motor_id, motor_polarity_t motor_polarity);
void config_set_gripper_open_encoder(int encoder);
void config_set_gripper_close_encoder(int encoder);
void config_set_angle_offsets(int B, int C, int D, int E, int F);
void config_set_min_max_encoders(motor_id_t motor_id, int min_encoder, int max_encoder);
void config_set_home_encoders(motor_id_t motor_id, int home_forward_on_encoder, int home_forward_off_encoder, int home_reverse_on_encoder, int home_reverse_off_encoder);
void config_print(void);
bool config_test(void);
