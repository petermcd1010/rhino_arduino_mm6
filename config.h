#pragma once

/*
 * Declarations for persistent configuration stored in EEPROM.
 */

#include <Arduino.h>
#include "mm6.h"

#define CONFIG_ROBOT_SERIAL_NBYTES 15
#define CONFIG_ROBOT_NAME_NBYTES 20

typedef enum {
  CONFIG_ROBOT_ID_FIRST = 0,
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
  CONFIG_ROBOT_ID_LAST = CONFIG_ROBOT_ID_COUNT - 1,
  CONFIG_ROBOT_ID_DEFAULT = CONFIG_ROBOT_ID_NOT_CONFIGURED,
} config_robot_id_t; 

extern const char* const config_robot_name_by_id[CONFIG_ROBOT_ID_COUNT]; 

typedef struct __attribute__((packed)) {
  bool configured;
  int angle_offset;
  motor_orientation_t orientation;  
  motor_polarity_t polarity;  // Easy to wire motors backwards.
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

bool config_read();
bool config_write();
bool config_check();
void config_clear();
void config_set_robot_id(config_robot_id_t robot_id);
void config_set_robot_serial(char robot_serial[CONFIG_ROBOT_SERIAL_NBYTES]);
void config_set_robot_name(char robot_name[CONFIG_ROBOT_NAME_NBYTES]);
void config_set_motor_configured(motor_id_t motor_id, bool configured);
void config_set_motor_orientation(motor_id_t motor_id, motor_orientation_t motor_orientation);
void config_set_motor_polarity(motor_id_t motor_id, motor_polarity_t motor_polarity);
void config_set_gripper_open_encoder(int encoder);
void config_set_gripper_close_encoder(int encoder);
void config_set_angle_offsets(int B, int C, int D, int E, int F);
void config_print();
bool config_test();
