/*
 * Implementation for persistent configuration stored in EEPROM.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <EEPROM.h>
#include <limits.h>

#include "config.h"
#include "crc32c.h"
#include "log.h"
#include "waypoint.h"

static const char name_not_configured[] PROGMEM = "Not configured";
static const char name_xr_1[] PROGMEM = "Rhino XR-1 6-axis arm";
static const char name_xr_2[] PROGMEM = "Rhino XR-2 6-axis arm";
static const char name_xr_3[] PROGMEM = "Rhino XR-3 6-axis arm";
static const char name_xr_4[] PROGMEM = "Rhino XR-4 6-axis arm";
static const char name_scara[] PROGMEM = "Rhino SCARA 5-axis arm";
static const char name_linear[] PROGMEM = "Rhino linear slide table";
static const char name_xy[] PROGMEM = "Rhino XY slide table";
static const char name_tilt[] PROGMEM = "Rhino tilt carousel";
static const char name_conveyor[] PROGMEM = "Rhino conveyor belt";

const char * const config_robot_name_by_id[CONFIG_ROBOT_ID_COUNT] = {
    name_not_configured,
    name_xr_1,
    name_xr_2,
    name_xr_3,
    name_xr_4,
    name_scara,
    name_linear,
    name_xy,
    name_tilt,
    name_conveyor,
};

static const int config_base_address = EEPROM.length() - sizeof(config_t);
static const int config_version = 1;
static const int config_magic = 0x5678FEAD;

config_t config = {};

void config_get_waypoint_eeprom_region(int *base_address, int *nbytes)
{
    assert(base_address);
    assert(nbytes);

    *base_address = 0;
    *nbytes = config_base_address;
}

static bool buffer_contains(char *pbuffer, size_t buffer_nbytes, char c)
{
    assert(pbuffer);

    for (int i = 0; i < buffer_nbytes; i++) {
        if (pbuffer[i] == c)
            return true;
    }

    return false;
}

static void config_sign()
{
    config.nbytes = sizeof(config_t);
    config.version = config_version;
    config.magic = config_magic;
    config.crc = 0;
    config.crc = crc32c_calculate(&config, sizeof(config_t));
}

bool config_read()
{
    EEPROM.get(config_base_address, config);
    return config_check();
}

bool config_write()
{
    if (!config_check())
        return false;

    EEPROM.put(config_base_address, config);  // Doesn't return success/failure indiction.
    return true;
}

bool config_check()
{
    // If there's an error in the header, all bets are off. Just return false.
    if (config.nbytes != sizeof(config_t)) {
        log_writeln(F("ERROR: config_check: Invalid config block size."));
        return false;
    }

    if (config.version != config_version) {
        log_writeln(F("ERROR: config_check: Invalid config version %d."), config.version);
        return false;
    }

    if (config.magic != config_magic) {
        log_writeln(F("ERROR: config_check: Invalid config magic %08x."), config.magic);
        return false;
    }

    long given_crc = config.crc;

    config.crc = 0;
    long calculated_crc = crc32c_calculate(&config, sizeof(config_t));

    config.crc = given_crc;  // Restore it.
    if (calculated_crc != given_crc) {
        log_writeln(F("ERROR: config_check: Invalid CRC %08lx. Expected %08lx."), calculated_crc, config.crc);
        return false;
    }

    // If there's an error in the contents, keep going, but return false.
    bool ret = true;

    if ((config.robot_id < CONFIG_ROBOT_ID_FIRST) || (config.robot_id > CONFIG_ROBOT_ID_LAST)) {
        log_writeln(F("ERROR: config_check: Invalid robot ID %d."), config.robot_id);
        ret = false;
    }

    if (!buffer_contains(config.robot_serial, CONFIG_ROBOT_SERIAL_NBYTES, '\0')) {
        log_writeln(F("ERROR: config_check: Robot serial isn't null-terminated."));
        ret = false;
    }

    if (!buffer_contains(config.robot_name, CONFIG_ROBOT_NAME_NBYTES, '\0')) {
        log_writeln(F("ERROR: config_check: Robot name isn't null-terminated."));
        ret = false;
    }

    const int min_count = -9999;  // TODO: Determine min_count.
    const int max_count = 9999;  // TODO: Determine max_count..

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if ((config.motor[i].angle_offset < min_count) || (config.motor[i].angle_offset > max_count)) {
            log_writeln(F("ERROR: config_check: Invalid angle offset."));
            ret = false;
        }

        if ((config.motor[i].orientation != MOTOR_ORIENTATION_NOT_INVERTED) &&
            (config.motor[i].orientation != MOTOR_ORIENTATION_INVERTED)) {
            log_writeln(F("ERROR: config_check: Invalid motor orientation for motor %c."), 'A' + i);
            ret = false;
        }

        if ((config.motor[i].polarity != MOTOR_POLARITY_NOT_REVERSED) &&
            (config.motor[i].polarity == MOTOR_POLARITY_REVERSED)) {
            log_writeln(F("ERROR: config_check: Invalid motor polarity for motor %c."), 'A' + i);
            ret = false;
        }
    }

    if ((config.gripper_open_encoder < min_count) || (config.gripper_open_encoder > max_count)) {
        log_writeln(F("ERROR: config_check: Invalid gripper_open."));
        ret = false;
    }

    if ((config.gripper_close_encoder < min_count) || (config.gripper_close_encoder > max_count)) {
        log_writeln(F("ERROR: config_check: Invalid gripper_open."));
        ret = false;
    }

    return ret;
}

void config_clear()
{
    memset(&config, 0, sizeof(config_t));
    config.robot_id = CONFIG_ROBOT_ID_DEFAULT;
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        config.motor[i].orientation = MOTOR_ORIENTATION_NOT_INVERTED;
        config.motor[i].polarity = MOTOR_POLARITY_NOT_REVERSED;
        config.motor[i].min_encoder = INT_MIN;
        config.motor[i].max_encoder = INT_MAX;
        config.motor[i].home_forward_on_encoder = INT_MAX;
        config.motor[i].home_forward_off_encoder = INT_MAX;
        config.motor[i].home_reverse_on_encoder = INT_MIN;
        config.motor[i].home_reverse_off_encoder = INT_MIN;
        config.motor[i].stall_current_threshold = 200;
    }
    config.gripper_open_encoder = -130;
    config.gripper_close_encoder = -310;
    config_sign();
}

void config_set_robot_id(config_robot_id_t robot_id)
{
    assert(robot_id >= CONFIG_ROBOT_ID_FIRST);
    assert(robot_id <= CONFIG_ROBOT_ID_LAST);

    assert(config_check());
    config.robot_id = robot_id;
    config_sign();
}

void config_set_robot_serial(char robot_serial[CONFIG_ROBOT_SERIAL_NBYTES])
{
    assert(robot_serial);
    assert(buffer_contains(robot_serial, CONFIG_ROBOT_SERIAL_NBYTES, 0));
    assert(config_check());
    memcpy(config.robot_serial, robot_serial, CONFIG_ROBOT_SERIAL_NBYTES);
    config_sign();
}

void config_set_robot_name(char robot_name[CONFIG_ROBOT_NAME_NBYTES])
{
    assert(robot_name);
    assert(buffer_contains(robot_name, CONFIG_ROBOT_NAME_NBYTES, 0));
    assert(config_check());
    memcpy(config.robot_name, robot_name, CONFIG_ROBOT_NAME_NBYTES);
    config_sign();
}

void config_set_motor_orientation(motor_id_t motor_id, motor_orientation_t motor_orientation)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);
    assert((motor_orientation == MOTOR_ORIENTATION_NOT_INVERTED) ||
           (motor_orientation == MOTOR_ORIENTATION_INVERTED));

    assert(config_check());
    config.motor[motor_id].orientation = motor_orientation;
    config_sign();
}

void config_set_motor_polarity(motor_id_t motor_id, motor_polarity_t motor_polarity)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);
    assert((motor_polarity == MOTOR_POLARITY_NOT_REVERSED) ||
           (motor_polarity == MOTOR_POLARITY_REVERSED));
    assert(config_check());
    config.motor[motor_id].polarity = motor_polarity;
    config_sign();
}

void config_set_gripper_open_encoder(int encoder)
{
    // TODO: assert encoder is in valid range.

    assert(config_check());
    config.gripper_open_encoder = encoder;
    config_sign();
}

void config_set_gripper_close_encoder(int encoder)
{
    // TODO: assert encoder is in valid range.

    assert(config_check());
    config.gripper_close_encoder = encoder;
    config_sign();
}

void config_set_angle_offsets(int B, int C, int D, int E, int F)
{
    config.motor[MOTOR_ID_B].angle_offset = B;
    config.motor[MOTOR_ID_C].angle_offset = C;
    config.motor[MOTOR_ID_D].angle_offset = D;
    config.motor[MOTOR_ID_E].angle_offset = E;
    config.motor[MOTOR_ID_F].angle_offset = F;
    log_write(F("Angle Offsets Set to: %d, %d, %d, %d, %d."),
              config.motor[MOTOR_ID_B].angle_offset,
              config.motor[MOTOR_ID_B].angle_offset,
              config.motor[MOTOR_ID_B].angle_offset,
              config.motor[MOTOR_ID_B].angle_offset,
              config.motor[MOTOR_ID_B].angle_offset);
}

void config_set_min_max_encoders(motor_id_t motor_id, int min_encoder, int max_encoder)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);
    config.motor[motor_id].min_encoder = min_encoder;
    config.motor[motor_id].max_encoder = max_encoder;
    config_sign();
}

void config_set_home_encoders(motor_id_t motor_id, int home_forward_on_encoder, int home_forward_off_encoder, int home_reverse_on_encoder, int home_reverse_off_encoder)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);
    config.motor[motor_id].home_forward_on_encoder = home_forward_on_encoder;
    config.motor[motor_id].home_forward_off_encoder = home_forward_off_encoder;
    config.motor[motor_id].home_reverse_on_encoder = home_reverse_on_encoder;
    config.motor[motor_id].home_reverse_off_encoder = home_reverse_off_encoder;
    config_sign();
}

void config_set_stall_current_threshold(motor_id_t motor_id, int stall_current_threshold)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);
    config.motor[motor_id].stall_current_threshold = stall_current_threshold;
    config_sign();
}

void config_print()
{
    if (!config_check())
        log_writeln(F("Invalid configuration."));

    log_writeln(F("Configuration:"));
    log_write(F("  Robot ID: "));
    log_writeln((const __FlashStringHelper *)config_robot_name_by_id[config.robot_id]);
    log_writeln(F("  Robot serial: %s"), config.robot_serial);
    log_writeln(F("  Robot name: %s"), config.robot_name);

    char str[15] = {};

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        config_motor_t *motor = &config.motor[i];

        log_write(F("  Motor %c: "), 'A' + i);
        dtostrf(motor->angle_offset, 3, 2, str);
        log_writeln(F("angle_offset:%s motor_orientation:%s, direction_logic:%s"),
                    str,
                    motor->orientation == MOTOR_ORIENTATION_NOT_INVERTED ? "not inverted" : "inverted",
                    motor->polarity == MOTOR_POLARITY_NOT_REVERSED ? "not reversed" : "reversed");
        log_writeln(F("           Min encoder: %d, max encoder: %d"), motor->min_encoder, motor->max_encoder);
        log_writeln(F("           Home switch forward on encoder: %d, forward off encoder: %d"), motor->home_forward_on_encoder, motor->home_forward_off_encoder);
        log_writeln(F("           Home switch reverse on encoder: %d, reverse off encoder: %d"), motor->home_reverse_on_encoder, motor->home_reverse_off_encoder);
        log_writeln(F("           Stall current threshold: %d"), motor->stall_current_threshold);
    }

    log_writeln(F("  Gripper open encoder: %d"), config.gripper_open_encoder);
    log_writeln(F("  Gripper close encoder: %d"), config.gripper_close_encoder);

    log_writeln(F("  Validated %d stored waypoints."), waypoint_get_used_count());
}

bool config_test()
{
    bool ret = true;

    if (sizeof(config_t) > EEPROM.length()) {
        LOG_ERROR(F("sizeof(config_t) > EEPROM.length()"));
        ret = false;
    }

    // test_config_check
    // test_config_read -- skip, as we don't want to exercise the EEPROM.
    // test_config_write -- skip, as we don't want to exercise the EEPROM.
    return ret;
}
