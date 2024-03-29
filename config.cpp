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
static const char name_linear[] PROGMEM = "Rhino linear slide base";
static const char name_xy[] PROGMEM = "Rhino XY slide base";
static const char name_rotary[] PROGMEM = "Rhino rotary carousel";
static const char name_tilt[] PROGMEM = "Rhino tilt carousel";
static const char name_conveyor[] PROGMEM = "Rhino conveyor belt";

const char *const config_robot_name_by_id[CONFIG_ROBOT_ID_COUNT] = {
    name_not_configured,
    name_xr_1,
    name_xr_2,
    name_xr_3,
    name_xr_4,
    name_scara,
    name_linear,
    name_xy,
    name_rotary,
    name_tilt,
    name_conveyor,
};

static const char boot_mode_wait_user_input[] PROGMEM = "Wait for user input";
static const char boot_mode_execute_waypoint_sequence[] PROGMEM = "Execute waypoint sequence";

const char *const config_boot_mode_by_id[CONFIG_BOOT_MODE_COUNT] = {
    boot_mode_wait_user_input,
    boot_mode_execute_waypoint_sequence,
};

static const int config_base_address = EEPROM.length() - sizeof(config_t);
static const int config_version = 1;
static const int config_magic = 0x5678FEAD;

config_t config = {};
static bool modified = false;

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
    modified = false;
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

    if ((config.boot_mode < 0) || (config.boot_mode >= CONFIG_BOOT_MODE_COUNT)) {
        log_writeln(F("ERROR: config_check: Invalid bootmode %d."), config.boot_mode);
        ret = false;
    }

    if ((config.robot_id < 0) || (config.robot_id >= CONFIG_ROBOT_ID_COUNT)) {
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

        if ((config.motor[i].orientation != MOTOR_ORIENTATION_NOT_INVERTED) && (config.motor[i].orientation != MOTOR_ORIENTATION_INVERTED)) {
            log_writeln(F("ERROR: config_check: Invalid motor orientation for motor %c."), 'A' + i);
            ret = false;
        }

        if ((config.motor[i].forward_polarity != HIGH) && (config.motor[i].forward_polarity != LOW)) {
            log_writeln(F("ERROR: config_check: Invalid motor polarity for motor %c."), 'A' + i);
            ret = false;
        }
    }

    return ret;
}

bool config_modified()
{
    return modified;
}

void config_clear()
{
    memset(&config, 0, sizeof(config_t));
    config.robot_id = CONFIG_ROBOT_ID_DEFAULT;
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        config.motor[i].orientation = MOTOR_ORIENTATION_NOT_INVERTED;
        config.motor[i].forward_polarity = LOW;  // Correctly-wired polarity is LOW.
        config.motor[i].min_encoder = INT_MIN;
        config.motor[i].max_encoder = INT_MAX;
        config.motor[i].stall_current_threshold = 200;  // Determined empirically. Max is 255.
    }
    config_sign();

    modified = true;
}

void config_print_one_gpio_pin_config(hardware_gpio_pin_t gpio_pin)
{
    assert(gpio_pin >= 0 && gpio_pin < HARDWARE_GPIO_PIN_COUNT);

    log_write(F("%d : "), gpio_pin);
    log_write((const __FlashStringHelper *)hardware_gpio_pin_name_by_index[gpio_pin]);
    log_write(F(" -- "));
    if (config.gpio_pin_mode[gpio_pin] == HARDWARE_GPIO_PIN_MODE_INPUT)
        log_write(F("input."));
    else if (config.gpio_pin_mode[gpio_pin] == HARDWARE_GPIO_PIN_MODE_INPUT_PULLUP)
        log_write(F("input with pull-up resistor (high impedence)."));
    else if (config.gpio_pin_mode[gpio_pin] == HARDWARE_GPIO_PIN_MODE_OUTPUT)
        log_write(F("output."));
}

void config_init_gpio_pins()
{
    bool all_default = true;

    for (int i = 0; i < HARDWARE_GPIO_PIN_COUNT; i++) {
        if (config.gpio_pin_mode[i] != HARDWARE_GPIO_PIN_MODE_DEFAULT)
            all_default = false;
    }

    if (all_default) {
        log_writeln(F("GPIO header pins all configured as power-up default (input)."));
        return;
    }

    log_writeln(F("Configuring GPIO pins as non-default (i.e. non-input):"));
    for (int i = 0; i < HARDWARE_GPIO_PIN_COUNT; i++) {
        if (config.gpio_pin_mode[i] == HARDWARE_GPIO_PIN_MODE_INPUT)
            // As a precaution against misconfigurations, only change mode if it's not the power-on default.
            continue;

        log_write(F("  "));
        config_print_one_gpio_pin_config(i);
        log_writeln();
        hardware_set_gpio_pin_mode(i, config.gpio_pin_mode[i]);
    }
}

void config_set_boot_mode(config_boot_mode_t boot_mode)
{
    assert(config_check());
    config.boot_mode = boot_mode;
    config_sign();

    modified = true;
}

void config_set_robot_id(config_robot_id_t robot_id)
{
    assert(robot_id >= 0);
    assert(robot_id < CONFIG_ROBOT_ID_COUNT);

    assert(config_check());
    config.robot_id = robot_id;
    config_sign();

    modified = true;
}

void config_set_robot_serial(char robot_serial[CONFIG_ROBOT_SERIAL_NBYTES])
{
    assert(robot_serial);
    assert(buffer_contains(robot_serial, CONFIG_ROBOT_SERIAL_NBYTES, 0));
    assert(config_check());
    memcpy(config.robot_serial, robot_serial, CONFIG_ROBOT_SERIAL_NBYTES);
    config_sign();

    modified = true;
}

void config_set_robot_name(char robot_name[CONFIG_ROBOT_NAME_NBYTES])
{
    assert(robot_name);
    assert(buffer_contains(robot_name, CONFIG_ROBOT_NAME_NBYTES, 0));
    assert(config_check());
    memcpy(config.robot_name, robot_name, CONFIG_ROBOT_NAME_NBYTES);
    config_sign();

    modified = true;
}

void config_set_motor_orientation(motor_id_t motor_id, motor_orientation_t motor_orientation)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);
    assert((motor_orientation == MOTOR_ORIENTATION_NOT_INVERTED) || (motor_orientation == MOTOR_ORIENTATION_INVERTED));

    assert(config_check());
    config.motor[motor_id].is_configured = true;
    config.motor[motor_id].orientation = motor_orientation;
    config_sign();

    modified = true;
}

void config_set_motor_forward_polarity(motor_id_t motor_id, int high_or_low)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);
    assert((high_or_low == HIGH) || (high_or_low == LOW));
    assert(config_check());

    config.motor[motor_id].is_configured = true;
    config.motor[motor_id].forward_polarity = high_or_low;
    config_sign();

    modified = true;
}

void config_set_motor_min_max_encoders(motor_id_t motor_id, int min_encoder, int max_encoder)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);

    config.motor[motor_id].is_configured = true;
    config.motor[motor_id].min_encoder = min_encoder;
    config.motor[motor_id].max_encoder = max_encoder;
    config_sign();

    modified = true;
}

void config_set_motor_home_encoder(motor_id_t motor_id, int encoder)
{
    config.motor[motor_id].is_configured = true;
    config.motor[motor_id].min_encoder = min(INT_MAX, max(INT_MIN, (long)config.motor[motor_id].min_encoder - encoder));
    config.motor[motor_id].max_encoder = min(INT_MAX, max(INT_MIN, (long)config.motor[motor_id].max_encoder - encoder));

    if (config.motor[motor_id].is_gripper) {
        int new_gripper_close_encoder = min(INT_MAX, max(INT_MIN, (long)config.motor[motor_id].gripper_close_encoder - encoder));
        config_set_motor_gripper_close_encoder(motor_id, new_gripper_close_encoder);
    }

    config_sign();

    modified = true;
};

void config_set_motor_encoders_per_degree(motor_id_t motor_id, float encoders_per_degree)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);

    config.motor[motor_id].is_configured = true;
    config.motor[motor_id].encoders_per_degree = encoders_per_degree;
    config_sign();

    modified = true;
}

void config_set_motor_angle_offset(motor_id_t motor_id, float angle_offset)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);

    config.motor[motor_id].is_configured = true;
    config.motor[motor_id].angle_offset = angle_offset;
    config_sign();

    modified = true;
}

int config_motor_angle_to_encoders(motor_id_t motor_id, float angle)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);

    return (angle + config.motor[motor_id].angle_offset) * config.motor[motor_id].encoders_per_degree;
}

float config_motor_encoders_to_angle(motor_id_t motor_id, int encoders)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);

    return (encoders / config.motor[motor_id].encoders_per_degree) - config.motor[motor_id].angle_offset;
}

void config_set_motor_gripper_close_encoder(motor_id_t motor_id, int gripper_close_encoder)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);

    config.motor[motor_id].is_configured = true;
    config.motor[motor_id].is_gripper = true;
    config.motor[motor_id].gripper_close_encoder = gripper_close_encoder;
    config_sign();

    modified = true;
}

void config_set_motor_stall_current_threshold(motor_id_t motor_id, int stall_current_threshold)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);

    config.motor[motor_id].stall_current_threshold = stall_current_threshold;
    config_sign();

    modified = true;
}

void config_set_gpio_pin_mode(hardware_gpio_pin_t gpio_pin, hardware_gpio_pin_mode_t gpio_mode)
{
    assert(gpio_pin >= 0 && gpio_pin < HARDWARE_GPIO_PIN_COUNT);

    config.gpio_pin_mode[gpio_pin] = gpio_mode;
    hardware_set_gpio_pin_mode(gpio_pin, gpio_mode);
    config_sign();

    modified = true;
}

void config_print_one_motor(motor_id_t motor_id)
{
    assert(motor_id >= 0 && motor_id < MOTOR_ID_COUNT);

    if (!config_check())
        log_writeln(F("Invalid configuration."));

    config_motor_t *m = &config.motor[motor_id];

    log_write(F("  Motor %c: "), 'A' + motor_id);
    char str[15] = {};

    if (!m->is_configured) {
        log_writeln(F("Not configured. Run calibration."));
        return;
    }

    log_write(F("Encoder min: %d, home: 0, max: %d"), m->min_encoder, m->max_encoder);
    if (m->is_gripper)
        log_writeln(F(", gripper close: %d."), m->gripper_close_encoder);
    else
        log_writeln(F("."));

    if (m->encoders_per_degree == 0.0) {
        log_writeln(F("           Angle encoders/degree: Not configured."));
    } else {
        dtostrf(m->encoders_per_degree, 3, 2, str);
        log_write(F("           Angle encoders/degree: %s, "), str);

        dtostrf(m->angle_offset, 3, 2, str);
        log_write(F("offset: %s, "), str);

        dtostrf(config_motor_encoders_to_angle(motor_id, m->min_encoder), 3, 2, str);
        log_write(F("min: %s, "), str);

        dtostrf(config_motor_encoders_to_angle(motor_id, m->max_encoder), 3, 2, str);
        log_writeln(F("max: %s."), str);
    }

    log_write(F("           Motor orientation: %s, polarity: %s, stall current threshold: "),
              m->orientation == MOTOR_ORIENTATION_NOT_INVERTED ? "not inverted" : "inverted",
              m->forward_polarity == LOW ? "not reversed" : "reversed",
              m->stall_current_threshold);

    if (m->stall_current_threshold == 0)
        log_writeln(F("*DISABLED*."));
    else
        log_writeln(F("%d."), m->stall_current_threshold);
}

void config_print()
{
    if (!config_check())
        log_writeln(F("Invalid configuration."));

    log_writeln(F("Configuration:"));
    log_write(F("  Boot mode: "));
    log_write((const __FlashStringHelper *)config_boot_mode_by_id[config.boot_mode]);
    log_writeln(F("."));
    log_writeln(F("  GPIO pins:"));
    for (int i = 0; i < HARDWARE_GPIO_PIN_COUNT; i++) {
        log_write(F("    "));
        config_print_one_gpio_pin_config(i);
        log_writeln();
    }
    log_write(F("  Robot ID: "));
    log_write((const __FlashStringHelper *)config_robot_name_by_id[config.robot_id]);
    log_writeln(F("."));
    log_writeln(F("  Robot serial: '%s'."), config.robot_serial);
    log_writeln(F("  Robot name: '%s'."), config.robot_name);
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        config_print_one_motor(i);
    }
    log_writeln(F("  Waypoints: %d of maximum %d waypoints saved."), waypoint_get_used_count(), waypoint_get_max_count());
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
