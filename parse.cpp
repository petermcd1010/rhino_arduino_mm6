/*
 * Implementation for input parsing functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "limits.h"
#include "log.h"
#include "motor.h"
#include "parse.h"
#include "waypoint.h"  // WAYPOINT_COMMAND_*.

size_t parse_whitespace(char *buf, size_t buf_nbytes)
{
    assert(buf);

    char *p = buf;

    while (isspace(*p)) {
        p++;
    }

    return p - buf;
}

size_t parse_bool(char *buf, size_t buf_nbytes, bool *out_bool)
{
    assert(buf);
    assert(out_bool);

    // TODO: switch to table of PROGMEM.
    const char *bool_strings[] = {
        "on",
        "off",
        "true",
        "false",
        "1",
        "0",
        "high",
        "low"
    };

#define BOOL_STRINGS_COUNT sizeof(bool_strings) / sizeof(bool_strings[0])

    int entry_num = -1;
    size_t nbytes = parse_string_in_table(buf, buf_nbytes, (char **)bool_strings, BOOL_STRINGS_COUNT, &entry_num);
    if (nbytes > 0) {
        if ((entry_num & 1) == 0)
            *out_bool = true;
        else
            *out_bool = false;
    }

    return nbytes;
}

size_t parse_char(char *buf, size_t buf_nbytes, char *out_char)
{
    assert(buf);
    assert(out_char);

    if (buf_nbytes == 0)
        return 0;

    if ((*buf < ' ') || (*buf >= 127))
        return 0;

    *out_char = *buf;
    return 1;
}

size_t parse_int(char *buf, size_t buf_nbytes, int *out_int)
{
    assert(buf);
    assert(out_int);

    long int li = -1;
    size_t nbytes = parse_long_int(buf, buf_nbytes, &li);

    if (nbytes == 0)
        return 0;

    *out_int = (int)li;
    if (*out_int != li) {
        LOG_ERROR(F("Overflow. %ld is larger than maximum integer of %d."), li, INT_MAX);
        return 0;
    }

    return nbytes;
}

size_t parse_long_int(char *buf, size_t buf_nbytes, long int *out_int)
{
    assert(buf);
    assert(out_int);
    int sign = 1;
    bool is_valid = false;
    char *p = buf;
    long int i = 0;

    size_t nbytes = parse_whitespace(p, buf_nbytes);

    buf_nbytes -= nbytes;
    p += nbytes;

    while (*p && (p - buf < buf_nbytes)) {
        if (isspace(*p))
            break;

        if ((*p == '-') && ((p - buf) == 0)) {
            sign = -1.0f;
        } else if ((*p == '+') && ((p - buf) == 0)) {
            sign = 1.0f;
        } else if (isdigit(*p)) {
            is_valid = true;
            i *= 10;
            i += (*p - '0');
        } else {
            is_valid = false;
            break;
        }

        p++;
    }

    nbytes = parse_whitespace(p, buf_nbytes);
    buf_nbytes -= nbytes;
    p += nbytes;

    if (!is_valid)
        return 0;

    *out_int = i * sign;
    return p - buf;
}

size_t parse_float(char *buf, size_t buf_nbytes, float *out_float)
{
    assert(buf);
    assert(out_float);
    *out_float = 0.0f;

    bool is_reading_significand = true;
    bool is_valid = false;
    float sign = 1.0f;
    float f = 0;
    float fraction_div = 10.0f;
    char *p = buf;

    size_t nbytes = parse_whitespace(p, buf_nbytes);

    buf_nbytes -= nbytes;
    p += nbytes;

    while (*p && (p - buf < buf_nbytes)) {
        if (isspace(*p)) {
            break;
        } else if (is_reading_significand) {
            if ((*p == '-') && ((p - buf) == 0)) {
                sign = -1.0f;
            } else if ((*p == '+') && ((p - buf) == 0)) {
                sign = 1.0f;
            } else if (*p == '.') {
                is_reading_significand = false;
            } else if (isdigit(*p)) {
                is_valid = true;
                f *= 10;
                f += (*p - '0');
            } else {
                is_valid = false;
                break;
            }
        } else {
            if (isdigit(*p)) {
                f += (*p - '0') / fraction_div;
                fraction_div *= 10;
            } else {
                is_valid = false;
                break;
            }
        }

        p++;
    }

    nbytes = parse_whitespace(p, buf_nbytes);
    buf_nbytes -= nbytes;
    p += nbytes;

    if (!is_valid)
        return 0;

    *out_float = sign * f;
    return p - buf;
}

typedef struct {
    char *string;
    int   expected_nbytes;
    float expected_value;
} parse_float_test_case;

static parse_float_test_case parse_float_test_case_by_index[] = {
    { "",       0, 0.0f    },
    { "0",      1, 0.0f    },
    { "1",      1, 1.0f    },
    { "+0",     2, 0.0f    },
    { "1.0",    3, 1.0f    },
    { "-1.0",   4, -1.0f   },
    { "1.1",    3, 1.1f    },
    { "-1.1",   4, -1.1f   },
    { "1.23",   4, 1.23f   },
    { "-3.456", 6, -3.456f },
};
#define PARSE_FLOAT_TEST_CASE_BY_INDEX_COUNT sizeof(parse_float_test_case_by_index) / sizeof(parse_float_test_case_by_index[0])

static bool test_parse_float()
{
    bool ret = true;

    for (int i = 0; i < PARSE_FLOAT_TEST_CASE_BY_INDEX_COUNT; i++) {
        parse_float_test_case *test_case = &parse_float_test_case_by_index[i];
        float f = -1.0f;
        size_t nbytes = parse_float(test_case->string, strlen(test_case->string), &f);
        if (test_case->expected_nbytes != nbytes) {
            LOG_ERROR(F("Expected nbytes=%d, but got %d"), test_case->expected_nbytes, nbytes);
            ret = false;
        }

        if ((nbytes != 0) && (test_case->expected_value != f)) {
            LOG_ERROR(F("Expected value=%f, but got %f"), test_case->expected_value, f);
            ret = false;
        }
    }

    return ret;
}

size_t parse_string(char *buf, size_t buf_nbytes, char *out_string, size_t out_string_nbytes)
{
    assert(buf);
    assert(out_string);
    bool is_valid = false;
    char *p = buf;

    size_t nbytes = parse_whitespace(p, buf_nbytes);

    buf_nbytes -= nbytes;
    p += nbytes;

    // This copies trailing whitespace.
    nbytes = strlen(p);

    if (nbytes >= out_string_nbytes) {
        log_writeln(F("ERROR: string too long."));
        return 0;
    }

    memcpy(out_string, buf, nbytes + 1);

    return nbytes;
}

size_t parse_string_in_table(char *buf, size_t buf_nbytes, char *table[], int ntable_entries, int *out_entry_num)
{
    assert(buf);
    assert(table);
    assert(out_entry_num);

    char *p = buf;

    size_t nbytes = parse_whitespace(p, buf_nbytes);

    buf_nbytes -= nbytes;
    p += nbytes;

    char *end = p + buf_nbytes;

    for (int i = 0; i < ntable_entries; i++) {
        size_t nbytes = strlen(table[i]);
        if ((nbytes == (end - buf)) && (strncasecmp(table[i], buf, end - buf) == 0)) {
            *out_entry_num = i;
            return nbytes;
        }
    }

    return 0;
}

size_t parse_motor_ids(char *buf, size_t buf_nbytes, int *out_mask)
{
    assert(buf);
    assert(out_mask);

    char *p = buf;
    char c = 0;

    *out_mask = 0;

    size_t nbytes = parse_whitespace(p, buf_nbytes);

    buf_nbytes -= nbytes;
    p += nbytes;

    do {
        size_t n = parse_char(p, buf_nbytes - (p - buf), &c);
        if (n == 0)
            break;

        p += n;

        if (isspace(c))
            break;

        c = toupper(c);
        if ((c < 'A') || (c > 'F'))
            goto error;

        *out_mask |= 1 << (c - 'A');
    } while (1);

    nbytes = parse_whitespace(p, buf_nbytes);
    buf_nbytes -= nbytes;
    p += nbytes;

    return p - buf;

error:
    *out_mask = -1;
    log_writeln(F("ERROR: Invalid motor ID %c. Expected A-%c."), c, 'A' + MOTOR_ID_COUNT - 1);
    return 0;
}

size_t parse_motor_id(char *buf, size_t buf_nbytes, motor_id_t *out_motor_id)
{
    assert(buf);
    assert(out_motor_id);
    char *p = buf;

    size_t nbytes = parse_whitespace(p, buf_nbytes);

    buf_nbytes -= nbytes;
    p += nbytes;

    char id;

    if (parse_char(p, buf_nbytes, &id) == 0)
        goto error;
    p++;
    buf_nbytes--;

    nbytes = parse_whitespace(p, buf_nbytes);
    buf_nbytes -= nbytes;
    p += nbytes;

    id = toupper(id);
    if ((id < 'A') || (id > 'F'))
        goto error;

    *out_motor_id = (motor_id_t)(id - 'A');
    return p - buf;

error:
    log_writeln(F("ERROR: Invalid motor ID. Expected 'A'-'%c'."), 'A' + MOTOR_ID_COUNT - 1);
    return 0;
}

size_t parse_motor_position(char *buf, size_t buf_nbytes, float *out_value)
{
    assert(buf);
    assert(out_value);
    char *p = buf;

    size_t nbytes = parse_whitespace(p, buf_nbytes);

    buf_nbytes -= nbytes;
    p += nbytes;

    float new_value = 0.0f;

    nbytes = parse_float(p, buf_nbytes, &new_value);

    if (nbytes > 0) {
        *out_value = new_value;
        buf_nbytes -= nbytes;
        p += nbytes;
    } else if (nbytes <= 0) {
        new_value = *out_value;
        // Not a float, check for +/++/-/--.
        char first_plus_or_minus = '\0';
        nbytes = parse_char(p, buf_nbytes, &first_plus_or_minus);
        buf_nbytes -= nbytes;
        p += nbytes;
        if ((nbytes <= 0) || ((first_plus_or_minus != '+') && (first_plus_or_minus != '-')))
            goto error;                // Something other than a + or -

        char second_plus_or_minus = '\0';
        nbytes = parse_char(p, buf_nbytes, &second_plus_or_minus);
        buf_nbytes -= nbytes;
        p += nbytes;

        if (nbytes == 0) {
            if (first_plus_or_minus == '+')
                new_value += 10;
            else
                new_value -= 10;
        } else if (nbytes > 0) {
            if ((second_plus_or_minus != '+') && (second_plus_or_minus != '-'))
                goto error;            // Something other than a + or -

            if (first_plus_or_minus != second_plus_or_minus)
                goto error;            // Not ++ or --

            if (first_plus_or_minus == '+')
                new_value += 100;
            else
                new_value -= 100;
        }
    }

    *out_value = new_value;

    nbytes = parse_whitespace(p, buf_nbytes);
    buf_nbytes -= nbytes;
    p += nbytes;

    return p - buf;

error:
    log_writeln(F("ERROR: Invalid floating point number. Expected [+/-]digits.digits."));
    return 0;
}

bool parse_test()
{
    float ret = true;

    // TODO: Implement parse_test().
    // int parse_bool(buf, nbytes, out_bool) <= true/false/on/off/1/0
    // int parse_char(buf, nbytes, out_char) <= returns 1 if > 32 && < 127
    // parse_motor_angle_or_encoder

    ret = test_parse_float() ? ret : false;

    return ret;
}

size_t parse_waypoint(char *buf, size_t buf_nbytes, waypoint_t *out_waypoint)
{
    assert(buf);
    assert(out_waypoint);
    char *p = buf;

    const long int max_wait_millis = 86400000;  // 24 * 60 * 60 * 1000.
    size_t nbytes = parse_whitespace(p, buf_nbytes);

    buf_nbytes -= nbytes;
    p += nbytes;

    int entry_num = -1;
    char *command_table[] = { "A", "B", "C", "D", "E", "G", "J", "K", "L", "O", "P", "W" };
    const int command_table_nentries = 12;
    int goto_step = -1;
    int gpio_pin = -1;
    bool is_high = false;
    long int millis = -1;
    int enabled_motors_mask = -1;

    nbytes = parse_string_in_table(p, 1, command_table, command_table_nentries, &entry_num);
    buf_nbytes -= nbytes;
    p += nbytes;

    if ((entry_num < 0) || (entry_num >= command_table_nentries))
        goto error;

    nbytes = parse_whitespace(p, buf_nbytes);
    buf_nbytes -= nbytes;
    p += nbytes;

    switch (command_table[entry_num][0]) {
    case 'A':  // Move motors to exactly current positions.
        if (buf_nbytes != 0)
            goto error;
        out_waypoint->command = WAYPOINT_COMMAND_MOVE_AT;
        for (int i = 0; i < MOTOR_ID_COUNT; i++) {
            out_waypoint->motor[i] = motor_get_encoder((motor_id_t)i);
        }
        break;
    case 'B':  // Move motors to within 1 encoder value of current positions.
        if (buf_nbytes != 0)
            goto error;
        out_waypoint->command = WAYPOINT_COMMAND_MOVE_BESIDE;
        for (int i = 0; i < MOTOR_ID_COUNT; i++) {
            out_waypoint->motor[i] = motor_get_encoder((motor_id_t)i);
        }
        break;
    case 'C':  // Move motors to within 30 encoder values of current positions.
        if (buf_nbytes != 0)
            goto error;
        out_waypoint->command = WAYPOINT_COMMAND_MOVE_CLOSE;
        for (int i = 0; i < MOTOR_ID_COUNT; i++) {
            out_waypoint->motor[i] = motor_get_encoder((motor_id_t)i);
        }
        break;
    case 'D':  // Move motors to within 200 encoder values current positions.
        if (buf_nbytes != 0)
            goto error;
        out_waypoint->command = WAYPOINT_COMMAND_MOVE_APPROACHING;
        for (int i = 0; i < MOTOR_ID_COUNT; i++) {
            out_waypoint->motor[i] = motor_get_encoder((motor_id_t)i);
        } \
        break;
    case 'E':  // Set enabled motors.
        nbytes = parse_motor_ids(p, buf_nbytes, &enabled_motors_mask);
        buf_nbytes -= nbytes;
        p += nbytes;

        if (buf_nbytes != 0)
            goto error;
        out_waypoint->command = WAYPOINT_COMMAND_SET_ENABLED_MOTORS;
        out_waypoint->enabled_motors_mask = enabled_motors_mask;
        break;
    case 'G':  // Goto step.
        nbytes = parse_int(p, buf_nbytes, &goto_step);
        if (nbytes == 0)
            goto error;
        buf_nbytes -= nbytes;
        p += nbytes;

        if (goto_step >= waypoint_get_max_count()) {
            log_writeln(F("ERROR: Goto index greater than max index %d."), waypoint_get_max_count());
            goto error;
        }

        if (buf_nbytes != 0)
            goto error;

        out_waypoint->command = WAYPOINT_COMMAND_GOTO_STEP;
        out_waypoint->io_goto.step = goto_step;
        break;
    case 'J':  // If GPIO pin value, goto step.
        nbytes = parse_int(p, buf_nbytes, &gpio_pin);
        if (nbytes == 0)
            goto error;
        buf_nbytes -= nbytes;
        p += nbytes;

        if (gpio_pin < 0 || gpio_pin >= HARDWARE_GPIO_PIN_COUNT) {
            log_writeln(F("ERROR: Invalid GPIO pin number %d. Pin number should be in the range [0, %d]."), gpio_pin, HARDWARE_GPIO_PIN_COUNT - 1);
            goto error;
        }

        if (hardware_get_gpio_pin_mode(gpio_pin) == HARDWARE_GPIO_PIN_MODE_OUTPUT) {
            log_writeln(F("ERROR: GPIO pin %d is is not configured for input."), gpio_pin);
            goto error;
        }

        nbytes = parse_bool(p, buf_nbytes, &is_high);
        if (nbytes == 0)
            goto error;
        buf_nbytes -= nbytes;
        p += nbytes;

        nbytes = parse_int(p, buf_nbytes, &goto_step);
        if (nbytes == 0)
            goto error;
        buf_nbytes -= nbytes;
        p += nbytes;

        if (goto_step >= waypoint_get_max_count()) {
            log_writeln(F("ERROR: Goto index greater than max index %d."), waypoint_get_max_count());
            goto error;
        }

        if (buf_nbytes != 0)
            goto error;

        out_waypoint->command = WAYPOINT_COMMAND_IF_GPIO_PIN_GOTO_STEP;
        out_waypoint->io_goto.gpio_pin = gpio_pin | (is_high ? 0x8000 : 0);
        out_waypoint->io_goto.step = goto_step;
        break;
    case 'K':  // Wait for GPIO pin value.
        nbytes = parse_int(p, buf_nbytes, &gpio_pin);
        if (nbytes == 0)
            goto error;
        buf_nbytes -= nbytes;
        p += nbytes;

        if (gpio_pin < 0 || gpio_pin >= HARDWARE_GPIO_PIN_COUNT) {
            log_writeln(F("ERROR: Invalid GPIO pin number %d. Pin number should be in the range [0, %d]."), gpio_pin, HARDWARE_GPIO_PIN_COUNT - 1);
            goto error;
        }

        if (hardware_get_gpio_pin_mode(gpio_pin) == HARDWARE_GPIO_PIN_MODE_OUTPUT) {
            log_writeln(F("ERROR: GPIO pin %d is is not configured for input."), gpio_pin);
            goto error;
        }

        if (goto_step >= waypoint_get_max_count()) {
            log_writeln(F("ERROR: Goto index greater than max index %d."), waypoint_get_max_count());
            goto error;
        }

        nbytes = parse_bool(p, buf_nbytes, &is_high);
        if (nbytes == 0)
            goto error;
        buf_nbytes -= nbytes;
        p += nbytes;

        if (buf_nbytes != 0)
            goto error;

        out_waypoint->command = WAYPOINT_COMMAND_WAIT_GPIO_PIN;
        out_waypoint->io_goto.gpio_pin = gpio_pin | (is_high ? 0x8000 : 0);
        break;
    case 'L':  // Calibrate home switches and limits.
        nbytes = parse_motor_ids(p, buf_nbytes, &enabled_motors_mask);
        buf_nbytes -= nbytes;
        p += nbytes;

        if (buf_nbytes != 0)
            goto error;
        out_waypoint->command = WAYPOINT_COMMAND_CALIBRATE_HOME_SWITCHES_AND_LIMITS;
        out_waypoint->enabled_motors_mask = enabled_motors_mask;
        break;
    case 'O':  // calibrate home switches.
        nbytes = parse_motor_ids(p, buf_nbytes, &enabled_motors_mask);
        buf_nbytes -= nbytes;
        p += nbytes;

        if (buf_nbytes != 0)
            goto error;
        out_waypoint->command = WAYPOINT_COMMAND_CALIBRATE_HOME_SWITCHES;
        out_waypoint->enabled_motors_mask = enabled_motors_mask;
        break;
    case 'P':  // Set GPIO pin output.
        nbytes = parse_int(p, buf_nbytes, &gpio_pin);
        if (nbytes == 0)
            goto error;

        buf_nbytes -= nbytes;
        p += nbytes;

        if (gpio_pin < 0 || gpio_pin >= HARDWARE_GPIO_PIN_COUNT) {
            log_writeln(F("ERROR: Invalid GPIO pin number %d. Pin number should be in the range [0, %d]."), gpio_pin, HARDWARE_GPIO_PIN_COUNT - 1);
            goto error;
        }

        if (hardware_get_gpio_pin_mode(gpio_pin) != HARDWARE_GPIO_PIN_MODE_OUTPUT) {
            log_writeln(F("ERROR: GPIO pin %d is is not configured for output."), gpio_pin);
            goto error;
        }

        nbytes = parse_bool(p, buf_nbytes, &is_high);
        if (nbytes == 0)
            goto error;

        buf_nbytes -= nbytes;
        p += nbytes;

        out_waypoint->command = WAYPOINT_COMMAND_SET_GPIO_PIN_OUTPUT;
        if (is_high)
            gpio_pin |= 0x8000;
        out_waypoint->io_goto.gpio_pin = gpio_pin;
        break;
    case 'W':  // Wait milliseconds.
        nbytes = parse_long_int(p, buf_nbytes, &millis);
        if (nbytes == 0)
            goto error;

        buf_nbytes -= nbytes;
        p += nbytes;

        if ((millis < 0) || (millis > max_wait_millis)) {
            log_writeln(F("Invalid wait of %ld milliseconds. Maximum wait time is 24 hours (24 * 60 * 60 * 1000 = %ld)."), millis, max_wait_millis);
            goto error;
        }
        if (buf_nbytes != 0)
            goto error;
        out_waypoint->command = WAYPOINT_COMMAND_WAIT_MILLIS;
        out_waypoint->wait_millis = millis;
        break;
    default:
        assert(false);
    }

    nbytes = parse_whitespace(p, buf_nbytes);
    buf_nbytes -= nbytes;
    p += nbytes;

    return p - buf;

error:
    log_writeln(F("ERROR: Cannot parse waypoint command."));
    return 0;
}
