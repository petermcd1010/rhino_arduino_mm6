/*
 * Implementation for input parsing functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#define __ASSERT_USE_STDERR
#include "log.h"
#include "motor.h"
#include "parse.h"
#include "waypoint.h"  // WAYPOINT_COMMAND_*. TODO: sort out config_waypoint_t vs waypoint_command, etc..

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

    const char *bool_strings[] = {
        "on",
        "off",
        "true",
        "false",
        "1",
        "0",
    };

#define BOOL_STRINGS_COUNT sizeof(bool_strings) / sizeof(bool_strings[0])

    int entry_num = -1;
    size_t nbytes = parse_string_in_table(buf, buf_nbytes, bool_strings, BOOL_STRINGS_COUNT, &entry_num);
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
    int sign = 1;
    bool is_valid = false;
    char *p = buf;
    int i = 0;

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
            }
        } else {
            if (isdigit(*p)) {
                f += (*p - '0') / fraction_div;
                fraction_div *= 10;
            } else {
                is_valid = false;
            }
        }

        p++;
    }

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

    size_t nbytes = strlen(buf);

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

    char *end = buf + buf_nbytes;

    for (int i = 0; i < ntable_entries; i++) {
        size_t nbytes = strlen(table[i]);
        if (nbytes == (end - buf) &&
            (strncasecmp(table[i], buf, end - buf) == 0)) {
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

    do {
        size_t n = parse_char(p, buf_nbytes - (p - buf), &c);
        if (n == 0)
            break;

        p += n;

        if (isspace(c))
            continue;

        c = toupper(c);
        if ((c < 'A') || (c > 'F'))
            goto error;

        *out_mask |= 1 << (c - 'A');
    } while (1);

    return p - buf;

error:
    *out_mask = -1;
    log_writeln(F("ERROR: Invalid motor ID %c. Expected A-%c."), c, 'A' + MOTOR_ID_LAST);
    return 0;
}

size_t parse_motor_id(char *buf, size_t buf_nbytes, motor_id_t *out_motor_id)
{
    assert(buf);
    assert(out_motor_id);

    char id;

    if (parse_char(buf, buf_nbytes, &id) == 0)
        goto error;

    id = toupper(id);
    if ((id < 'A') || (id > 'F'))
        goto error;

    *out_motor_id = (motor_id_t)(id - 'A');
    return 1;

error:
    log_writeln(F("ERROR: Invalid motor ID. Expected 'A'-'%c'."), 'A' + MOTOR_ID_LAST);
    return 0;
}

size_t parse_motor_angle_or_encoder(char *buf, size_t buf_nbytes, float *out_value)
{
    assert(buf);
    assert(out_value);
    char *p = buf;

    float new_value = 0.0f;
    size_t nbytes = parse_float(p, buf_nbytes, &new_value);

    if (nbytes > 0) {
        *out_value = new_value;
        buf_nbytes -= nbytes;
        p += nbytes;
    } else if (nbytes == 0) {
        new_value = *out_value;
        // Not a float, check for +/++/-/--.
        char first_plus_or_minus = '\0';
        nbytes = parse_char(p, buf_nbytes, &first_plus_or_minus);
        buf_nbytes -= nbytes;
        p += nbytes;
        if ((nbytes == 0) || ((first_plus_or_minus != '+') && (first_plus_or_minus != '-')))
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

static size_t parse_waypoint_motors(char *buf, size_t buf_nbytes, config_waypoint_t *out_waypoint)
{
    assert(buf);
    assert(out_waypoint);

    size_t nbytes = parse_float(buf, buf_nbytes, &out_waypoint->motor.a);

    if (nbytes == 0)
        goto error;
    buf += nbytes;
    buf_nbytes -= nbytes;

    nbytes = parse_float(buf, buf_nbytes, &out_waypoint->motor.b);
    if (nbytes == 0)
        goto error;
    buf += nbytes;
    buf_nbytes -= nbytes;

    nbytes = parse_float(buf, buf_nbytes, &out_waypoint->motor.c);
    if (nbytes == 0)
        goto error;
    buf += nbytes;
    buf_nbytes -= nbytes;

    nbytes = parse_float(buf, buf_nbytes, &out_waypoint->motor.d);
    if (nbytes == 0)
        goto error;
    buf += nbytes;
    buf_nbytes -= nbytes;

    nbytes = parse_float(buf, buf_nbytes, &out_waypoint->motor.e);
    if (nbytes == 0)
        goto error;
    buf += nbytes;
    buf_nbytes -= nbytes;

    nbytes = parse_float(buf, buf_nbytes, &out_waypoint->motor.f);
    if (nbytes == 0)
        goto error;
    buf += nbytes;
    buf_nbytes -= nbytes;

error:
    log_writeln(F("ERROR: Failed to process waypoint motor positions."));
    return 0;
}

size_t parse_waypoint(char *buf, size_t buf_nbytes, config_waypoint_t *out_waypoint)
{
    assert(buf);
    assert(out_waypoint);
    char *p = buf;

    const char *command_strings[] = {
        "a",  // WAYPOINT_COMMAND_MOVE_AT.
        "b",  // WAYPOINT_COMMAND_MOVE_BESIDE.
        "c",  // WAYPOINT_COMMAND_MOVE_CLOSE.
        "d",  // WAYPOINT_COMMAND_MOVE_APPROACHING.
        "g",  // WAYPOINT_COMMAND_GOTO_STEP.
        "j",  // WAYPOINT_COMMAND_GOTO_STEP_IF_IO.
        "i",  // WAYPOINT_COMMAND_INTERROGATE_SWITCHES.
        "w",  // WAYPOINT_COMMAND_WAIT_MILLIS.
    };

#define COMMAND_STRINGS_COUNT sizeof(command_strings) / sizeof(command_strings[0])

    int entry_num = -1;
    size_t nbytes = parse_string_in_table(p, buf_nbytes, command_strings, COMMAND_STRINGS_COUNT, &entry_num);
    if (nbytes = 0) {
        log_writeln(F("Error processing waypoint.")); // TODO: Test and remove.
        return 0;
    }

    buf_nbytes -= nbytes;
    p += nbytes;

    switch (entry_num) {
    case 0:
        out_waypoint->command = WAYPOINT_COMMAND_MOVE_AT;
        nbytes = parse_waypoint_motors(p, buf_nbytes, out_waypoint);
        break;
    case 1:
        out_waypoint->command = WAYPOINT_COMMAND_MOVE_BESIDE;
        nbytes = parse_waypoint_motors(p, buf_nbytes, out_waypoint);
        break;
    case 2:
        out_waypoint->command = WAYPOINT_COMMAND_MOVE_CLOSE;
        nbytes = parse_waypoint_motors(p, buf_nbytes, out_waypoint);
        break;
    case 3:
        out_waypoint->command = WAYPOINT_COMMAND_MOVE_APPROACHING;
        nbytes = parse_waypoint_motors(p, buf_nbytes, out_waypoint);
        break;
    case 4:
        out_waypoint->command = WAYPOINT_COMMAND_GOTO_STEP;
        nbytes = parse_int(p, buf_nbytes, &out_waypoint->goto_step);
        break;
    case 5:
        out_waypoint->command = WAYPOINT_COMMAND_GOTO_STEP_IF_IO;
        nbytes = parse_int(p, buf_nbytes, &out_waypoint->goto_step);
        break;
    case 6:
        out_waypoint->command = WAYPOINT_COMMAND_INTERROGATE_SWITCHES;
        nbytes = 0;
        break;
    case 7:
        out_waypoint->command = WAYPOINT_COMMAND_WAIT_MILLIS;
        nbytes = parse_int(p, buf_nbytes, &out_waypoint->wait_millis);
        break;
    default:
        assert(false);
        break;
    }

    buf_nbytes -= nbytes;
    p += nbytes;

    nbytes = parse_whitespace(p, buf_nbytes);

    buf_nbytes -= nbytes;
    p += nbytes;

    return p - buf;
}
