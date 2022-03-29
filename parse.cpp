/*
 * Implementation for input parsing functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#define __ASSERT_USE_STDERR
#include "log.h"
#include "motor.h"
#include "parse.h"

size_t parse_whitespace(char *pbuf, size_t buf_nbytes)
{
    assert(pbuf);

    char *p = pbuf;

    while (isspace(*p)) {
        p++;
    }

    return p - pbuf;
}

size_t parse_bool(char *pbuf, size_t buf_nbytes, bool *pout_bool)
{
    assert(pbuf);
    assert(pout_bool);

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
    size_t nbytes = parse_string_in_table(pbuf, buf_nbytes, bool_strings, BOOL_STRINGS_COUNT, &entry_num);
    if (nbytes > 0) {
        if ((entry_num & 1) == 0)
            *pout_bool = true;
        else
            *pout_bool = false;
    }

    return nbytes;
}

size_t parse_char(char *pbuf, size_t buf_nbytes, char *pout_char)
{
    assert(pbuf);
    assert(pout_char);

    if (buf_nbytes == 0)
        return 0;

    if ((*pbuf <= ' ') || (*pbuf >= 127))
        return 0;

    *pout_char = *pbuf;
    return 1;
}

size_t parse_int(char *pbuf, size_t buf_nbytes, int *pout_int)
{
    assert(pbuf);
    assert(pout_int);
    int sign = 1;
    bool is_valid = false;
    char *p = pbuf;
    int i = 0;

    while (*p && (p - pbuf < buf_nbytes)) {
        if (isspace(*p))
            break;

        if ((*p == '-') && ((p - pbuf) == 0)) {
            sign = -1.0f;
        } else if ((*p == '+') && ((p - pbuf) == 0)) {
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

    *pout_int = i * sign;
    return p - pbuf;
}

size_t parse_float(char *pbuf, size_t buf_nbytes, float *pout_float)
{
    assert(pbuf);
    assert(pout_float);
    *pout_float = 0.0f;

    bool is_reading_significand = true;
    bool is_valid = false;
    float sign = 1.0f;
    float f = 0;
    float fraction_div = 10.0f;
    char *p = pbuf;

    while (*p && (p - pbuf < buf_nbytes)) {
        if (isspace(*p)) {
            break;
        } else if (is_reading_significand) {
            if ((*p == '-') && ((p - pbuf) == 0)) {
                sign = -1.0f;
            } else if ((*p == '+') && ((p - pbuf) == 0)) {
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

    *pout_float = sign * f;
    return p - pbuf;
}

typedef struct {
    char *pstring;
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
        parse_float_test_case *ptest_case = &parse_float_test_case_by_index[i];
        float f = -1.0f;
        size_t nbytes = parse_float(ptest_case->pstring, strlen(ptest_case->pstring), &f);
        if (ptest_case->expected_nbytes != nbytes) {
            LOG_ERROR(F("Expected nbytes=%d, but got %d"), ptest_case->expected_nbytes, nbytes);
            ret = false;
        }

        if ((nbytes != 0) && (ptest_case->expected_value != f)) {
            LOG_ERROR(F("Expected value=%f, but got %f"), ptest_case->expected_value, f);
            ret = false;
        }
    }

    return ret;
}

size_t parse_string(char *pbuf, size_t buf_nbytes, char *pout_string, size_t out_string_nbytes)
{
    assert(pbuf);
    assert(pout_string);
    bool is_valid = false;
    char *p = pbuf;

    size_t nbytes = strlen(pbuf);

    if (nbytes >= out_string_nbytes) {
        log_writeln(F("ERROR: string too long."));
        return 0;
    }

    memcpy(pout_string, pbuf, nbytes + 1);

    return nbytes;
}

size_t parse_string_in_table(char *pbuf, size_t buf_nbytes, char *ptable[], int ntable_entries, int *pout_entry_num)
{
    assert(pbuf);
    assert(ptable);
    assert(pout_entry_num);

    char *pend = pbuf + buf_nbytes;

    for (int i = 0; i < ntable_entries; i++) {
        size_t nbytes = strlen(ptable[i]);
        if (nbytes == (pend - pbuf) &&
            (strncasecmp(ptable[i], pbuf, pend - pbuf) == 0)) {
            *pout_entry_num = i;
            return nbytes;
        }
    }

    return 0;
}

size_t parse_motor_id(char *pbuf, size_t buf_nbytes, motor_id_t *pout_motor_id)
{
    assert(pbuf);
    assert(pout_motor_id);

    char id;

    if (parse_char(pbuf, buf_nbytes, &id) == 0)
        goto error;

    id = toupper(id);
    if ((id < 'A') || (id > 'F'))
        goto error;

    *pout_motor_id = id - 'A';
    return 1;

error:
    log_writeln(F("ERROR: Invalid motor ID. Expected 'A'-'%c'."), 'A' + MOTOR_ID_LAST);
    return 0;
}

size_t parse_motor_angle_or_encoder(char *pargs, size_t args_nbytes, float *pvalue)
{
    assert(pargs);
    assert(pvalue);
    char *p = pargs;

    float new_value = 0.0f;
    size_t nbytes = parse_float(p, args_nbytes, &new_value);

    if (nbytes > 0) {
        *pvalue = new_value;
        args_nbytes -= nbytes;
        p += nbytes;
    } else if (nbytes == 0) {
        new_value = *pvalue;
        // Not a float, check for +/++/-/--.
        char first_plus_or_minus = '\0';
        nbytes = parse_char(p, args_nbytes, &first_plus_or_minus);
        args_nbytes -= nbytes;
        p += nbytes;
        if ((nbytes == 0) || ((first_plus_or_minus != '+') && (first_plus_or_minus != '-')))
            goto error;                // Something other than a + or -

        char second_plus_or_minus = '\0';
        nbytes = parse_char(p, args_nbytes, &second_plus_or_minus);
        args_nbytes -= nbytes;
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

    *pvalue = new_value;

    return p - pargs;

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
