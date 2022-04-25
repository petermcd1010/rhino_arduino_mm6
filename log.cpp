/*
 * Implementation for logging and assert functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>
#include "hardware.h"
#include "log.h"
#include "motor.h"

const int progmem_copy_buffer_nbytes = 256;

static void write_string_serial(const char *pstring)
{
    assert(pstring);
    Serial.print(pstring);
}

// This function pointer provides a way to confirm log output in log_test() below.
static void (*write_string_function)(const char *pstring) = write_string_serial;

static void write_string(const char *pstring)
{
    assert(pstring);
    write_string_function(pstring);
}

static void write_string(const __FlashStringHelper *pstring)
{
    assert(pstring);

    char buffer[progmem_copy_buffer_nbytes];

    strncpy_P(buffer, (char *)pstring, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);

    write_string_function(buffer);
}

static void log_format_string_va_list(const char *pformat, va_list args)
{
    assert(pformat);

    char buffer[progmem_copy_buffer_nbytes];

    vsnprintf(buffer, progmem_copy_buffer_nbytes, pformat, args);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);

    write_string(buffer);
}

void log_write(const __FlashStringHelper *pformat, ...)
{
    assert(pformat);
    va_list args;

    char buffer[progmem_copy_buffer_nbytes];

    strncpy_P(buffer, (char *)pformat, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);

    va_start(args, pformat);
    log_format_string_va_list(buffer, args);
    va_end(args);
}

void log_writeln()
{
    write_string(F("\n\r"));
}

void log_writeln(const __FlashStringHelper *pformat, ...)
{
    assert(pformat);
    va_list args;

    char buffer[progmem_copy_buffer_nbytes];

    strncpy_P(buffer, (char *)pformat, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);

    va_start(args, pformat);
    log_format_string_va_list(buffer, args);
    va_end(args);

    log_writeln();
}

static void write_file_name(const __FlashStringHelper *pfile_path)
{
    if (!pfile_path)
        return;

    // Find the last '/' in the pfile_path.
    void *pfile_name = pfile_path;

    while (pgm_read_byte(pfile_path)) {
        if (pgm_read_byte(pfile_path) == '/')
            pfile_name = (char *)pfile_path + 1;
        pfile_path = (const __FlashStringHelper *)((char *)pfile_path + 1);
    }

    // Write the file name.
    char buffer[progmem_copy_buffer_nbytes];

    strncpy_P(buffer, (char *)pfile_name, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);
    write_string(buffer);
}

static void log_internal(int line_num, const char *pfunction_name, bool is_error, const __FlashStringHelper *pformat, va_list args)
{
    assert(line_num > 0);
    assert(pfunction_name);
    assert(pformat);

    char buffer[progmem_copy_buffer_nbytes];

    snprintf(buffer, progmem_copy_buffer_nbytes, ":%d:%s:%s ", line_num, pfunction_name, is_error ?  "ERROR:" : "");
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    write_string(buffer);

    strncpy_P(buffer, (char *)pformat, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);

    log_format_string_va_list(buffer, args);

    write_string(F("\n\r"));
}

void log_error(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...)
{
    assert(pfile_path);
    assert(line_num > 0);
    assert(pfunction_name);
    assert(pformat);

    write_file_name(pfile_path);

    va_list args;

    va_start(args, pformat);
    log_internal(line_num, pfunction_name, true, pformat, args);
    va_end(args);
}

void log_debug(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...)
{
    assert(pfile_path);
    assert(line_num > 0);
    assert(pfunction_name);
    assert(pformat);

    write_file_name(pfile_path);

    va_list args;

    va_start(args, pformat);
    log_internal(line_num, pfunction_name, false, pformat, args);
    va_end(args);
}

void log_flush(void)
{
    Serial.flush();
}

void log_assert(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, ...)
{
    // Don't check for valid arguments, because this function is what the assert() macro calls.

    static bool in_assert = false;

    if (in_assert)
        return;                        // Avoid infinite assert() loops.
    in_assert = true;

    motor_disable_all();

    write_file_name(pfile_path);

    const __FlashStringHelper *pformat = F("assertion failed: %s");

    va_list args;

    va_start(args, line_num);
    log_internal(line_num, pfunction_name, true, pformat, args);
    va_end(args);
    log_flush();

    in_assert = false;
    // TODO: Write the assertion to persistent memory, reboot, and print it at boot.
    //   assert_get(), assert_clear()?

    hardware_halt();
}

bool log_test()
{
    // TODO: Override write_string_function() to check that logs output as expected.
    return true;
}
