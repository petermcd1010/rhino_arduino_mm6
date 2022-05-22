/*
 * Implementation for logging and assert functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>
#include "hardware.h"
#include "log.h"
#include "motor.h"

const int progmem_copy_buffer_nbytes = 256;

static void write_string_serial(const char *string)
{
    assert(string);
    Serial.print(string);
}

// This function pointer provides a way to confirm log output in log_test() below.
static void (*write_string_function)(const char *string) = write_string_serial;

static void write_string(const char *string)
{
    assert(string);
    write_string_function(string);
}

static void write_string(const __FlashStringHelper *string)
{
    assert(string);

    char buffer[progmem_copy_buffer_nbytes];

    strncpy_P(buffer, (char *)string, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);

    write_string_function(buffer);
}

static void log_format_string_va_list(const char *format, va_list args)
{
    assert(format);

    char buffer[progmem_copy_buffer_nbytes];

    vsnprintf(buffer, progmem_copy_buffer_nbytes, format, args);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);

    write_string(buffer);
}

void log_write(const __FlashStringHelper *format, ...)
{
    assert(format);
    va_list args;

    char buffer[progmem_copy_buffer_nbytes];

    strncpy_P(buffer, (char *)format, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);

    va_start(args, format);
    log_format_string_va_list(buffer, args);
    va_end(args);
}

void log_writeln()
{
    write_string(F("\n\r"));
}

void log_writeln(const __FlashStringHelper *format, ...)
{
    assert(format);
    va_list args;

    char buffer[progmem_copy_buffer_nbytes];

    strncpy_P(buffer, (char *)format, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);

    va_start(args, format);
    log_format_string_va_list(buffer, args);
    va_end(args);

    log_writeln();
}

static void write_file_name(const __FlashStringHelper *file_path)
{
    if (!file_path)
        return;

    // Find the last '/' in the pfile_path.
    void *file_name = file_path;

    while (pgm_read_byte(file_path)) {
        if (pgm_read_byte(file_path) == '/')
            file_name = (char *)file_path + 1;
        file_path = (const __FlashStringHelper *)((char *)file_path + 1);
    }

    // Write the file name.
    char buffer[progmem_copy_buffer_nbytes];

    strncpy_P(buffer, (char *)file_name, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);
    write_string(buffer);
}

static void log_internal_position(int line_num, const char *function_name, bool is_error)
{
    assert(line_num > 0);
    assert(function_name);

    char buffer[progmem_copy_buffer_nbytes];

    if (function_name)
        snprintf(buffer, progmem_copy_buffer_nbytes, ":%d:%s:%s ", line_num, function_name, is_error ? "ERROR:" : "");
    else
        snprintf(buffer, progmem_copy_buffer_nbytes, ":%d:%s ", line_num, is_error ? "ERROR:" : "");

    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    write_string(buffer);
}

static void log_internal(int line_num, const char *function_name, bool is_error, const __FlashStringHelper *format, va_list args)
{
    assert(line_num > 0);
    assert(function_name);
    assert(format);

    log_internal_position(line_num, function_name, is_error);

    char buffer[progmem_copy_buffer_nbytes];

    strncpy_P(buffer, (char *)format, progmem_copy_buffer_nbytes);
    buffer[progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
    assert(strlen(buffer) != progmem_copy_buffer_nbytes - 1);
    log_format_string_va_list(buffer, args);
    write_string(F("\n\r"));
}

void log_error(const __FlashStringHelper *file_path, int line_num, const char *function_name, const __FlashStringHelper *format, ...)
{
    assert(file_path);
    assert(line_num > 0);
    assert(function_name);
    assert(format);

    write_file_name(file_path);

    va_list args;

    va_start(args, format);
    log_internal(line_num, function_name, true, format, args);
    va_end(args);
}

void log_debug(const __FlashStringHelper *file_path, int line_num, const char *function_name, const __FlashStringHelper *format, ...)
{
    assert(file_path);
    assert(line_num > 0);
    assert(function_name);
    assert(format);

    write_file_name(file_path);

    va_list args;

    va_start(args, format);
    log_internal(line_num, function_name, false, format, args);
    va_end(args);
}

void log_flush(void)
{
    Serial.flush();
}

void log_assert(const __FlashStringHelper *file_path, int line_num, const char *function_name, ...)
{
    // Don't check for valid arguments, because this function is what the assert() macro calls.

    static bool in_assert = false;

    if (in_assert)
        return;                        // Avoid infinite assert() loops.
    in_assert = true;

    motor_disable_all();

    write_file_name(file_path);

    const __FlashStringHelper *format = F("assertion failed: %s");

    va_list args;

    va_start(args, line_num);
    log_internal(line_num, function_name, true, format, args);
    va_end(args);
    log_flush();

    in_assert = false;
    // TODO: Write the assertion to persistent memory, reboot, and print it at boot.
    //   assert_get(), assert_clear()?

    hardware_halt();
}

void log_assert_short(const __FlashStringHelper *file_path, int line_num)
{
    // The function name and va-args take up a significant amount of RAM. This version saves that RAM.

    // Don't check for valid arguments, because this function is what the assert() macro calls.

    static bool in_assert = false;

    if (in_assert)
        return;                        // Avoid infinite assert() loops.
    in_assert = true;

    motor_disable_all();

    write_file_name(file_path);

    const __FlashStringHelper *format = F("assertion failed.");

    va_list args;

    log_internal_position(line_num, NULL, true);
    write_string(F("\n\r"));
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
