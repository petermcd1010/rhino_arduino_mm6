#pragma once

/*
 * Declarations for logging and assert functionality.
 */

#define __ASSERT_USE_STDERR

#include <Arduino.h>

#define LOG_D(format, args ...) do { log_debug(F(__FILE__), __LINE__, __FUNCTION__, format, ## args); } while (0)
#define LOG_E(format, args ...) do { log_error(F(__FILE__), __LINE__, __FUNCTION__, format, ## args); } while (0)

void log_write(const __FlashStringHelper *pformat, ...);
void log_writeln();
void log_writeln(const __FlashStringHelper *pformat, ...);
void log_error(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...);
void log_debug(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...);

bool test_log();
