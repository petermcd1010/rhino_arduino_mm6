#pragma once
/*
 * Declarations for logging and assert functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#define __ASSERT_USE_STDERR

#include <Arduino.h>

#define LOG_DEBUG(format, args ...) do { log_debug(F(__FILE__), __LINE__, __FUNCTION__, format, ## args); } while (0)
#define LOG_ERROR(format, args ...) do { log_error(F(__FILE__), __LINE__, __FUNCTION__, format, ## args); } while (0)

void log_write(const __FlashStringHelper *pformat, ...);
void log_writeln();
void log_writeln(const __FlashStringHelper *pformat, ...);
void log_error(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...);
void log_debug(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...);
void log_flush(void);

#ifdef assert
#undef assert
#endif
void log_assert(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, ...);
#define assert(EX) (void)((EX) || (log_assert(F(__FILE__), __LINE__, __FUNCTION__, #EX), 0))

bool log_test();
