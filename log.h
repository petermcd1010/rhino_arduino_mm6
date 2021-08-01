#pragma once

#include <Arduino.h>

#define LOG_D(format, args ...) do { log_debug(F(__FILE__), __LINE__, __FUNCTION__, format, ## args); } while (0)
#define LOG_E(format, args ...) do { log_error(F(__FILE__), __LINE__, __FUNCTION__, format, ## args); } while (0)

void log_error(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...);
