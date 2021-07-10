/*
 * Logging functions and assert implementation.
 */

#include "log.h"

const int log_progmem_copy_buffer_nbytes = 256;

void log_string(const char *pstring)  // TODO: Add PROGMEM version.
{
  Serial.print(pstring);
}

void log_format_str_va_list(const char *pformat, va_list args) 
{
  assert(pformat);

  char buffer[log_progmem_copy_buffer_nbytes];
  vsnprintf(buffer, log_progmem_copy_buffer_nbytes, pformat, args);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
  assert(strlen(buffer) < log_progmem_copy_buffer_nbytes);

  log_string(buffer);
}

void log_write(const __FlashStringHelper *pformat, ...)
{
  assert(pformat);
  va_list args;

  char buffer[log_progmem_copy_buffer_nbytes];
  strncpy_P(buffer, (char*)pformat, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) < log_progmem_copy_buffer_nbytes);

  va_start(args, pformat);
  log_format_str_va_list(buffer, args);
  va_end(args);
}

void log_writeln(const __FlashStringHelper *pformat, ...)
{
  assert(pformat);
  va_list args;

  char buffer[log_progmem_copy_buffer_nbytes];
  strncpy_P(buffer, (char*)pformat, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) < log_progmem_copy_buffer_nbytes);

  va_start(args, pformat);
  log_format_str_va_list(buffer, args);
  va_end(args);

  log_string("\n\r");
}

static char* get_file_name_from_path(const char *pfile_path)
{
  assert(pfile_path);
  char *pfile_name = pfile_path;
  while (*pfile_path) {
    if (*pfile_path == '/')
      pfile_name = pfile_path + 1;
    pfile_path++;
  }
  return pfile_name;
}

void log_debug(const char *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...)
{
  assert(pfile_path);
  assert(line_num > 0);
  assert(pfunction_name);
  assert(pformat);

  va_list args;

  char buffer[log_progmem_copy_buffer_nbytes];
  snprintf(buffer, log_progmem_copy_buffer_nbytes, "%s:%d:%s: ", get_file_name_from_path(pfile_path), line_num, pfunction_name);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  // TODO: Assert no overflow.
  log_string(buffer);

  strncpy_P(buffer, (char*)pformat, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) < log_progmem_copy_buffer_nbytes);

  va_start(args, buffer);
  log_format_str_va_list(buffer, args);
  va_end(args);

  log_string("\n\r");
}

void log_error(const char* pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...)
{
  // TODO: pformat to PROGMEM
  assert(pfile_path);
  assert(line_num > 0);
  assert(pfunction_name);
  assert(pformat);

  va_list args;

  char buffer[log_progmem_copy_buffer_nbytes];
  snprintf(buffer, log_progmem_copy_buffer_nbytes, "%s:%d:%s: ERROR: ", get_file_name_from_path(pfile_path), line_num, pfunction_name);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  log_string(buffer);

  strncpy_P(buffer, (char*)pformat, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) < log_progmem_copy_buffer_nbytes);

  va_start(args, pformat);
  log_format_str_va_list(buffer, args);
  va_end(args);

  log_string("\n\r");
  state = STATE_ERROR;
}

bool test_log() 
{
  // TODO.
  return true;
}

void __assert(const char *pfunction_name, const char *pfile_name, int line_num, const char *failedexpr)
{
  hardware_emergency_stop();

  log_error(pfile_name, line_num, pfunction_name, F("assertion failed: %s"), failedexpr);
  state = STATE_ERROR;
}

