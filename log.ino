/*
 * Logging functions and assert implementation.
 */

// TODO: There's some repeated code here that can be reduced in size.

#include "log.h"

const int log_progmem_copy_buffer_nbytes = 256;

void log_string(const char *pstring)
{
  Serial.print(pstring);
}

void log_string(const __FlashStringHelper *pstring)
{
  assert(pstring);

  char buffer[log_progmem_copy_buffer_nbytes];
  strncpy_P(buffer, (char*)pstring, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) != log_progmem_copy_buffer_nbytes - 1);

  log_string(buffer);
}

void log_format_str_va_list(const char *pformat, va_list args) 
{
  assert(pformat);

  char buffer[log_progmem_copy_buffer_nbytes];
  vsnprintf(buffer, log_progmem_copy_buffer_nbytes, pformat, args);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';  // Force null-termination.
  assert(strlen(buffer) != log_progmem_copy_buffer_nbytes - 1);

  log_string(buffer);
}

void log_write(const __FlashStringHelper *pformat, ...)
{
  assert(pformat);
  va_list args;

  char buffer[log_progmem_copy_buffer_nbytes];
  strncpy_P(buffer, (char*)pformat, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) != log_progmem_copy_buffer_nbytes - 1);

  va_start(args, pformat);
  log_format_str_va_list(buffer, args);
  va_end(args);
}

void log_writeln()
{
  log_string(F("\n\r"));
}

void log_writeln(const __FlashStringHelper *pformat, ...)
{
  assert(pformat);
  va_list args;

  char buffer[log_progmem_copy_buffer_nbytes];
  strncpy_P(buffer, (char*)pformat, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) != log_progmem_copy_buffer_nbytes - 1);

  va_start(args, pformat);
  log_format_str_va_list(buffer, args);
  va_end(args);

  log_writeln();
}

static void print_file_name(const __FlashStringHelper *pfile_path)
{
  assert(pfile_path);

  // Find the last '/' in the pfile_path.
  void *pfile_name = pfile_path;
  while (pgm_read_byte(pfile_path)) {
    if (pgm_read_byte(pfile_path) == '/')
      pfile_name = (char*)pfile_path + 1;
    pfile_path = (const __FlashStringHelper*)((char*)pfile_path + 1);
  }

  // Print the file name.
  char buffer[log_progmem_copy_buffer_nbytes];
  strncpy_P(buffer, (char*)pfile_name, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) != log_progmem_copy_buffer_nbytes - 1);
  log_string(buffer);
}

void log_error_internal(int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, va_list args)
{
  assert(line_num > 0);
  assert(pfunction_name);
  assert(pformat);

  char buffer[log_progmem_copy_buffer_nbytes];
  snprintf(buffer, log_progmem_copy_buffer_nbytes, ":%d:%s: ERROR: ", line_num, pfunction_name);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  log_string(buffer);

  strncpy_P(buffer, (char*)pformat, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) != log_progmem_copy_buffer_nbytes - 1);

  log_format_str_va_list(buffer, args);

  log_string(F("\n\r"));
  state = STATE_ERROR;
}

void log_error(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...)
{
  assert(pfile_path);
  assert(line_num > 0);
  assert(pfunction_name);
  assert(pformat);

  print_file_name(pfile_path);

  va_list args;
  va_start(args, pformat);
  log_error_internal(line_num, pfunction_name, pformat, args);
  va_end(args);
}

void log_debug_internal(int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, va_list args)
{
  assert(line_num > 0);
  assert(pfunction_name);
  assert(pformat);

  char buffer[log_progmem_copy_buffer_nbytes];
  snprintf(buffer, log_progmem_copy_buffer_nbytes, ":%d:%s: ", line_num, pfunction_name);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) != log_progmem_copy_buffer_nbytes - 1);
  log_string(buffer);

  strncpy_P(buffer, (char*)pformat, log_progmem_copy_buffer_nbytes);
  buffer[log_progmem_copy_buffer_nbytes - 1] = '\0';
  assert(strlen(buffer) != log_progmem_copy_buffer_nbytes - 1);
  log_format_str_va_list(buffer, args);

  log_string(F("\n\r"));
}

void log_debug(const __FlashStringHelper *pfile_path, int line_num, const char *pfunction_name, const __FlashStringHelper *pformat, ...)
{
  assert(pfile_path);
  assert(line_num > 0);
  assert(pfunction_name);
  assert(pformat);

  print_file_name(pfile_path);

  va_list args;
  va_start(args, pformat);
  log_debug_internal(line_num, pfunction_name, pformat, args);
  va_end(args);
}

bool test_log() 
{
  // TODO.
  return true;
}

void __assert(const char *pfunction_name, const char *pfile_name, int line_num, const char *failedexpr)
{
  // Don't check for valid arguments, because this function is what the assert() macro calls.
  hardware_emergency_stop();

  log_string(pfunction_name);
  log_error_internal(line_num, pfunction_name, F("assertion failed: %s"), failedexpr);

  // TODO: Would it be better to write the assertion to persistent memory, reboot, and have it checked at boot?  
  state = STATE_ERROR;
}

