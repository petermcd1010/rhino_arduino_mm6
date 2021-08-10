/*
 * Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories.
 * Written by Scott Savage, Peter McDermott.
 * Feb 2017-21 GNU General Public License (GPL).
 *
 * Software change log at the end of file.
 *
 * See also:
 *   https://www.ti.com/lit/gpn/lmd18200 for the MegaMotor6's motor drivers.
 *
 * TODO:
 *   Analyze for JPL/etc. C coding rules.
 *   Run through a static analyzer?
 *   Fix all TODOs.
 */

#define __ASSERT_USE_STDERR
#include <EEPROM.h>
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>

#include "config.h"
#include "crc32c.h"
#include "hardware.h"
#include "log.h"
#include "menu.h"
#include "mm6.h"
#include "parse.h"
#include "sm.h"

// Used for status messages.
typedef struct {
  float angle;
  bool switch_triggered;
  bool thermal_overload_active;
  bool overcurrent_active;
} motor_status_t;

typedef struct {
  sm_state_t state;
  motor_status_t motor[MOTOR_ID_COUNT];
} status_t;

void gather_status(status_t *pstatus)
{
  assert(pstatus);

  pstatus->state = sm_state_current;
  for (int i = 0; i < MOTOR_ID_COUNT; i++) {
    if (config.motor[i].configured) {
      pstatus->motor[i].angle = mm6_get_angle(i);  // TODO: Or active?
      pstatus->motor[i].switch_triggered = mm6_get_switch_triggered(i);
      pstatus->motor[i].thermal_overload_active = mm6_get_thermal_overload_active(i);
      pstatus->motor[i].overcurrent_active = mm6_get_overcurrent_active(i);
    } else {
      memset(&pstatus->motor[i], 0, sizeof(motor_status_t));
    }
  }
}

void process_serial_input()
{
  const char ASCII_CTRL_C = 3;
  const char ASCII_BACKSPACE = 8;
  const char ASCII_BELL = 9;
  const char ASCII_RETURN = 13;
  const char ASCII_DELETE = 127;
  const int command_args_max_nbytes = 64;
  static char command_args[command_args_max_nbytes] = {};
  static int command_args_nbytes = 0;
  static menu_item_t *pprev_menu_item = NULL;
  static bool have_command = false;
  static bool reset_prompt = true;

  bool status_updated = false;
  static status_t previous_status = {};
  status_t status;
  gather_status(&status);

  if (memcmp(&previous_status, &status, sizeof(status_t)) != 0) {
    status_updated = true;
  }

  static unsigned long previous_status_time_millis = 0;
  unsigned long current_time_millis = millis();

  if (reset_prompt || 
      (!pprev_menu_item &&
       status_updated && 
       (current_time_millis - previous_status_time_millis > 250)))
  {
    if (status_updated && !reset_prompt)
      log_writeln(F(""));

    // mm6_dump_motor(MOTOR_ID_E);
    // mm6_dump_motor(MOTOR_ID_F);

    pprev_menu_item = NULL;
    command_args_nbytes = 0;
    have_command = false;
    reset_prompt = false;

    for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
      if (!config.motor[i].configured) {
        log_write(F("%c:_.__ "), 'A' + i);        
      } else {
        char angle_str[15];
        dtostrf(status.motor[i].angle, 3, 2, angle_str);
        char motor_name = (status.motor[i].switch_triggered ? 'A' : 'a') + i;

        log_write(F("%c:%s,%d,%d "), motor_name, angle_str, mm6_get_current_draw(i), motor_state[i].pid_perror);
      }
    }

    if (strlen(config.robot_name) != 0)
      log_write(F("%s "), config.robot_name);

    log_write(F("%s> "), sm_get_state_name(sm_state_current));

    memcpy(&previous_status, &status, sizeof(status_t));
    previous_status_time_millis = current_time_millis;
  }

  if (Serial.available()) {
    char input_char = Serial.read();           

    if (input_char == ASCII_CTRL_C) {
      log_writeln(F("<CTRL+C>"));
      reset_prompt = true;
    } else if (!have_command) {
      if (input_char == ASCII_RETURN) {
        log_writeln(F(""));
        reset_prompt = true;
      } else if ((input_char == ASCII_BACKSPACE) || (input_char == ASCII_DELETE)) {
        log_write(F("\9"));  // Emit ASCII bell (flashes screen on some terminals)
      } else {
        menu_item_t *pmenu_item = menu_item_by_command_char(input_char);
        pprev_menu_item = pmenu_item;

        if (!pmenu_item) {
          log_writeln(F("ERROR: Invalid command '%c'. Type '?' for help."), input_char);  // TODO: NAK?
          reset_prompt = true;
        } else {
          log_write(F("%s"), pmenu_item->pname);
          if (pmenu_item->print_sub_menu_fn)
            pmenu_item->print_sub_menu_fn();
          if (pmenu_item->has_args)
            log_write(F(" "));  // Additional space for args.

          have_command = true;
        }
      }
    } else {
      assert(pprev_menu_item);
      if ((input_char == '\n') || (input_char == '\r')) {
        log_writeln(F(""));
        command_args[command_args_nbytes] = '\0';
        
        // Eat whitespace at start of arguments.
        size_t nbytes = parse_whitespace(command_args, command_args_nbytes);
        char *p = command_args + nbytes;
        command_args_nbytes -= nbytes;

        if (pprev_menu_item->pfunction(p, command_args_nbytes) != command_args_nbytes) {
          log_writeln(F("ERROR: Invalid command arguments. Type '?' for help."));  // TODO: NAK?
        }
        reset_prompt = true;
      } else if ((input_char == ASCII_BACKSPACE) || (input_char == ASCII_DELETE)) {
        // TODO: only 1 BS if we're entering args.
        int nbackspaces_count = strlen(pprev_menu_item->pname) + 2;  // +2 for " >"
        if (pprev_menu_item->has_args) 
          nbackspaces_count++;  // Account for additional ' '.
        for (int i = 0; i < nbackspaces_count; i++) {
          Serial.write(ASCII_BACKSPACE);
          Serial.write(" ");
          Serial.write(ASCII_BACKSPACE);
        }
        reset_prompt = true;
      } else if (command_args_nbytes < command_args_max_nbytes - 1) {  // -1 to leave space for '\0' at end.
        log_write(F("%c"), input_char);
        command_args[command_args_nbytes++] = input_char;
      } else {
        log_writeln(F(""));
        LOG_ERROR(F("Too many characters in input buffer."));
        reset_prompt = true;
      }
    }
  }
}

bool thermal_overload_detected()
{
  bool detected = false;

  static bool thermal_overload_state[MOTOR_ID_COUNT] = {};

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    if (config.motor[i].configured) {
      bool thermal_overload = mm6_get_thermal_overload_active(i);
      if (thermal_overload)
        detected = true;

      if (thermal_overload != thermal_overload_state[i]) {
        if (thermal_overload)
          log_writeln(F("ERROR: Thermal overload error detected on motor %c."), 'A' + i);
        else
          log_writeln(F("Thermal overload error cleared on motor %c."), 'A' + i);
        thermal_overload_state[i] = thermal_overload;
      }
    }
  }

  return detected;
}

bool overcurrent_detected()
{
  const int current_limit = 1023;
  bool detected = false;

  return false;  // TODO  

  static int max_current = 0;

  static bool overcurrent_state[MOTOR_ID_COUNT] = {};

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    if (config.motor[i].configured) {
      int current_draw = mm6_get_current_draw(i);
      if (current_draw > current_limit)
        detected = true;

      if (current_draw > max_current) {
        LOG_DEBUG(F("max_current: %d"), current_draw);  // TODO: Why does this never print?
        max_current = current_draw;
      }

      if ((current_draw > current_limit) != overcurrent_state[i]) {
        if (current_draw > current_limit)
          log_writeln(F("ERROR: Overcurrent error detected on motor %c."), 'A' + i);
        else 
          log_writeln(F("Overcurrent error cleared on motor %c."), 'A' + i);
        overcurrent_state[i] = current_draw > current_limit;
      }
    }
  }

  return detected;
}

void setup() 
{
  /* 
   * The setup() function will only run once, after each powerup or reset of the Arduino board.
   * https://www.arduino.cc/reference/en/language/structure/sketch/setup/
   */

  sm_state_current = sm_execute(SM_STATE_INIT);
}

bool check_system_integrity()
{
  static bool previous_ok = true;
  bool ok = previous_ok;

  if (previous_ok)
    ok = config_check();

  if (config.robot_id == CONFIG_ROBOT_ID_NOT_CONFIGURED) {
    if (previous_ok)
      log_writeln(F("ERROR: robot_id == CONFIG_ROBOT_ID_NOT_CONFIGURED. Configure robot and restart."));
    ok = false;
  }

  ok = (thermal_overload_detected() || overcurrent_detected()) ? false : ok;

  // TODO: Detect error if any encoders don't transition from 0 -> 1 -> 3 -> 4 (or reverse).

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    if (motor_state[i].error_flags != 0) {
      if (previous_ok) {
        log_writeln(F(""));
        log_writeln(F("ERROR: motor %c error %d:"), 'A' + i, motor_state[i].error_flags);
        if (motor_state[i].error_flags & MOTOR_ERROR_FLAG_INVALID_ENCODER_TRANSITION)
          log_writeln(F("  Invalid quadrature encoder transition"));
        if (motor_state[i].error_flags & MOTOR_ERROR_FLAG_OPPOSITE_DIRECTION)
          log_writeln(F("  Motor direction opposite of expectation"));
      }
      ok = false;
    }
  }

  previous_ok = ok;  
  return ok;
}

void loop()
{
  /*
   * After creating a setup() function, which initializes and sets the initial values, the loop() 
   * function does precisely what its name suggests, and loops consecutively, allowing your 
   * program to change and respond. Use it to actively control the Arduino board.
   * https://www.arduino.cc/reference/en/language/structure/sketch/loop/
  */

  static bool previous_check_system_integrity_ok = true;
  if (previous_check_system_integrity_ok && !check_system_integrity()) {
    previous_check_system_integrity_ok = false;
    sm_state_current = SM_STATE_ERROR;
    log_writeln(F("ERROR: System integrity check failed."));
  }

  sm_state_current = sm_execute(sm_state_current);

  return;
}
