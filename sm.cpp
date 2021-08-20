/*
 * Implementation for state machine functions.
 */

#include <assert.h>

#include "command.h"
#include "crc32c.h"
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "menu.h"
#include "motor.h"
#include "parse.h"
#include "sm.h"

sm_state_t sm_state_current = SM_STATE_INIT;

static char* sm_state_name_by_state[] = { 
  "init", 
  "motors_off", 
  "motors_on", 
  "error" 
};

const char* sm_get_state_name(sm_state_t state)
{
  assert((state >= SM_STATE_FIRST) && (state <= SM_STATE_LAST));
  return sm_state_name_by_state[state];
}

/*
 * Self-test functions.
 */

typedef struct {
  bool (*test_function)();
  char *pname;
} test_case_t;

static test_case_t test_case[] = {
  { config_test, "config" },
  { crc32c_test, "crc32c" },
  { log_test, "log" },
  { menu_test, "menu" },
  { parse_test, "parse" },
  { sm_test, "sm (state smachine)" },
};
#define TEST_CASE_COUNT sizeof(test_case) / sizeof(test_case[0])

static bool run_self_test() {
  int failure_count = 0;
  log_writeln(F("Running self test:"));
  for (int i = 0; i < TEST_CASE_COUNT; i ++) {
    if (test_case[i].test_function()) {
      log_writeln(F("  %d. %s ... pass."), i, test_case[i].pname);
    } else {
      log_writeln(F("  %d. %s ... fail."), i, test_case[i].pname);
      failure_count++;
    }
  }

  log_writeln(F("%d test cases run, %d passed, %d failed."), TEST_CASE_COUNT, TEST_CASE_COUNT - failure_count, failure_count);

  return failure_count == 0;
}

// Used for status messages.
typedef struct {
  float angle;
  bool switch_triggered;
  bool thermal_overload_detected;
  bool overcurrent_detected;
} motor_status_t;

typedef struct {
  sm_state_t state;
  motor_status_t motor[MOTOR_ID_COUNT];
} status_t;

static void gather_status(status_t *pstatus)
{
  assert(pstatus);

  pstatus->state = sm_state_current;
  for (int i = 0; i < MOTOR_ID_COUNT; i++) {
    if (config.motor[i].configured) {
      pstatus->motor[i].angle = motor_get_angle(i);
      pstatus->motor[i].switch_triggered = motor_get_switch_triggered(i);
      pstatus->motor[i].thermal_overload_detected = motor_get_thermal_overload_detected(i);
      pstatus->motor[i].overcurrent_detected = motor_get_overcurrent_detected(i);
      motor_clear_thermal_overload(i);
      motor_clear_overcurrent(i);
    } else {
      memset(&pstatus->motor[i], 0, sizeof(motor_status_t));
    }
  }
}

static void process_serial_input()
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

    // motor_dump_motor(MOTOR_ID_E);
    // motor_dump_motor(MOTOR_ID_F);

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

        log_write(F("%c:%s,%d,%d "), motor_name, angle_str, motor_get_current_draw(i), motor_state[i].pid_perror);
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

static sm_state_t init_execute()
{
  Serial.begin(38400);

  log_writeln(F("\n\rBooting Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories."));
  command_print_software_version("", 0);  // Pass empty args to mimic user typing version command.
  log_writeln();

  bool config_read_success = config_read();
  if (!config_read_success) {
    log_writeln(F("ERROR: Invalid configuration. Re-initializing configuration data."));
    config_clear();
  } else {
    log_writeln(F("Read %d bytes of configuration data from EEPROM. Configuration is valid."), sizeof(config_t));
    log_writeln(F("Configured for '%s'."), config_robot_name_by_id[config.robot_id]);
  }

  config_print();
  hardware_init();
  motor_init_all();

  bool self_test_success = run_self_test();
  menu_help();
  log_writeln(F("Ready."));
  
  return (config_read_success && self_test_success) ? SM_STATE_MOTORS_OFF : SM_STATE_ERROR;
}

static bool motors_off_enter()
{
  motor_set_pid_enable(false);
  return true;
}

static sm_state_t motors_off_execute()
{
  process_serial_input();
  return sm_state_current;
}

static bool motors_on_enter()
{
  motor_set_pid_enable(true);
  return true;
}

sm_state_t motors_on_execute()
{
  process_serial_input();
  return sm_state_current;  
}

bool motors_on_exit()
{
  motor_set_pid_enable(false);
  return true;
}

bool error_enter()
{
  motor_set_pid_enable(false);
  return true;
}

sm_state_t error_execute()
{
  process_serial_input();
  return SM_STATE_ERROR;
}

typedef struct {
  bool (*enter)();
  sm_state_t (*execute)();
  bool (*exit)();
} sm_entry;

sm_entry sm_entry_by_state[] = {
  { NULL, init_execute, NULL },
  { motors_off_enter, motors_off_execute, NULL },
  { motors_on_enter, motors_on_execute, motors_on_exit },
  { error_enter, error_execute, NULL }  
};

static void enter_state(sm_state_t start_state)
{
  if (!sm_entry_by_state[start_state].enter()) {
    sm_state_current = SM_STATE_ERROR;
  } else {
    sm_state_current = start_state;
  }
}

sm_state_t sm_execute(sm_state_t current_state)
{
  assert(current_state >= SM_STATE_FIRST);
  assert(current_state <= SM_STATE_LAST);

  static sm_state_t previous_state = SM_STATE_FIRST - 1;

  if (previous_state != current_state) {
    if (previous_state != SM_STATE_FIRST - 1) {
      if (sm_entry_by_state[previous_state].exit &&
          !sm_entry_by_state[previous_state].exit())
        current_state = SM_STATE_ERROR;

      if (sm_entry_by_state[current_state].enter &&
          !sm_entry_by_state[current_state].enter())
        current_state = SM_STATE_ERROR;
    }
    previous_state = current_state;
  }

  assert(sm_entry_by_state[current_state].execute);
  return sm_entry_by_state[current_state].execute();
}

bool sm_test()
{ 
  // TODO: Implement.

  // Confirm can't escape error state.
  return true;
}
