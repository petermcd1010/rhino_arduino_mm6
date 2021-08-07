/*
 * Implementation for state machine functions.
 */

#include <assert.h>

#include "command.h"
#include "crc32c.h"
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "mm6.h"
#include "menu.h"
#include "parse.h"
#include "sm.h"

extern void process_serial_input();  // TODO: remove.
extern void check_noinit_data();  // TODO: remove.

static char* sm_state_name_by_state[] = { 
  "init", 
  "motors_off", 
  "motors_on", 
  "error" 
};

sm_state_t sm_state_current = SM_STATE_INIT;

const char* sm_get_state_name(sm_state_t state)
{
  assert(state >= SM_STATE_FIRST);
  assert(state <= SM_STATE_LAST);
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
  { sm_test, "sm (state smachine" },
};
#define TEST_CASE_COUNT sizeof(test_case) / sizeof(test_case[0])

static bool run_self_test() {
  bool ret = true;
  int failure_count = 0;
  log_writeln(F("Running self test:"));
  for (int i = 0; i < TEST_CASE_COUNT; i ++) {
    if (test_case[i].test_function()) {
      log_writeln(F("  %d. %s ... pass."), i, test_case[i].pname);
    } else {
      log_writeln(F("  %d. %s ... fail."), i, test_case[i].pname);
      ret = false;
      failure_count++;
    }
  }

  log_writeln(F("%d test cases run, %d passed, %d failed."), TEST_CASE_COUNT, TEST_CASE_COUNT - failure_count, failure_count);

  return ret;
}

static sm_state_t init_execute()
{
  Serial.begin(38400);

  log_writeln(F("\n\rBooting Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories."));
  command_print_software_version("", 0);
  log_writeln();

  bool config_read_success = config_read();
  if (!config_read_success) {
    log_writeln(F("ERROR: Invalid configuration. Re-initializing configuration data."));
    config_clear();
  } else {
    log_writeln(F("Read %d bytes of configuration data from EEPROM. Configuration is valid."), sizeof(config_t));
    log_writeln(F("Configured for '%s'."), config_robot_name_by_id[config.robot_id]);
    mm6_print_encoders();
  }

  config_print();
  mm6_init();
  check_noinit_data();
  hardware_init();

  bool self_test_success = run_self_test();
  menu_help();
  log_writeln(F("Ready."));
  
  // return (config_read_success && self_test_success) ? SM_STATE_MOTORS_OFF : SM_STATE_ERROR; TODO2
  return SM_STATE_MOTORS_ON;
}

static bool motors_off_enter()
{
  // mm6_enable_motors(false);  !! TODO
  return true;
}

static sm_state_t motors_off_execute()
{
  process_serial_input();
  sm_state_current = SM_STATE_MOTORS_OFF;
  return sm_state_current;
}

static bool motors_on_enter()
{
  mm6_enable_all(true);
  mm6_pid_enable(true);
  return true;
}

sm_state_t motors_on_execute()
{
  process_serial_input();
  return SM_STATE_MOTORS_ON;
}

bool motors_on_exit()
{
  mm6_pid_enable(false);
  mm6_enable_all(false);
  return true;
}

bool error_enter()
{
  mm6_enable_all(false);
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
  { motors_on_enter, motors_on_execute, motors_on_enter },
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
      LOG_D(F("%s_exit()"), sm_entry_by_state[previous_state]);
      if (sm_entry_by_state[previous_state].exit &&
          !sm_entry_by_state[previous_state].exit())
        current_state = SM_STATE_ERROR;

      LOG_D(F("%s_enter()"), sm_entry_by_state[current_state]);
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
  return true;
}
