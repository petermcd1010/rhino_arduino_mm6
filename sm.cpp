/*
 * Implementation for state machine functions.
 */

#include <assert.h>

#include "config.h"
#include "hardware.h"
#include "log.h"
#include "menu.h"
#include "sm.h"

extern void hardware_emergency_stop();  // TODO: remove.
extern void print_software_version();  // TODO: remove.
extern void process_serial_input();  // TODO: remove.
extern void check_noinit_data();  // TODO: remove.
extern bool run_self_test();  // TODO: remove.

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

static sm_state_t init_execute()
{
  Serial.begin(38400);

  log_writeln(F("\n\rBooting Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories."));
  print_software_version();
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
  hardware_emergency_stop();
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
