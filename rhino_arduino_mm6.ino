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
#include "mm6.h"
#include "parse.h"

static const float rhino_arduino_mm6_version = 2.00;  

static bool motor_sync_move_enabled = false;
int End[] = {0,0,0,0,0,0};
int LeadMotor = 0;
int Start[] = {0,0,0,0,0,0};
float Ratio[] = {0,0,0,0,0,0};

typedef struct {
  char command_char;
  char *pname;
  void(*print_sub_menu_fn)();
  bool has_args;
  int (*pfunction)(char *payload, size_t nbytes);
  char *phelp;
} menu_item_t;

// Used in the state machine.
typedef enum {
  STATE_FIRST = 0,  // TODO: remove when config is crc'd?
  STATE_INIT = STATE_FIRST,
  STATE_MOTORS_OFF,
  STATE_MOTORS_ON,
  STATE_ERROR,
  STATE_COUNT,
  STATE_LAST = STATE_COUNT - 1,
} state_t;

static char* state_name_by_state[] = { 
  "init", 
  "motors_off", 
  "motors_on", 
  "error" 
};
static state_t state = STATE_INIT;

// Used for status messages.
typedef struct {
  float angle;
  bool switch_triggered;
  bool thermal_overload_active;
  bool overcurrent_active;
} motor_status_t;

typedef struct {
  state_t state;
  motor_status_t motor[MOTOR_ID_COUNT];
} status_t;

void hardware_emergency_stop()  // TODO: Move to mm6_stop_all()?
{
  // TODO: call hardware_emergency_stop when entering ERROR state.

  mm6_enable_all(false);

  // Avoid extra log messages if emergency stop has already been executed.
  static bool stopped = false;
  if (stopped) {
    log_writeln(F("Emergency stop completed."));
    stopped = true;
  }

  state = STATE_ERROR;
}

/*
 * Menu functions.
*/

static const int menu_item_max_name_nbytes = 25;

void extended_menu_robot_id()
{
  log_writeln(F(""));  
  log_writeln(F("Current robot ID: %s."), config_robot_name_by_id[config.robot_id]);
  log_writeln(F("Available robot IDs:"));
  for (int i = CONFIG_ROBOT_ID_FIRST; i <= CONFIG_ROBOT_ID_LAST; i++) {
    log_writeln(F("  %d. %s."), i, config_robot_name_by_id[i]);
  }
  log_writeln(F("Select new robot ID, or press [RETURN] to keep current robot ID."));
  log_write(F(">"));
}

void extended_menu_robot_serial()
{
  log_writeln(F(""));  
  log_writeln(F("Current robot serial is '%s'."), config.robot_serial);
  log_writeln(F("Enter new robot serial, or press [RETURN] to keep current serial."));
  log_write(F(">"));
}

void extended_menu_robot_name()
{
  log_writeln(F(""));  
  log_writeln(F("Current robot name is '%s'."), config.robot_name);
  log_writeln(F("Enter new robot name shorter than %d characters, or press [RETURN] to keep current name."), CONFIG_ROBOT_NAME_NBYTES);
  log_write(F(">"));
}

void extended_menu_factory_reset()
{
  log_writeln(F(""));    
  log_writeln(F("Type 'RESET' in all capital letters to clear the EEPROM and reboot the system or <CTRL+C> to exit."));
  log_write(F(">"));
}

void extended_menu_reboot()
{
  log_writeln(F(""));  
  log_writeln(F("Type 'REBOOT' in all capital letters to reboot the system or <CTRL+C> to exit.")) ;
  log_write(F(">"));
}

const menu_item_t menu_item_by_index[] = {  // TODO: F()
  { '1', "print configuration", NULL, true, command_print_config, "-- print configuration." },
  { '2', "configure robot ID", extended_menu_robot_id, true, command_config_robot_id, "[id] print or set configured robot ID." },
  { '3', "configure robot serial", extended_menu_robot_serial, true, command_config_robot_serial, "[string] -- print or set configured robot serial." },  
  { '4', "configure robot name", extended_menu_robot_name, true, command_config_robot_name, "[name] -- print or set configured robot name." },
  { '0', "write configuration", NULL, true, command_config_write, "Write configuration data to EEPROM." },
  { 'B', "heartbeat", NULL, true, command_heartbeat, "off / on -- enable heartbeat (only emitted when input mode is packet)." },
  { 'C', "run calibration", NULL, true, command_run_calibration, "-- calibrate motor and switch limits." },
  { 'D', "PID mode", NULL, false, command_pid_mode, "-- Enable/disable motors" },
  { 'E', "emergency stop", NULL, false, command_emergency_stop, "-- execute hardware emergency stop (E-Stop). Enters 'error' state. Requires reboot to reset."},
  { 'G', "set gripper position", NULL, true, command_set_gripper_position, "-- set current encoders as gripper?" },
  { 'H', "set home position", NULL, false, command_set_home_position, "-- set current encoders as home position." },
  { 'I', "input mode", NULL, true, command_input_mode, "packet / keyboard."},
  { 'K', "clock", NULL, true, command_clock, "get / set [YYYY-MM-DD] HH:MM[:SS].fraction]]." },
  { 'M', "print motor status", NULL, false, command_print_motor_status, "-- print motor status." },
  { 'N', "set motor angle", NULL, true, command_set_motor_angle, "motorid degrees -- degrees is 0.0 to 360.0, +15, -20, +, -, ++, --." },
  { 'P', "set motor encoder", NULL, true, command_set_motor_encoder, "motorid encoder -- encoder is in the range X - Y." },  // TODO
  { 'Q', "run test sequence", NULL, false, command_run_test_sequence, "" },
  { 'T', "test motors", NULL, false, command_test_motors, "-- test motors." },
  { 'V', "print software version", NULL, false, command_print_software_version, "-- print software version." },
  { 'W', "waypoint", NULL, true, command_waypoint, "" },
  { 'X', "expansion I/O", NULL, true, command_expansion_io, ""},
  // { 'M', "motor config", true, command_motor_config, ""},
  { 'Z', "set zero position", NULL, false, command_set_zero_position, "-- set current encoders as zero position." },
  { '*', "factory reset", extended_menu_factory_reset, false, command_factory_reset, "RESET -- factory reset system, clearing EEPROM and rebooting."},
  { '!', "reboot", extended_menu_reboot, true, command_reboot, "REBOOT -- reboot system. Requires typing the 'REBOOT' keyword." }, 
  { '?', "print help", NULL, false, command_print_help, "-- print this help message." },
};
#define MENU_ITEM_COUNT sizeof(menu_item_by_index) / sizeof(menu_item_by_index[0])

menu_item_t* menu_item_by_command_char(char ch)
{
  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    if (toupper(ch) == toupper(menu_item_by_index[i].command_char)) {
      return &menu_item_by_index[i];
    }
  }
  return NULL;
}

void menu_help()
{
  log_writeln(F("Command list:"));

  int longest_name_nbytes = 0;
  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    menu_item_t *pitem = &menu_item_by_index[i];
    int n = strlen(pitem->pname);
    longest_name_nbytes = n > longest_name_nbytes ? n : longest_name_nbytes;
  }

  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    menu_item_t *pitem = &menu_item_by_index[i];
    // Pad with spaces afer the name to align the help message.
    char spaces[menu_item_max_name_nbytes];  // Just a buffer of whitespace.
    int nspaces = longest_name_nbytes - strlen(pitem->pname);
    nspaces = (nspaces >= menu_item_max_name_nbytes) ? menu_item_max_name_nbytes - 1 : nspaces;
    memset(spaces, ' ', nspaces);
    spaces[nspaces] = '\0';
    log_writeln(F("%c: %s %s %s"), pitem->command_char, pitem->pname, spaces, pitem->phelp);
  }
}

bool test_menu()
{
  bool ret = true;

  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    menu_item_t *pitem = &menu_item_by_index[i];
    if (!pitem->pname) {
      LOG_E(F("%d NULL pname"), i);
      ret = false;
    }
    if (strlen(pitem->pname) > menu_item_max_name_nbytes) {
      LOG_E(F("%d pname too long"), i);
      ret = false;
    }
    if (!pitem->pfunction) {
      LOG_E(F("%d NULL pfunction"), i);
      ret = false;
    }
    if (!pitem->phelp) {
      LOG_E(F("%d NULL phelp"), i);
      ret = false;
    }
  }

  return ret;
}

/*
 * Self-test functions.
 */

typedef struct {
  bool (*test_function)();
  char *pname;
} test_case_t;

test_case_t test_case[] = {
  { test_config, "config" },
  { test_crc32c, "crc32c" },
  { test_log, "log" },
  { test_menu, "menu" },
  { test_parse, "parse" },
  { test_time, "time" },
};
#define TEST_CASE_COUNT sizeof(test_case) / sizeof(test_case[0])

bool run_self_test() {
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

/*
 * Command processing functions.
 */

int command_clock(char *pargs, unsigned nybtes_args)
{
  // TODO.
  assert(false);
}

int command_emergency_stop(char *pargs, size_t args_nbytes)
{
  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (args_nbytes != nbytes) {
    return -1;
  } 

  hardware_emergency_stop();

  return 0;
}

int command_set_gripper_position(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_print_config(char *pargs, unsigned args_nbytes)
{
  assert(pargs);  

  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (args_nbytes != nbytes) {
    return -1;
  } 

  config_print();
  
  return 0;
}

int command_config_robot_id(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  char *p = pargs;

  size_t nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes == 0) {
    log_writeln(F("Maintaining robot ID as '%s'."), config_robot_name_by_id[config.robot_id]);    
    return p - pargs;
  }  

  config_robot_id_t robot_id = CONFIG_ROBOT_ID_FIRST - 1;
  nbytes = parse_int(p, args_nbytes, (int*)(&robot_id));
  if (nbytes < 0)  // TODO: is this ever true?
    return -1;
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if ((robot_id < CONFIG_ROBOT_ID_FIRST) || (robot_id > CONFIG_ROBOT_ID_LAST)) {
    log_writeln(F("ERROR: Invalid robot ID."));
    goto error;
  }
  
  if (args_nbytes > 0)
    goto error;

  log_writeln(F("Setting robot ID to '%s'."), config_robot_name_by_id[robot_id]);

  config_set_robot_id(robot_id);  // TODO: return value?

  return p - pargs;

error:
  return -1;
}

int command_config_robot_serial(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  char *p = pargs;

  size_t nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes == 0) {
    log_writeln(F("Maintaining robot serial '%s'."), config.robot_serial);
    return p - pargs;
  }  

  char robot_serial[CONFIG_ROBOT_SERIAL_NBYTES];
  nbytes = parse_string(p, args_nbytes, robot_serial, CONFIG_ROBOT_SERIAL_NBYTES);
  if (nbytes < 0)  // TODO: is this ever true?
    return -1;
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes > 0)
    goto error;

  log_writeln(F("Setting robot serial to '%s'."), robot_serial);

  config_set_robot_serial(robot_serial);  // TODO: return value?

  return p - pargs;

error:
  return -1;
}

int command_config_robot_name(char *pargs, size_t args_nbytes)  // TODO: should these return a size_t?
{
  assert(pargs);

  char *p = pargs;

  size_t nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes == 0) {
    log_writeln(F("Maintaining robot name '%s'."), config.robot_name);
    return p - pargs;
  }  

  char robot_name[CONFIG_ROBOT_NAME_NBYTES];
  nbytes = parse_string(p, args_nbytes, robot_name, CONFIG_ROBOT_NAME_NBYTES);
  if (nbytes < 0)  // TODO: is this ever true?
    return -1;
  args_nbytes -= nbytes;
  p += nbytes;
  
  if (args_nbytes > 0)
    goto error;

  log_writeln(F("Setting robot name to '%s'."), robot_name);

  config_set_robot_name(robot_name); // TODO: return value?

  return p - pargs;

error:
  return -1;
}

int command_config_write(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  } 

  assert(config_write());
  log_writeln(F("%d bytes of configuration data written to EEPROM."), sizeof(config_t));
  return 0;
}

int command_heartbeat(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_print_help(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  // TODO: confirm pargs is only whitespace.
  menu_help();
  
  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  } 

  return 0;
}

int command_set_home_position(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  } 

  SetPositionToHome();

  return nbytes;
}

int command_run_calibration(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  }
  
  log_writeln(F("Calibration ... %s"), mm6_calibrate_all() ? "passed" : "FAILED");

  return nbytes;
}

int command_input_mode(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);
}

int command_print_motor_status()
{  
  log_writeln(F("PID:%s"), mm6_pid_enabled ? "enabled" : "disabled");

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {    
    char angle_str[15] = {};
    dtostrf(mm6_get_angle(i), 3, 2, angle_str);
    log_writeln(F("%c: home:%d sta:%d enc:%d tar:%d err:%d spd:%d PWM:%d cur:%d hs:%d,%d,%d,%d->%d angle:%s"), 
        'A' + i, 
        motor_state[i].switch_previously_triggered,  // home.
        motor_state[i].progress,  // sta. Report whether or not the Motor has reached the target location.
        /* mm6_get_encoder(iMotor), */  // pos.
        noinit_data.encoder[i] * motor_state[i].logic,  // enc.
        motor_state[i].target_encoder * motor_state[i].logic,  // tar.
        motor_state[i].pid_perror * motor_state[i].logic,  // err.
        motor_state[i].speed * motor_state[i].logic,  // spd.
        /* motor_state[i].target_speed, */  // tspd.
        motor_state[i].pwm,
        motor_state[i].current_draw,  // cur.
        motor_state[i].switch_reverse_off,  // hs.
        motor_state[i].switch_forward_on,  // hs.
        motor_state[i].switch_reverse_on,  // hs.
        motor_state[i].switch_forward_off,  // hs.
        (motor_state[i].switch_forward_off + motor_state[i].switch_reverse_on + motor_state[i].switch_forward_on + motor_state[i].switch_reverse_off) / 4, // hs.
        angle_str);
  } 

  return 0;
}

int command_set_motor_angle(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  motor_id_t motor_id = MOTOR_ID_A;
  char *p = pargs;
  size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);  // TODO: all nbytes to size_t.
  if (nbytes <- 0)
    goto error;  // parse_motor_id will emit message if error.   
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  float angle = mm6_get_angle(motor_id);
  nbytes = parse_motor_angle_or_encoder(p, args_nbytes, &angle);
  if (nbytes <= 0)
    goto error;  // parse_motor_angle_or_encoder will emit message if error.   
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes > 0)
    goto error;

  char angle_str[15] = {};
  dtostrf(angle, 3, 2, angle_str);

  if (mm6_enabled(motor_id)) {
    log_writeln(F("Move Motor %c to an angle of %s degrees."), 'A' + motor_id, angle_str);
    mm6_set_target_angle(motor_id, angle);
  } else {
    log_writeln(F("ERROR: Motor %c not enabled."), 'A' + motor_id);
    // TODO: error state?
  }
  return p - pargs;

error:
  LOG_E(F(""));
  return -1;
}

int command_set_motor_encoder(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  motor_id_t motor_id = MOTOR_ID_A;
  char *p = pargs;
  size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);  // TODO: all nbytes to size_t.
  if (nbytes <= 0)
    goto error;  // parse_motor_id will emit message if error.   
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  float encoder = mm6_get_encoder(motor_id);
  nbytes = parse_motor_angle_or_encoder(p, args_nbytes, &encoder);
  if (nbytes <= 0)
    return -1;
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes > 0)
    goto error;

  char encoder_str[15] = {};
  dtostrf(encoder, 3, 2, encoder_str);

  if (mm6_enabled(motor_id)) {
    log_writeln(F("Move Motor %c to encoder %s."), 'A' + motor_id, encoder_str);
    mm6_set_target_encoder(motor_id, encoder);
  } else {
    log_writeln(F("ERROR: Motor %c not enabled."), 'A' + motor_id);
    // TODO: error state?
  }

  return p - pargs;

error:
  LOG_E(F(""));
  return -1;
}

int command_pid_mode(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_run_test_sequence(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_test_motors(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (args_nbytes != nbytes) {
    return -1;
  } 

  mm6_test_all();

  return nbytes;
}

int command_factory_reset(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  char *p = pargs;
  
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;
  if (args_nbytes == 0)
    return -1;

  int entry_num = -1;
  char *reboot_table[] = { "RESET" };
  nbytes = parse_string_in_table(pargs, args_nbytes, reboot_table, 1, &entry_num);
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes > 0)
    goto error;

  assert(entry_num == 0);
  hardware_reset();
  hardware_reboot();

  return 0;

error:

  return -1;
}

int command_reboot(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  pargs += nbytes;
  args_nbytes -= nbytes;
  if (args_nbytes == 0)
    return -1;

  int entry_num = -1;
  char *reboot_table[] = { "REBOOT" };
  nbytes = parse_string_in_table(pargs, args_nbytes, reboot_table, 1, &entry_num);
  // TODO: check entry_num == -1?
  // TODO: allow whitespace after REBOOT.
  if (nbytes != args_nbytes)
    return -1;

  assert(entry_num == 0);
  hardware_reboot();

  return 0;
}

const size_t command_args_max_nbytes = 64;

int command_print_software_version(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  if (args_nbytes >= command_args_max_nbytes)
    return -1;

  char *p = pargs;
  size_t nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes > 0)
    return -1;

  print_software_version();
  log_writeln();

  return 0;
}


int command_waypoint(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_expansion_io(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_set_zero_position(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  } 
  
  ZeroPositions();

  return nbytes;
}

//----------------------------------------------------------------------

// Define other I/O lines.
// #define ESERT 18 // Serial Transmit
// #define ESERR 19 // Serial Receive
// #define EI2CD 20 // I2C Data
// #define EI2CC 21 // I2C Clock

//*************************************
// Motor Status / Motion Control Vars.
//*************************************
int Distance[] = {0,0,0,0,0,0};
//*************************************
// Waypoint Structure and vars
//*************************************
struct sWayPoint {
  byte Number;
  char Command;
  float A;
  float B;
  float C;
  float D;
  float E;
  float F;  
};
  sWayPoint WayPoint = {
    0,0,
    0,0,0,0,0,0
  };
String StringSplits[8];  //Used to store waypoints
//-----------------------------------------

String InBuffer = "";
char padbuffer [50];
int Command_Motor = MOTOR_ID_E;

void check_noinit_data()
{
  // Zero out saved variables on power cycle. On reset, these values are NOT erased.

  if ((noinit_data.nbytes != sizeof(noinit_data_t) ||
      (noinit_data.version != noinit_data_version) ||
      (noinit_data.magic != noinit_data_magic))) {
    log_writeln(F("Initializing noinit data."));
    noinit_data.nbytes = sizeof(noinit_data_t);
    noinit_data.version = noinit_data_version;
    noinit_data.magic = noinit_data_magic;
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
      noinit_data.encoder[i] = 0;
      noinit_data.previous_quadrature_encoder[i] = 0;
    }
  } else {
    log_writeln(F("Reset without power cycle detected. Reusing stored motor encoder values."));
  }
}

void gather_status(status_t *pstatus)
{
  assert(pstatus);

  pstatus->state = state;
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

    log_write(F("%s> "), state_name_by_state[state]);  // TODO: F()

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
        LOG_E(F("Too many characters in input buffer."));
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
        LOG_D(F("max_current: %d"), current_draw);  // TODO: Why does this never print?
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

void print_software_version()
{
  char version_number_string[10] = {};
  dtostrf(rhino_arduino_mm6_version, 3, 2, version_number_string);
  log_writeln(F("Version: %s (%s %s)"), version_number_string, __DATE__, __TIME__);
}

state_t state_init_execute()
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
  
  // return (config_read_success && self_test_success) ? STATE_MOTORS_OFF : STATE_ERROR; TODO2
  return STATE_MOTORS_ON;
}

bool state_motors_off_enter()
{
  // mm6_enable_motors(false);  !! TODO
  return true;
}

state_t state_motors_off_execute()
{
  process_serial_input();
  state = STATE_MOTORS_OFF;
  return state;
}

bool state_motors_on_enter()
{
  mm6_enable_all(true);
  mm6_pid_enable(true);
  return true;
}

state_t state_motors_on_execute()
{
  process_serial_input();
  return STATE_MOTORS_ON;
}

bool state_motors_on_exit()
{
  mm6_pid_enable(false);
  mm6_enable_all(false);
  return true;
}

bool state_error_enter()
{
  hardware_emergency_stop();
  return true;
}

state_t state_error_execute()
{
  process_serial_input();
  return STATE_ERROR;
}

typedef struct {
  bool (*enter)();
  state_t (*execute)();
  bool (*exit)();
} state_machine_entry;

state_machine_entry state_machine_entry_by_state[] = {
  { NULL, state_init_execute, NULL },
  { state_motors_off_enter, state_motors_off_execute, NULL },
  { state_motors_on_enter, state_motors_on_execute, state_motors_on_enter },
  { state_error_enter, state_error_execute, NULL }  
};

void state_machine_enter_state(state_t start_state)
{
  if (!state_machine_entry_by_state[start_state].enter()) {
    state = STATE_ERROR;
  } else {
    state = start_state;
  }
}

state_t state_machine_execute(state_t current_state)
{
  assert(current_state >= STATE_FIRST);
  assert(current_state <= STATE_LAST);

  static state_t previous_state = STATE_FIRST - 1;

  if (previous_state != current_state) {
    if (previous_state != STATE_FIRST - 1) {
      LOG_D(F("%s_exit()"), state_name_by_state[previous_state]);
      if (state_machine_entry_by_state[previous_state].exit &&
          !state_machine_entry_by_state[previous_state].exit())
        current_state = STATE_ERROR;

      LOG_D(F("%s_enter()"), state_name_by_state[current_state]);
      if (state_machine_entry_by_state[current_state].enter &&
          !state_machine_entry_by_state[current_state].enter())
        current_state = STATE_ERROR;
    }
    previous_state = current_state;
  }

  assert(state_machine_entry_by_state[current_state].execute);
  return state_machine_entry_by_state[current_state].execute();
}

void setup() 
{
  /* 
   * The setup() function will only run once, after each powerup or reset of the Arduino board.
   * https://www.arduino.cc/reference/en/language/structure/sketch/setup/
   */

  state = state_machine_execute(STATE_INIT);
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
    state = STATE_ERROR;
    log_writeln(F("ERROR: System integrity check failed."));
  }

  state = state_machine_execute(state);

  return;

/*
  static int lMotor = 0;
  lMotor++; if (lMotor==6) lMotor=0;// Move to the next motor  
  motor_state[lMotor].current  = analogRead(motor_pinout[lMotor].in_current_draw);

  if (tracking>0) {
    TrackReport(lMotor);
  } 
*/
/*  int HomeButton = digitalRead(Home_IO);  // TODO: Pin 51 in original code. But this is motor F's direction pin, too.?
  if (HomeButton == 0) {
    for (int hMotor=MOTOR_IF_A; hMotor<=MOTOR_IF_F; hMotor++){
      //motor_state[hMotor].target_encoder = 0;
    }
  }*/

  if (Gripper_StallX>80) {
    // Motor A has been stalled for over 1/4 second.
    Serial.print("Gripper Stall @");
    Serial.print(Gripper_StallE);
    Serial.print("  Cur=");
    Serial.print(Gripper_StallC);
    Serial.print("  Open=");
    Serial.print(config.gripper_open_encoder);
    Serial.print("  Closed=");
    Serial.print(config.gripper_close_encoder);   
    Serial.println(".");
    Gripper_StallX=0;
    motor_state[MOTOR_ID_A].target_encoder = noinit_data.encoder[MOTOR_ID_A];
    motor_state[MOTOR_ID_A].current_draw = 0;
  }

#if 0
    if (input_char = '\n'){
      Serial.print(InBuffer + ":");
      } else if ((InBuffer=="G") || (InBuffer=="g")){
        mm6_pid_enable(true);        
      } else if ((InBuffer=="I") || (InBuffer=="i")){
        InterrogateLimitSwitches();
      } else if ((InBuffer=="K") || (InBuffer=="k")){
        RunWayPointSeq();
      } else if ((InBuffer=="O") || (InBuffer=="o")){
        OpenGripper();
      } else if ((InBuffer=="M") || (InBuffer=="m")){
        mm6_test_motors();
      } else if ((InBuffer=="P") || (InBuffer=="p")){
        mm6_print_encoders();
      } else if ((InBuffer=="Q") || (InBuffer=="q")){
        TestSeq1();
      } else if ((InBuffer=="R") || (InBuffer=="r")){
        if (InBuffer=="R"){
          config_set_robot_id(CONFIG_ROBOT_ID_RHINO_XR_4);  
        } else {
          config_set_robot_id(CONFIG_ROBOT_ID_RHINO_XR_3);  
        }        
      } else if ((InBuffer=="S") || (InBuffer=="s")){
        mm6_pid_enable(false);
      } else if ((InBuffer=="U") || (InBuffer=="u")){
        StopTracking();
      } else if ((InBuffer=="V") || (InBuffer=="v")){
        CloseGripper();
      } else if ((InBuffer=="W") || (InBuffer=="w")){
        StartTracking();
      } else if ((InBuffer=="X") || (InBuffer=="w")){
        SetHomeToCenterOfSwitches();
      } else if ((InBuffer=="Z") || (InBuffer=="z")){
        ZeroPositions();
      } else if ((InBuffer=="N") || (InBuffer=="n")){
        char version_str[15] = {};
        dtostrf(software_version, 3, 2, version_str);
        log_writeln(F("Version: %s (%s)."), version_str, psoftware_date);  
      } else if (InBuffer=="~"){
        SetZeroAngles();
      } else if (InBuffer=="?"){
        ShowHelp();
      } else if (InBuffer=="~!@#$+"){
        RunMotorsForAsmTest();
      } else {
        char m = InBuffer.charAt(0);
        if ((m>=65) && (m<=70)) {
          // A-F
          Command_Motor = m - 65;
          String WhatToDo = InBuffer.substring(1);          
          if (WhatToDo.length()>0) 
          {
            if (WhatToDo=="X") {
              Exercise(Command_Motor);
            } else if (WhatToDo=="R") {
              Reverse(Command_Motor);
            } else if (WhatToDo=="-") {
              int Position = mm6_get_encoder(Command_Motor)-10;
              mm6_set_position(Command_Motor, Position);              
            } else if (WhatToDo=="--") {
              int Position = mm6_get_encoder(Command_Motor)-100;
              mm6_set_position(Command_Motor, Position);              
            } else if (WhatToDo=="+") {
              int Position = mm6_get_encoder(Command_Motor)+10;
              mm6_set_position(Command_Motor, Position);              
            } else if (WhatToDo=="++") {
              int Position = mm6_get_encoder(Command_Motor)+100;
              mm6_set_target_encoder(Command_Motor, Position);              
            } else {              
              int Position = WhatToDo.toInt();
              mm6_set_target_encoder(Command_Motor, Position);
            }      
          } else {
            Serial.print("Command Motor Set to: ");
            Serial.print(char(Command_Motor+65));    
            Serial.println(".");
          }
        } else if ((m>=97) && (m<=102)) {
          // a-f
          Command_Motor = m - 97;
          if (InBuffer.length()>1) 
          {
            InBuffer.setCharAt(0,32);
            float Angle = InBuffer.toFloat();
            mm6_set_target_angle(Command_Motor, Angle);
            //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
          }
        } else if (m==120) {
          // x
          SetWayPointAngle();
        } else if (m==114) {
          // r
          GetWayPointAngle();
        } else if (m==33) {
          // !
          MoveToAWayPointAngle();
        }
      }
      InBuffer = "";
    } else {
      InBuffer = InBuffer + input_char;
    }
  }
#endif
}

//*****************************************
// Track the motors by sending the current 
//  position to a connected computer 
//*****************************************
void StartTracking() {
  if (InBuffer=="W"){
    tracking=1; // Track Position 
    Serial.println("Tracking Positions");
  } else {
    tracking=2; // Track Angles
    Serial.println("Tracking Angles");
  }
  // Set the last tracked position to some big number just  
  // to make the routine report the current position.
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    tracked[iMotor]=32000;
  }
}

void StopTracking(){
  tracking=0; // Track none
  Serial.println("Tracking Off");
}
 
void TrackReport(motor_id_t motor_id) {
  if (tracking>0){
    int Position = mm6_get_encoder(motor_id);
    if (tracked[motor_id]!=Position) {
      Serial.print("@");
      if (tracking==1) {
        Serial.print(char(motor_id+65));
        Serial.print(mm6_get_encoder(motor_id));    
      } else if (tracking==2) {
        Serial.print(char(motor_id+97));
        Serial.print(mm6_get_angle(motor_id));    
      }    
      Serial.print(":HS");
      Serial.print(motor_state[motor_id].switch_previously_triggered);
      Serial.println(":");
      tracked[motor_id]=Position;
    }
  }
}

//*************************************
// A test to see if the motor's current positions
//  are equal to the Target Positions.
// Which means that the motors are all at rest.
//*************************************
#if 0
int AllAtTarget(){
  // Calculate and return a value that indicates that 
  // all motors have reached the target position.
  return ((motor_state[MOTOR_ID_A].progress == 0) && 
          (motor_state[MOTOR_ID_B].progress == 0) && 
          (motor_state[MOTOR_ID_C].progress == 0) && 
          (motor_state[MOTOR_ID_D].progress == 0) && 
          (motor_state[MOTOR_ID_E].progress == 0) && 
          (motor_state[MOTOR_ID_F].progress == 0));
}
#endif

void ZeroPositions() {
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    noinit_data.encoder[iMotor] = 0;
    motor_state[iMotor].target_encoder = 0;
  }
  Serial.println("Current Positions set to Zero.");
}

void SetHomeToCenterOfSwitches() {
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    SetNewHome(iMotor);
  }
  Serial.println("Home Positions set to Center Of Switches.");
}

void SetPositionToHome() {
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    motor_state[iMotor].target_encoder = 0;
  }  
  Serial.println("Setting Targets to Home Position.");
}

/*
void ShowHelp() {
  Serial.println("Command List:");
  Serial.println("  +  : Target Position +10");
  Serial.println("  ++ : Target Position +100");
  Serial.println("  -  : Target Position -10");
  Serial.println("  -- : Target Position -100");
  Serial.println("  G  : Go (Turn On PID)");        
  Serial.println("  H  : SetTarget Positions To Home");
  Serial.println("  I  : Interrogate Limit Switches");
  Serial.println("  K  : Run Way Point Seq");
  Serial.println("  M  : Test Motors");
  Serial.println("  N  : Show firmware Ver No.");
  Serial.println("  O  : Open Gripper");
  Serial.println("  P  : Show Current Positions");
  Serial.println("  Q  : Run Test Seq 50 times");
  Serial.println("  R  : Set Rhino Ver");
  Serial.println("  S  : Stop (Turn Off PID");
  Serial.println("  T  : Show Status");
  Serial.println("  U  : Stop Tracking");
  Serial.println("  V  : CloseGripper");
  Serial.println("  W  : StartTracking");
  Serial.println("  X  : Set Home to Center of Switches");
  Serial.println("  Z  : Set Current Positions to 0");
  Serial.println("  ~  : Set 0-Angles to Current Positions");  
  Serial.println("  ?  : Show this list");  
  Serial.println("---------------------------------");
}
*/

/*
  Serial.println("Command List:");
  Serial.println("+  : Target Position +10");
  Serial.println("++ : Target Position +100");
  Serial.println("-  : Target Position -10");
  Serial.println("-- : Target Position -100");
  Serial.println("GO  : Open Gripper");
  Serial.println("GC  : Close Gripper");
  Serial.println("G?  : Show Gripper Status");
  Serial.println("HM  : Set Target Positions To Home");
  Serial.println("HF  : Find Home (Center of Limit Switches");
  Serial.println("HZ  : Set Home to Center of Switches");
  Serial.println("MT  : Test Motors");
  Serial.println("M0  : Turn Motors Off (PID");
  Serial.println("M1  : Turn Motors On (PID)");
  Serial.println("M?  : Show Motor Status");
  Serial.println("PZ  : Set Current Positions to 0");  
  Serial.println("PA  : Set 0-Angles to Current Positions");  
  Serial.println("P?  : Show Current Positions and Angles");
  Serial.println("T0  : Stop Tracking");
  Serial.println("T1  : Start Tracking");
  Serial.println("T?  : Show Tracking");
  Serial.println("V1 - V4  : Set Rhino Ver");
  Serial.println("V?  : Show Firmware and Rhino Ver No.");
  Serial.println("?  : Show this list");  

  Serial.println("Q  : Run Test Seq 50 times");
  Serial.println("K  : Run Way Point Seq");
  Serial.println("---------------------------------");
 */

void OpenGripper(){
  Gripper_StallX=0;
  motor_state[MOTOR_ID_A].target_encoder = config.gripper_open_encoder;
  Serial.println("Opening Gripper.");
}

void CloseGripper(){
  Gripper_StallX=0;
  motor_state[MOTOR_ID_A].target_encoder = config.gripper_close_encoder;
  Serial.println("Closing Gripper.");
}

//**********************************************
// Move a single Motor to a specified position. 
//**********************************************
void MoveMotorToE(int zMotor, int Position) {
  motor_state[zMotor].target_encoder = Position;
  motor_state[zMotor].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET;  // Set the flag that indicates that the motor has been put in motion.
  Serial.print(" ->Move Motor ");
  Serial.print(char(zMotor+65));
  Serial.print(" to ");
  Serial.println(Position);  
}

//***********************************************************
// Move Motors in synchronous mode by specifying the angles. 
// (Except the Gripper)
//***********************************************************
void SyncMoveAngle(float AngleB, float AngleC, float AngleD, float AngleE, float AngleF) {
  int PositionB = mm6_angle_to_encoder(MOTOR_ID_B, AngleB);
  int PositionC = mm6_angle_to_encoder(MOTOR_ID_C, AngleC);
  int PositionD = mm6_angle_to_encoder(MOTOR_ID_D, AngleD);
  int PositionE = mm6_angle_to_encoder(MOTOR_ID_E, AngleE);
  int PositionF = mm6_angle_to_encoder(MOTOR_ID_F, AngleF);
  SyncMove(PositionB, PositionC, PositionD, PositionE, PositionF);
}

//**************************************************************
// Move Motors in synchronous mode by specifying the positions. 
// (Except the Gripper)
//**************************************************************
void SyncMove(int PositionB, int PositionC, int PositionD, int PositionE, int PositionF) {
  motor_sync_move_enabled = false;  
  SyncMove_Status = MOTOR_PROGRESS_ON_WAY_TO_TARGET;
  // Store the target positions.
  End[MOTOR_ID_B]=PositionB;
  End[MOTOR_ID_C]=PositionC;
  End[MOTOR_ID_D]=PositionD;
  End[MOTOR_ID_E]=PositionE;
  End[MOTOR_ID_F]=PositionF;
  float MaxDistance = 0;
  // Caculate the travel distance for each motor.
  for (int iMotor = MOTOR_ID_B; iMotor <= MOTOR_ID_F; iMotor++){
    Start[iMotor] = motor_state[iMotor].target_encoder;
    Distance[iMotor]=End[iMotor]-Start[iMotor];
    // Keep track of the furthest travel distance.
    MaxDistance = max(MaxDistance,abs(Distance[iMotor]));
  }
  // Using the motor with the furthest travel distance,
  // caculate the ratios of travel distance between all motors.
  for (int iMotor = MOTOR_ID_B; iMotor <= MOTOR_ID_F; iMotor++){
    Ratio[iMotor] = Distance[iMotor]/MaxDistance;
    if (abs(Distance[iMotor])==MaxDistance) {
      LeadMotor = iMotor;
    }
    Serial.print(char(iMotor+65));
    Serial.print(": From:");
    Serial.print(Start[iMotor]);
    Serial.print(" To:");
    Serial.print(End[iMotor]);
    Serial.print(" Distance:");
    Serial.print(Distance[iMotor]);
    Serial.print(" Speed Ratio:");  
    Serial.println(Ratio[iMotor] * 100);
  }  
  motor_state[LeadMotor].target_encoder = End[LeadMotor];  
  SyncMove_Status = MOTOR_PROGRESS_ON_WAY_TO_TARGET;
  motor_sync_move_enabled = true;
  Serial.println("Start Sync Move");
}

void InterrogateLimitSwitches(){
  Serial.println("Interrogate Limit Switches");  
  Serial.print("  ");
  int CurrentMotorState = mm6_pid_enabled ? 1 : 0;  
  mm6_pid_enable(true);    
  InterrogateLimitSwitches2();
  Serial.println("  Done Interrogating Limit Switches");  
  if (CurrentMotorState == 0) {
    // mm6_pid_enable(false);
  }  
}

void Reverse(motor_id_t motor_id) {
  noinit_data.encoder[motor_id] *= -1;
  motor_state[motor_id].target_encoder *= -1;
  motor_state[motor_id].logic *= -1;
  // config_set_motor_orientation(motor_id, !config.motor[motor_id].orientation);
  Serial.print("Motor ");
  Serial.print(char(Command_Motor+65));    
  Serial.println(" reversed.");  
}

void Exercise(int m) {
  Serial.print("Exercise Motor ");    
  Serial.println(m);    
  int Rep = 0;
  mm6_pid_enable(true);  
  do {
      Rep++;
      Serial.print("Rep: ");    
      Serial.println(Rep);        
      switch (m)
      {
        case MOTOR_ID_F:; //A
          InterrogateLimitSwitch(MOTOR_ID_F, 450, -450);
          break;
        case MOTOR_ID_E:; //A
          InterrogateLimitSwitch(MOTOR_ID_E, 280, -280);
          break;
        case MOTOR_ID_D:; //A
          InterrogateLimitSwitch(MOTOR_ID_D, 320, -320);
          break;
        case MOTOR_ID_C:; //A
          InterrogateLimitSwitch(MOTOR_ID_C, 300, -300);
          break;
        case MOTOR_ID_B:; //A
          InterrogateLimitSwitch(MOTOR_ID_B, 380, -380);
          break;
        case MOTOR_ID_A:; //A
          Serial.println("Swing Gripper out.");
          motor_state[MOTOR_ID_A].target_encoder = 30;
          do {TrackReport(m);} while (noinit_data.encoder[m] < 25);
          Serial.println("Swing Gripper in.");
          motor_state[MOTOR_ID_A].target_encoder = -280;
          do {TrackReport(m);} while (noinit_data.encoder[m] > -275);  
          break;
      }   
      delay(500);
    } while (1);
}

void InterrogateLimitSwitches2(){
  InterrogateLimitSwitch(MOTOR_ID_F, 450, -450);
  InterrogateLimitSwitch(MOTOR_ID_E, 280, -280);
  InterrogateLimitSwitch(MOTOR_ID_D, 320, -320);
  InterrogateLimitSwitch(MOTOR_ID_C, 300, -300);
  InterrogateLimitSwitch(MOTOR_ID_B, 380, -380);
  InterrogateLimitSwitchA();
}

void InterrogateLimitSwitchA() {  
  MoveMotorToE(MOTOR_ID_A, 9999);
  delay(2000);
  int CurF = analogRead(motor_pinout[MOTOR_ID_A].in_current_draw);
  int EncF = noinit_data.encoder[MOTOR_ID_A];
  int SwcF = digitalRead(motor_pinout[MOTOR_ID_A].in_switch);
  MoveMotorToE(MOTOR_ID_A, -9999);
  delay(1500);
  int CurR = analogRead(motor_pinout[MOTOR_ID_A].in_current_draw);
  int EncR = noinit_data.encoder[MOTOR_ID_A];
  int SwcR = digitalRead(motor_pinout[MOTOR_ID_A].in_switch);  
  MoveMotorToE(MOTOR_ID_A, 9999);
  delay(1500);
  MoveMotorToE(MOTOR_ID_A, -9999);
  delay(1500);
    //Serial.print("  For Cur=");
    //Serial.println(CurF);
    //Serial.print("  For Enc=");
    //Serial.println(EncF);
    //Serial.print("  For Swt=");
    //Serial.println(SwcF);
    //Serial.print("  Rev Cur=");
    //Serial.println(CurR);
    //Serial.print("  Rev Enc=");
    //Serial.println(EncR);
    //Serial.print("  Rev Swt=");
    //Serial.println(SwcR);  
    //Serial.print("  motor_state[MOTOR_ID_A].switch_forward_on=");
    //Serial.println(motor_state[MOTOR_ID_A].switch_forward_on);        
  if (SwcF==0) {
    // Encoder goes Positive towards switch.
    int OverSwitch = ((motor_state[MOTOR_ID_A].switch_forward_on + EncF ) / 2);      
    MoveMotorToE(MOTOR_ID_A, OverSwitch);
    do {TrackReport(MOTOR_ID_A);} while (noinit_data.encoder[MOTOR_ID_A] != OverSwitch);
    noinit_data.encoder[MOTOR_ID_A] = 0;  
    motor_state[MOTOR_ID_A].target_encoder  = 0;
    motor_state[MOTOR_ID_A].logic = -1;
    // config_set_motor_orientation(MOTOR_ID_A, motor_state[MOTOR_ID_A].orientation);
    config_set_gripper_open_encoder(-140);
    config_set_gripper_close_encoder(-310);
    Serial.println("Done");
  } else {
    // Encoder goes Negative towards switch.
    int OverSwitch = ((motor_state[MOTOR_ID_A].switch_reverse_on + EncR ) / 2);      
    MoveMotorToE(MOTOR_ID_A, OverSwitch);
    do {TrackReport(MOTOR_ID_A);} while (noinit_data.encoder[MOTOR_ID_A] != OverSwitch);
    noinit_data.encoder[MOTOR_ID_A] = 0;  
    motor_state[MOTOR_ID_A].target_encoder  = 0;
    motor_state[MOTOR_ID_A].logic = 1;
    // config_set_motor_orientation(MOTOR_ID_A, motor_state[MOTOR_ID_A].orientation);
    config_set_gripper_open_encoder(140);
    config_set_gripper_close_encoder(310);
    Serial.println("Done");
  }
}

void InterrogateLimitSwitch(int m, int f, int r) {
  if (motor_state[m].switch_previously_triggered){
    //Serial.print(" Centering Motor ");
    //Serial.println(char(m+65));
    //Serial.print("   Moving to: ");
    noinit_data.encoder[m] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
    motor_state[m].target_encoder = 0;
    
    // Move to one side of switch and wait for the switch to be unpressed.
    Serial.print("  ");
    MoveMotorToE(m,r-130);
    do {TrackReport(m);} while (motor_state[m].switch_previously_triggered);

    // Move to the other side of switch and wait for the switch to be pressed and then unpressed.
    Serial.print("  ");
    MoveMotorToE(m,f+130);
    do {TrackReport(m);} while (!motor_state[m].switch_previously_triggered);
    do {TrackReport(m);} while (motor_state[m].switch_previously_triggered);
    //do {TrackReport(m);} while (mm6_get_encoder(m) < f);

    // Move back to first side of switch and wait for the switch to be pressed and then unpressed.        
    Serial.print("  ");
    MoveMotorToE(m,r-130);
    do {TrackReport(m);} while (!motor_state[m].switch_previously_triggered);
    do {TrackReport(m);} while (motor_state[m].switch_previously_triggered);
    //do {TrackReport(m);} while (mm6_get_encoder(m) > r);

    // Calculate center of switches and then move to that place.
    int CenterEncoder = ((motor_state[m].switch_forward_off + motor_state[m].switch_reverse_on + motor_state[m].switch_forward_on + motor_state[m].switch_reverse_off) / 4);
    Serial.print("   Switch Positions: ");
    Serial.print(motor_state[m].switch_reverse_off);
    Serial.print(",");
    Serial.print(motor_state[m].switch_forward_on);
    Serial.print(",");
    Serial.print(motor_state[m].switch_reverse_on);
    Serial.print(",");
    Serial.print(motor_state[m].switch_forward_off);
    Serial.println(".");
    Serial.print("     Centering");    
    MoveMotorToE(m,CenterEncoder);
    do {TrackReport(m);} while (noinit_data.encoder[m] != CenterEncoder);

    // Set Encoder and Target Values to 0.
    noinit_data.encoder[m] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
    motor_state[m].target_encoder = 0;
    
  } else {
    Serial.print(" Motor ");
    Serial.print(char(m+65));
    Serial.println(" Home Switch Not Closed. - Skipping.");
  }
}

void SetNewHome(int m) {
    int NewHome = ((motor_state[m].switch_forward_off + motor_state[m].switch_reverse_on + motor_state[m].switch_forward_on + motor_state[m].switch_reverse_off) / 4);
    MoveMotorToE(m,NewHome);
    do {TrackReport(m);} while (noinit_data.encoder[m] != NewHome);
    noinit_data.encoder[m] = 0;
    motor_state[m].target_encoder = 0;
    Serial.print(" Motor ");
    Serial.print(char(m+65));
    Serial.println(" Centered");  
}

void Serial_print(int a, int l) {
  String formatS ="%+0zd";
  String D = String(l+1);
  formatS.replace("z",D);
  char formatC[10];
  formatS.toCharArray(formatC,10);
  sprintf (padbuffer, formatC,a);
  Serial.print(padbuffer);
}

// ****************************************************
// ****************************************************
//
//                  Move Sequences
//
// ****************************************************
// ****************************************************

void TestSeq1() {
  for (int m=0;m<=50;m++){
    Serial.println(m);
    InterrogateLimitSwitches();
  }
}

void TestSeq1b() {
Serial.println("start");
  mm6_pid_enable(false);
  OpenGripper();
  mm6_set_target_angle(MOTOR_ID_B, 90);
  mm6_set_target_angle(MOTOR_ID_E, -130);
  mm6_set_target_angle(MOTOR_ID_F, 45);
  mm6_pid_enable(true);
  do {delay(50);} while (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_APPROACHING_TARGET); 
  mm6_set_target_angle(MOTOR_ID_D,17.1);
  do {delay(50);} while (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_APPROACHING_TARGET); 
  CloseGripper();
  delay(1000);
  mm6_set_target_angle(MOTOR_ID_D, 0);
  mm6_set_target_angle(MOTOR_ID_E, -80);
  do {delay(50);} while (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_APPROACHING_TARGET); 
  mm6_set_target_angle(MOTOR_ID_F,-45);
  do {delay(50);} while (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_APPROACHING_TARGET); 
  OpenGripper();
  delay(1000);
  SetPositionToHome();
  do {delay(50);} while (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_BESIDE_TARGET); 
  mm6_pid_enable(false); 
  Serial.println("stop");
}

void RunWayPointSeq() {
  Serial.println("Start WayPoints");
  EEPROM.get(0, WayPoint);
  int NumberOf = WayPoint.A;
  if (NumberOf < 100) {
    mm6_pid_enable(true);
    for (int Step = 1; Step <= NumberOf; Step++) 
    {
      int Pin = 4;
      int val = 0;
      int Stp = 0;
      Serial.print("Step: ");
      Serial.print(Step);
      Serial.print(" - ");
      int eeAddress = Step*40;   //Location we want the data to be put.
      EEPROM.get(eeAddress, WayPoint);
      switch (WayPoint.Command)
      {
        case 65:; //A
        case 66:; //B
        case 67:; //C
        case 68:; //D
          WayPointMove(Step);
          break;
        
        case 71: //G
          Stp = WayPoint.A;
          Step = Stp - 1;
          Serial.print("Goto Step ");
          Serial.println(Stp);        
          break;
        
        case 73: //I
          InterrogateLimitSwitches2();
          break;
  
        case 74: //J
          val = 0;
          Pin = WayPoint.B;
          Stp = WayPoint.A;
          Serial.print("Goto Step ");
          Serial.print(Stp);        
          Serial.print(" If I/O[");
          Serial.print(Pin);
          Serial.println("]");
          // pinMode(expansion_io_pinout[Pin-1], INPUT);
          // val = digitalRead(expansion_io_pinout[Pin-1]);
          if (val == 0) Step = Stp - 1;
          break;
          
        case 87: //W
          val = WayPoint.C;        
          Serial.print("Wait ");
          Serial.print(val);
          Serial.print(" Miliseconds");
          delay(val);
          break;
          
      }  
    }
    mm6_pid_enable(false);  
    Serial.println("Done");
  } else {
    Serial.println("No Waypoints");  
  }
}

void WayPointMove(int i) {
  Serial.print("Goto ");
  MoveToWayPointAngle(i);

  switch (WayPoint.Command)
  {
    case 65 :do {delay(50);TrackUm();} while (Check_A());
      break;
    case 66 :do {delay(50);TrackUm();} while (Check_B());
      break;
    case 67 :do {delay(50);TrackUm();} while (Check_C());
      break;
    case 68 :do {delay(50);TrackUm();} while (Check_D());
      break;
  }  
}

void TrackUm() {
  if (tracking>0) {
    for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
      TrackReport(iMotor);
    }
  }   
}

boolean Check_A() {
  return
  (motor_state[MOTOR_ID_C].progress > MOTOR_PROGRESS_AT_TARGET) ||   
  (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_AT_TARGET) ||   
  (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_AT_TARGET) ||   
  (motor_state[MOTOR_ID_F].progress > MOTOR_PROGRESS_AT_TARGET);       
}

boolean Check_B() {
  return
  (motor_state[MOTOR_ID_C].progress > MOTOR_PROGRESS_BESIDE_TARGET) ||   
  (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_BESIDE_TARGET) ||   
  (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_BESIDE_TARGET) ||   
  (motor_state[MOTOR_ID_F].progress > MOTOR_PROGRESS_BESIDE_TARGET);       
}

boolean Check_C() {
  return
  (motor_state[MOTOR_ID_C].progress > MOTOR_PROGRESS_NEAR_TARGET) ||   
  (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_NEAR_TARGET) ||   
  (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_NEAR_TARGET) ||   
  (motor_state[MOTOR_ID_F].progress > MOTOR_PROGRESS_NEAR_TARGET);       
}

boolean Check_D() {
  return
  (motor_state[MOTOR_ID_C].progress > MOTOR_PROGRESS_APPROACHING_TARGET) ||   
  (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_APPROACHING_TARGET) ||   
  (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_APPROACHING_TARGET) ||   
  (motor_state[MOTOR_ID_F].progress > MOTOR_PROGRESS_APPROACHING_TARGET);       
}

void SplitWayPoint(String waypoint)
{    
  for (int i = 0; i < 8; i++) 
  {
    StringSplits[i] = GetStringPartAtSpecificIndex(waypoint, '!', i); 
  }
}

String GetStringPartAtSpecificIndex(String StringToSplit, char SplitChar, int StringPartIndex)
{
  String originallyString = StringToSplit;
  String outString = "";
  for (int i1 = 0; i1 <= StringPartIndex; i1++)
  {
    outString = "";                   //if the for loop starts again reset the outString (in this case other part of the String is needed to take out)
    int SplitIndex = StringToSplit.indexOf(SplitChar);  //set the SplitIndex with the position of the SplitChar in StringToSplit

    if (SplitIndex == -1)               //is true, if no Char is found at the given Index
    {
      //outString += "Error in GetStringPartAtSpecificIndex: No SplitChar found at String '" + originallyString + "' since StringPart '" + (i1-1) + "'";    //just to find Errors
      return outString;
    }
    for (int i2 = 0; i2 < SplitIndex; i2++)
    {
      outString += StringToSplit.charAt(i2);      //write the char at Position 0 of StringToSplit to outString
    }
    StringToSplit = StringToSplit.substring(StringToSplit.indexOf(SplitChar) + 1);  //change the String to the Substring starting at the position+1 where last SplitChar found
  }
  return outString;
}

void SetZeroAngles() {
  int B = mm6_get_encoder(MOTOR_ID_B);
  int C = mm6_get_encoder(MOTOR_ID_C);
  int D = mm6_get_encoder(MOTOR_ID_D);
  int E = mm6_get_encoder(MOTOR_ID_E);
  int F = mm6_get_encoder(MOTOR_ID_F);
  config_set_angle_offsets(B, C, D, E, F);
}

void SetWayPointAngle() {  
  InBuffer.setCharAt(0,32);
  int Position = InBuffer.toInt();
  SplitWayPoint(InBuffer);  
  WayPoint.Number = Position;
  char Comm = StringSplits[1][0];
  WayPoint.Command = Comm;
  WayPoint.A = StringSplits[2].toFloat();
  WayPoint.B = StringSplits[3].toFloat();
  WayPoint.C = StringSplits[4].toFloat();
  WayPoint.D = StringSplits[5].toFloat();
  WayPoint.E = StringSplits[6].toFloat();
  WayPoint.F = StringSplits[7].toFloat();
  int eeAddress = Position*40;   //Location we want the data to be put.
  EEPROM.put(eeAddress, WayPoint);
  ReportWayPoint();
  
  Serial.println("Set");  
}

void GetWayPointAngle() {
  InBuffer.setCharAt(0,32);
  int Position = InBuffer.toInt();
  int eeAddress = Position*40;   //Location we want the data to be put.
  EEPROM.get(eeAddress, WayPoint);
  ReportWayPoint();
}

void ReportWayPoint() {
  Serial.print("WayPoint:");
  Serial.print(WayPoint.Number);
  Serial.print("!");
  Serial.print(WayPoint.Command);
  Serial.print("!");
  Serial.print(WayPoint.A);
  Serial.print("!");
  Serial.print(WayPoint.B);
  Serial.print("!");
  Serial.print(WayPoint.C);
  Serial.print("!");
  Serial.print(WayPoint.D);
  Serial.print("!");
  Serial.print(WayPoint.E);
  Serial.print("!");
  Serial.print(WayPoint.F);  
  Serial.println("!");
}

void MoveToAWayPointAngle() {
  InBuffer.setCharAt(0,32);
  int Position = InBuffer.toInt();
  Serial.print("Goto ");
  MoveToWayPointAngle(Position);
}

void MoveToWayPointAngle(int Position) {
  if (Position > 0) {
    int eeAddress = Position*40;   //Location we want the data to be put.
    EEPROM.get(eeAddress, WayPoint);
    ReportWayPoint();    
    for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
      float Angle = 0;
      switch (iMotor) {
        case MOTOR_ID_F: Angle = WayPoint.F;
          break;
        case MOTOR_ID_E: Angle = WayPoint.E;
          break;
        case MOTOR_ID_D: Angle = WayPoint.D;
          break;
        case MOTOR_ID_C: Angle = WayPoint.C;
          break;
        case MOTOR_ID_B: Angle = WayPoint.B;
          break;
        case MOTOR_ID_A: Angle = WayPoint.A;
      }      
      mm6_set_target_angle(iMotor, Angle);  
    }  
  }  
}

void RunMotorsForAsmTest() {
  Serial.println("Run Motors For Asm Test");    
  mm6_pid_enable(false);
  delay(250);

  // Set all motor power lines to High-Z state by 
  // Setting speed to 0 and turning off brakes.  
  Serial.println("Setting Drive Power for all Motors to High-Z.");
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){    
    mm6_set_brake(iMotor, false);
    mm6_set_speed(iMotor, 0);
  }  

  int MDir = 1;
  for (int z=0; z<10000; z++){
    for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){    
      MotorTestFullSpeed(iMotor, MDir);
    }
    delay(5000);               // Run for 5 sec.
    for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){    
      MotorTestStop(iMotor);
    }
    delay(100);                // Short Delay to allow the motor to stop.  
    MDir = MDir * -1;
    Serial.println("Reverse.");
  }
}

void MotorTestStop(int m) {
  mm6_set_speed(m, 0);
}

void MotorTestFullSpeed(int m, int direction) 
{
  int TestSpeed = 255 - motor_min_speed;
  mm6_set_speed(m, TestSpeed * direction);
}