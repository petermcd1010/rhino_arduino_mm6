/*
 * Implementation for command processing functions.
 */

#include <assert.h>
#include <stdlib.h>
#include "command.h"
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "menu.h"
#include "mm6.h"
#include "parse.h"
#include "sm.h"

extern void print_software_version();  // TODO: remove.

// TODO: Move the following two functions somewhere else.
void SetPositionToHome() {
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    motor_state[iMotor].target_encoder = 0;
  }  
  log_writeln(F("Setting Targets to Home Position."));
}

void ZeroPositions() {
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    noinit_data.encoder[iMotor] = 0;
    motor_state[iMotor].target_encoder = 0;
  }
  log_writeln(F("Current Positions set to Zero."));
}

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

  mm6_enable_all(false);
  sm_state_current = SM_STATE_ERROR;

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

int command_print_motor_status(char *pargs, size_t args_nbytes)
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
