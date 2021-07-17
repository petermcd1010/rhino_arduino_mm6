/*
 * Configuration functions.
 */

bool config_check()
{
  // If there's an error in the header, all bets are off. Just return false.
  if (config.nbytes != sizeof(config_t)) {
    log_writeln(F("ERROR: config_check: Invalid config block size."));
    return false;
  }

  if (config.version != config_version) {
    log_writeln(F("ERROR: config_check: Invalid config version %d."), config.version);
    return false;
  }

  if (config.magic != config_magic) {
    log_writeln(F("ERROR: config_check: Invalid config magic %08x."), config.magic);
    return false;
  }

  long given_crc = config.crc;
  config.crc = 0;
  long calculated_crc = crc_calculate(&config, sizeof(config_t));
  config.crc = given_crc;  // Restore it.
  if (calculated_crc != given_crc) {
    log_writeln(F("ERROR: config_check: Invalid CRC %08lx. Expected %08lx."), calculated_crc, config.crc);
    return false;
  }

  // If there's an error in the contents, keep going, but return false.
  bool ret = true;

  if ((config.robot_id < ROBOT_ID_FIRST) || (config.robot_id > ROBOT_ID_LAST)) {
    log_writeln(F("ERROR: config_check: Invalid robot id %d."), config.robot_id);
    ret = false;
  }

  if (!buffer_contains(config.robot_serial, config_robot_serial_nbytes, '\0')) {
    log_writeln(F("ERROR: config_check: Robot serial isn't null-terminated."));
    ret = false;
  }

  if (!buffer_contains(config.robot_name, config_robot_name_nbytes, '\0')) {
    log_writeln(F("ERROR: config_check: Robot name isn't null-terminated."));
    ret = false;
  }

  const int min_count = -9999;  // TODO.
  const int max_count = 9999;  // TODO.
  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    if ((config.motor[i].angle_offset < min_count) || (config.motor[i].angle_offset > max_count)) {
      log_writeln(F("ERROR: config_check: Invalid angle offset."));
      ret = false;
    }

    if ((config.motor[i].motor_logic & 0xfffe) != 0) {  // Confirm is 0 or 1.
      log_writeln(F("ERROR: config_check: Invalid motor logic."));
      ret = false;
    }

    if ((config.motor[i].direction_logic & 0xfffe) != 0) {  // Confirm is 0 or 1.
      log_writeln(F("ERROR: config_check: Invalid direction logic."));
      ret = false;
    }
  }

  if ((config.gripper_open_location < min_count) || (config.gripper_open_location > max_count)) {
    log_writeln(F("ERROR: config_check: Invalid gripper_open."));
  }

  if ((config.gripper_close_location < min_count) || (config.gripper_close_location > max_count)) {
    log_writeln(F("ERROR: config_check: Invalid gripper_open."));
  }

  return true;
}

bool config_read() 
{
  EEPROM.get(config_base_address, config);
  return config_check();
}

void config_sign()
{
  config.nbytes = sizeof(config_t);
  config.version = config_version;
  config.magic = config_magic;
  config.crc = 0;  
  config.crc = crc_calculate(&config, sizeof(config_t)); 
}

void config_clear() 
{
  memset(&config, 0, sizeof(config_t));
  config.robot_id = ROBOT_ID_DEFAULT;
  config_sign();
}

bool config_write()
{
  if (!config_check())
    return false;

  EEPROM.put(config_base_address, config); // TODO: Return value?
  return true;
}

void config_set_robot_id(robot_id_t robot_id)
{
  assert(robot_id >= ROBOT_ID_FIRST);
  assert(robot_id <= ROBOT_ID_LAST);

  assert(config_check());
  config.robot_id = robot_id;
  config_sign();
}

void config_set_robot_serial(char robot_serial[config_robot_serial_nbytes])
{
  assert(robot_serial);
  // TODO: Verify it's null-terminated.
  assert(config_check());
  memcpy(config.robot_serial, robot_serial, config_robot_serial_nbytes);
  config_sign();
}

void config_set_robot_name(char robot_name[config_robot_name_nbytes])
{
  assert(robot_name);
  // TODO: Verify it's null-terminated.
  assert(config_check());
  memcpy(config.robot_name, robot_name, config_robot_name_nbytes);
  config_sign();
}

void config_set_motor_configured(motor_id_t motor_id, bool configured)
{  
  assert(motor_id >= MOTOR_ID_FIRST && motor_id <= MOTOR_ID_LAST);
  
  assert(config_check());
  config.motor[motor_id].configured = configured;
  config_sign();
}

void config_set_motor_logic(motor_id_t motor_id, int motor_logic)
{  
  assert(motor_id >= MOTOR_ID_FIRST && motor_id <= MOTOR_ID_LAST);
  assert(motor_logic & 0xfffe == 0); // Motor logic == 0 or 1.
  
  assert(config_check());
  config.motor[motor_id].motor_logic = motor_logic;
  config_sign();
}

void config_set_direction_logic(motor_id_t motor_id, int direction_logic)
{  
  assert(motor_id >= MOTOR_ID_FIRST && motor_id <= MOTOR_ID_LAST);
  assert((direction_logic & 0xfffe) == 0); // Direction logic == 0 or 1.
  // TODO: assert logic is in valid range;
  
  assert(config_check());
  config.motor[motor_id].direction_logic = direction_logic;
  config_sign();
}

void config_set_gripper_open_location(int location)
{  
  // TODO: assert location is in valid range.
  
  assert(config_check());
  config.gripper_open_location = location;
  config_sign();
}

void config_set_gripper_close_location(int location)
{  
  // TODO: assert location is in valid range.
  
  assert(config_check());
  config.gripper_close_location = location;
  config_sign();
}

void config_set_angle_offsets(int B, int C, int D, int E, int F) {
  config.motor[MOTOR_ID_B].angle_offset = B;
  config.motor[MOTOR_ID_C].angle_offset = C;
  config.motor[MOTOR_ID_D].angle_offset = D;
  config.motor[MOTOR_ID_E].angle_offset = E;
  config.motor[MOTOR_ID_F].angle_offset = F;
  log_write(F("Angle Offsets Set to: %d, %d, %d, %d, %d."), 
      config.motor[MOTOR_ID_B].angle_offset, 
      config.motor[MOTOR_ID_B].angle_offset, 
      config.motor[MOTOR_ID_B].angle_offset, 
      config.motor[MOTOR_ID_B].angle_offset, 
      config.motor[MOTOR_ID_B].angle_offset);
}

void config_display()
{  
  // TODO: check for valid config.

  log_writeln(F("Configuration:"));
  log_writeln(F("  Robot ID: '%s'."), robot_name_by_robot_id[config.robot_id]);
  log_writeln(F("  Robot serial: '%s'."), config.robot_serial);
  log_writeln(F("  Robot name: '%s'."), config.robot_name);

  char str[15] = {};

  for (int i = 0; i < MOTOR_ID_COUNT; i++) {
    log_write(F("  Motor %c: "), 'A' + i);
    if (!config.motor[i].configured) {
      log_writeln(F("not configured."));
    } else {
      dtostrf(config.motor[i].angle_offset, 3, 2, str);
      log_writeln(F("angle_offset:%s motor_logic:%s, direction_logic:%s."), 
          str, 
          config.motor[i].motor_logic ? "forward" : "reverse",
          config.motor[i].direction_logic ? "forward" : "reverse");
    }
  }

  log_writeln(F("  Gripper open location: %d."), config.gripper_open_location);
  log_writeln(F("  Gripper close location: %d."), config.gripper_close_location);

  log_writeln(F(""));
}

bool test_config() {
  // TODO.
  // test_config_check
  // test_config_read -- skip, as we don't want to exercise the EEPROM.
  // test_config_write -- skip, as we don't want to exercise the EEPROM.
  return true;
}
