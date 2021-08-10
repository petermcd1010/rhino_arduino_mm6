/*
 * Implementation for non-MegaMotor6 hardware functionality.
 */

#define __ASSERT_USE_STDERR
#include <assert.h>
#include <stdlib.h>
#include <EEPROM.h>
#include "hardware.h"
#include "log.h"
#include "motor.h"

static const int OPRLED = 13;
static const int expansion_io_pinout[] = { A15, A14, A13, A12, 53, 49, 48, 41 };

void hardware_init() {
  pinMode(OPRLED, OUTPUT);
  pinMode(expansion_io_pinout[0], OUTPUT); // Tone
}

void hardware_erase_eeprom()
{
  log_write(F("Erasing EEPROM... "));
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);    
  }
  log_writeln(F("Completed. Zeroed %d bytes."), EEPROM.length());
}

void hardware_reset()
{
  motor_set_pid_enable_all(false);
  hardware_erase_eeprom();
  hardware_reboot();
}

void (*hardware_really_reboot)(void) = 0;  // Call hardware_really_reboot() to reset the board.
void hardware_reboot() 
{
  motor_set_pid_enable_all(false);
  delay(1000);  // Wait 1s for log output to complete writing.
  hardware_really_reboot();
}

bool hardware_get_led()
{
  return digitalRead(OPRLED) != 0;
}

void hardware_set_led(bool enable)
{
  digitalWrite(OPRLED, enable);
}

bool hardware_get_speaker()
{
  return digitalRead(expansion_io_pinout[0]) != 0;
}

void hardware_set_speaker(bool enable)
{
  // expansion_io_pinout 1 can be wired to a speaker.
  digitalWrite(expansion_io_pinout[0], enable); 
}
