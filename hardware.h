#pragma once

/*
 * Declarations for non-MegaMotor6 hardware functionality.
 */

#include <Arduino.h>

void hardware_init();
void hardware_erase_eeprom();
void hardware_reset();
void hardware_reboot();
bool hardware_get_led();
void hardware_set_led(bool enable);
bool hardware_get_speaker();
void hardware_set_speaker(bool enable);  // expansion_io_pinout 1 can be wired to a speaker.
