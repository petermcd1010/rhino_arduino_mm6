#pragma once

/*
 * Declarations for non-MegaMotor6-specific hardware functionality.
 */

#include <Arduino.h>

void hardware_init();
void hardware_erase_eeprom();
void hardware_reset();
void hardware_reboot();
bool hardware_get_led();
void hardware_set_led(bool enable);
