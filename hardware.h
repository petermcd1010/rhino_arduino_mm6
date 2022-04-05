#pragma once
/*
 * Declarations for non-MegaMotor6 hardware functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

void hardware_init(void);
void hardware_erase_eeprom(void);
void hardware_factory_reset(void);
void hardware_halt(void);
void hardware_reboot(void);
bool hardware_get_led_enabled(void);
void hardware_set_led_enabled(bool enabled);
bool hardware_get_speaker_enabled(void);
void hardware_set_speaker_enabled(bool enabled);  // expansion_io_pinout 1 can be wired to a speaker.
