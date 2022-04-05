/*
 * Implementation for non-MegaMotor6 hardware functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#define __ASSERT_USE_STDERR
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <EEPROM.h>
#include "hardware.h"
#include "log.h"
#include "motor.h"

static const int OPRLED = 13;
static const int expansion_io_pinout[] = { A15, A14, A13, A12, 53, 49, 48, 41 };

void hardware_init(void)
{
    pinMode(OPRLED, OUTPUT);
    pinMode(expansion_io_pinout[0], OUTPUT);  // Speaker tone.
}

void hardware_erase_eeprom(void)
{
    log_write(F("Erasing Arduino EEPROM... "));
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
    }
    log_writeln(F("Completed. Zeroed %d bytes."), EEPROM.length());
}

void hardware_factory_reset(void)
{
    motor_disable_all();
    motor_clear_ram_data();
    hardware_erase_eeprom();
}

void hardware_halt(void)
{
    motor_disable_all();
    hardware_set_led_enabled(false);
    log_writeln(F("\nHardware halted. Press reset button to reboot."));
    log_flush();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();  // Disable interrupts
    sleep_mode();
}


void hardware_reboot(void)
{
    static void (*hardware_really_reboot)(void) = 0;  // Call hardware_really_reboot() to reset the board.

    motor_disable_all();
    delay(1000);  // Wait 1s for log output to complete writing.
    hardware_really_reboot();
}

bool hardware_get_led_enabled(void)
{
    return digitalRead(OPRLED) != 0;
}

void hardware_set_led_enabled(bool enabled)
{
    digitalWrite(OPRLED, enabled);
}

bool hardware_get_speaker_enabled(void)
{
    return digitalRead(expansion_io_pinout[0]) != 0;
}

void hardware_set_speaker_enabled(bool enabled)
{
    // expansion_io_pinout 1 can be wired to a speaker.
    digitalWrite(expansion_io_pinout[0], enabled);
}
