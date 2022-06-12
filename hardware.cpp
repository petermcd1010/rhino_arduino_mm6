/*
 * Implementation for non-MegaMotor6 hardware functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <EEPROM.h>
#include "hardware.h"
#include "log.h"
#include "motor.h"

static const int OPRLED = 13;
static const int SPEAKER = A15;
static const int BUTTON = A14;

static const int header_pins[] = { A15, A14, A13, A12, 53, 49, 48, 41 };
#define NUM_HEADER_PINS (sizeof(header_pins) / sizeof(header_pins[0]))

void hardware_init(void)
{
    pinMode(OPRLED, OUTPUT);
    hardware_set_led_enabled(false);
    pinMode(SPEAKER, OUTPUT);  // Speaker wired to mm6 expansion header '01'
    hardware_set_speaker_enabled(false);
    pinMode(BUTTON, INPUT);  // External button wired to mm6 expansion header '02'
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
    return digitalRead(OPRLED) == 0;  // LED is active low.
}

void hardware_set_led_enabled(bool enabled)
{
    digitalWrite(OPRLED, !enabled);  // LED is active low.
}

bool hardware_get_speaker_enabled(void)
{
    return digitalRead(SPEAKER) == 0;
}

void hardware_set_speaker_enabled(bool enabled)
{
    digitalWrite(SPEAKER, !enabled);
}

int hardware_get_num_header_pins()
{
    return NUM_HEADER_PINS;
}

bool hardware_get_header_pin_pressed(int pin_index)
{
    assert(pin_index < NUM_HEADER_PINS);

    static uint8_t ret_vals[NUM_HEADER_PINS] = { 0 };
    static uint8_t prev_vals[NUM_HEADER_PINS] = { -1 };

    static unsigned prev_transition_millis = -1;
    unsigned const debounce_delay_millis = 50;

    pinMode(header_pins[pin_index], INPUT);
    int val = digitalRead(header_pins[pin_index]);

    // TODO: Look into millis() rollover and handle appropriately.

    if (val != prev_vals[pin_index]) {
        prev_transition_millis = millis();
        prev_vals[pin_index] = val;
    }

    if ((millis() - prev_transition_millis) > debounce_delay_millis)
        ret_vals[pin_index] = val;

    return ret_vals[pin_index];
}
