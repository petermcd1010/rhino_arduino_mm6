/*
 * Implementation for non-MegaMotor6 hardware functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <avr/sleep.h>
#include <EEPROM.h>
#include "hardware.h"
#include "log.h"
#include "motor.h"


static const char pin_name_speaker[] PROGMEM = "Speaker";
static const char pin_name_a14[] PROGMEM = "A14";
static const char pin_name_a13[] PROGMEM = "A13";
static const char pin_name_a12[] PROGMEM = "A12";
static const char pin_name_53[] PROGMEM = "53";
static const char pin_name_49[] PROGMEM = "49";
static const char pin_name_48[] PROGMEM = "48";
static const char pin_name_41[] PROGMEM = "41";

const char *const hardware_gpio_pin_name_by_index[HARDWARE_GPIO_PIN_COUNT] = {
    pin_name_speaker,
    pin_name_a14,
    pin_name_a13,
    pin_name_a12,
    pin_name_53,
    pin_name_49,
    pin_name_48,
    pin_name_41
};

static const int OPRLED = 13;
static const int SPEAKER = A15;

volatile uint8_t hardware_gpio_pin_value[HARDWARE_GPIO_PIN_COUNT] = { 0 };
static volatile unsigned long prev_transition_millis[HARDWARE_GPIO_PIN_COUNT] = { 0 };

static const int arduino_pin_by_index[] = { SPEAKER, A14, A13, A12, 53, 49, 48, 41 };


void hardware_init(void)
{
    pinMode(OPRLED, OUTPUT);
    hardware_set_led_enabled(false);
    hardware_set_speaker_enabled(false);
}

void hardware_erase_eeprom(void)
{
    log_write(F("Erasing configuration data in Arduino EEPROM... "));
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

void hardware_set_gpio_pin_mode(hardware_gpio_pin_t gpio_pin, hardware_gpio_pin_mode_t gpio_mode)
{
    assert(gpio_pin >= 0 && gpio_pin < HARDWARE_GPIO_PIN_COUNT);

    if (gpio_mode == HARDWARE_GPIO_PIN_MODE_INPUT)
        pinMode(arduino_pin_by_index[gpio_pin], INPUT);
    else if (gpio_mode == HARDWARE_GPIO_PIN_MODE_INPUT_PULLUP)
        pinMode(arduino_pin_by_index[gpio_pin], INPUT_PULLUP);
    else if (gpio_mode == HARDWARE_GPIO_PIN_MODE_OUTPUT)
        pinMode(arduino_pin_by_index[gpio_pin], OUTPUT);
    else
        assert(false);
}

hardware_gpio_pin_mode_t hardware_get_gpio_pin_mode(hardware_gpio_pin_t gpio_pin)
{
    assert(gpio_pin >= 0 && gpio_pin < HARDWARE_GPIO_PIN_COUNT);

    int arduino_pin = arduino_pin_by_index[gpio_pin];

    assert(arduino_pin < NUM_DIGITAL_PINS);

    uint8_t bit = digitalPinToBitMask(arduino_pin);
    uint8_t port = digitalPinToPort(arduino_pin);
    volatile uint8_t *reg = portModeRegister(port);

    if (*reg & bit)
        return HARDWARE_GPIO_PIN_MODE_OUTPUT;

    volatile uint8_t *out = portOutputRegister(port);

    if (*out & bit)
        return HARDWARE_GPIO_PIN_MODE_INPUT_PULLUP;

    return HARDWARE_GPIO_PIN_MODE_INPUT;
}

void hardware_set_gpio_pin_output(hardware_gpio_pin_t gpio_pin, bool is_high)
{
    assert(gpio_pin >= 0 && gpio_pin < HARDWARE_GPIO_PIN_COUNT);
    assert(hardware_get_gpio_pin_mode(gpio_pin) == HARDWARE_GPIO_PIN_MODE_OUTPUT);

    digitalWrite(arduino_pin_by_index[gpio_pin], is_high ? HIGH : LOW);
}

bool hardware_read_gpio_pin(hardware_gpio_pin_t gpio_pin)
{
    assert(gpio_pin >= 0 && gpio_pin < HARDWARE_GPIO_PIN_COUNT);
    assert(hardware_get_gpio_pin_mode(gpio_pin) != HARDWARE_GPIO_PIN_MODE_OUTPUT);

    return digitalRead(arduino_pin_by_index[gpio_pin]);
}

bool hardware_get_gpio_pin_pressed(hardware_gpio_pin_t gpio_pin)
{
    assert(gpio_pin >= 0 && gpio_pin < HARDWARE_GPIO_PIN_COUNT);
    assert(hardware_get_gpio_pin_mode(gpio_pin) != HARDWARE_GPIO_PIN_MODE_OUTPUT);

    if (hardware_get_gpio_pin_mode(gpio_pin) == HARDWARE_GPIO_PIN_MODE_OUTPUT)
        return false;

    unsigned const debounce_delay_millis = 50;

    // TODO: Look into millis() rollover and handle appropriately.
    if ((millis() - prev_transition_millis[gpio_pin]) > debounce_delay_millis)
        // TODO: latch if it's been in a constant state for > debounce_delay_millis.
        return hardware_gpio_pin_value[gpio_pin] == HIGH;  // Button is pressed if pin reads HIGH.

    return false;
}

void hardware_debounce_buttons()
{
    // Called from motor ISR.
    for (int gpio_pin = 0; gpio_pin < HARDWARE_GPIO_PIN_COUNT; gpio_pin++) {
        if (hardware_get_gpio_pin_mode(gpio_pin) != HARDWARE_GPIO_PIN_MODE_OUTPUT) {
            int value = digitalRead(arduino_pin_by_index[gpio_pin]);
            if (hardware_gpio_pin_value[gpio_pin] != value) {
                prev_transition_millis[gpio_pin] = millis();
                hardware_gpio_pin_value[gpio_pin] = value;
            }
        }
    }
}
