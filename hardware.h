#pragma once
/*
 * Declarations for non-MegaMotor6 hardware functionality.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

typedef enum {
    HARDWARE_GPIO_PIN_SPEAKER = 0,
    HARDWARE_GPIO_PIN_A14,
    HARDWARE_GPIO_PIN_A13,
    HARDWARE_GPIO_PIN_A12,
    HARDWARE_GPIO_PIN_53,
    HARDWARE_GPIO_PIN_49,
    HARDWARE_GPIO_PIN_48,
    HARDWARE_GPIO_PIN_41,
    HARDWARE_GPIO_PIN_COUNT,
} hardware_gpio_pin_t;

typedef enum {
    HARDWARE_GPIO_PIN_MODE_INPUT        = 0,  // Power-up default. High-impedence.
    HARDWARE_GPIO_PIN_MODE_INPUT_PULLUP = 1,  // Inverts behavior of input pin.
    HARDWARE_GPIO_PIN_MODE_OUTPUT       = 2,  // Low-impedence.
    HARDWARE_GPIO_PIN_MODE_COUNT,
    HARDWARE_GPIO_PIN_MODE_DEFAULT      = HARDWARE_GPIO_PIN_MODE_INPUT,
} hardware_gpio_pin_mode_t;

extern const char *const hardware_gpio_pin_name_by_index[HARDWARE_GPIO_PIN_COUNT];
extern volatile uint8_t hardware_gpio_pin_value[HARDWARE_GPIO_PIN_COUNT];

void hardware_init(void);
void hardware_erase_eeprom(void);
void hardware_factory_reset(void);
void hardware_halt(void);
void hardware_reboot(void);
bool hardware_get_led_enabled(void);
void hardware_set_led_enabled(bool enabled);
bool hardware_get_speaker_enabled(void);
void hardware_set_speaker_enabled(bool enabled);  // expansion_io_pinout 1 can be wired to a speaker.
void hardware_set_gpio_pin_mode(hardware_gpio_pin_t gpio_pin, hardware_gpio_pin_mode_t gpio_mode);
hardware_gpio_pin_mode_t hardware_get_gpio_pin_mode(hardware_gpio_pin_t gpio_pin);
void hardware_set_gpio_pin_output(hardware_gpio_pin_t gpio_pin, bool is_high);
bool hardware_read_gpio_pin(hardware_gpio_pin_t gpio_pin);
bool hardware_get_gpio_pin_pressed(hardware_gpio_pin_t gpio_pin);
void hardware_debounce_buttons(void);  // Called from motor ISR.
