/*
 * Implementation for state machine functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "command.h"
#include "crc32c.h"
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "menu.h"
#include "motor.h"
#include "parse.h"
#include "sm.h"

// Uncomment the following line to add 'Leaving state [name]' and 'Entering state [name]' logs.
// #define DEBUG_STATE

sm_state_t current_state;
sm_state_t next_state;

static int enabled_motors_mask = 0;
static bool reset_prompt = true;
static sm_display_mode display_mode = SM_DISPLAY_MODE_ENCODER;
static status_t previous_status = { 0 };

static const char sm_state_motors_off_enter_name[] PROGMEM = "motors off";  // Keep as 'motors off', so the prompt isn't confusing to the user.
static const char sm_state_motors_off_execute_name[] PROGMEM = "motors off";
static const char sm_state_motors_on_enter_name[] PROGMEM = "motors on enter";
static const char sm_state_motors_on_execute_name[] PROGMEM = "motors on execute";
static const char sm_state_error_name[] PROGMEM = "ERROR";

const sm_state_t sm_state_motors_off_enter = { .run = sm_motors_off_enter, .break_handler = NULL, .process_break_only = false, .name = sm_state_motors_off_enter_name };
const sm_state_t sm_state_motors_off_execute = { .run = sm_motors_off_execute, .break_handler = NULL, .process_break_only = false, .name = sm_state_motors_off_execute_name };
const sm_state_t sm_state_motors_on_enter = { .run = sm_motors_on_enter, .break_handler = NULL, .process_break_only = false, .name = sm_state_motors_on_enter_name };
const sm_state_t sm_state_motors_on_execute = { .run = sm_motors_on_execute, .break_handler = NULL, .process_break_only = false, .name = sm_state_motors_on_execute_name };
const sm_state_t sm_state_error_enter = { .run = sm_error_enter, .break_handler = NULL, .process_break_only = false, .name = sm_state_error_name };
const sm_state_t sm_state_error_execute = { .run = sm_error_execute, .break_handler = NULL, .process_break_only = false, .name = sm_state_error_name };

/*
 * Self-test functions.
 */

typedef struct {
    bool (*test_function)();
    char *pname;
} test_case_t;

static test_case_t test_case[] = {
    { config_test, "config" },
    { crc32c_test, "crc32c" },
    { log_test,    "log"    },
    { menu_test,   "menu"   },
    { parse_test,  "parse"  },
};
#define TEST_CASE_COUNT sizeof(test_case) / sizeof(test_case[0])

static bool run_self_test()
{
    int failure_count = 0;

    log_writeln(F("Running self test:"));
    for (int i = 0; i < TEST_CASE_COUNT; i++) {
        if (test_case[i].test_function()) {
            log_writeln(F("  %d. %s ... pass."), i, test_case[i].pname);
        } else {
            log_writeln(F("  %d. %s ... fail."), i, test_case[i].pname);
            failure_count++;
        }
    }

    log_writeln(F("%d test cases run, %d passed, %d failed."), TEST_CASE_COUNT, TEST_CASE_COUNT - failure_count, failure_count);
    log_writeln();

    return failure_count == 0;
}

void gather_status(status_t *status)
{
    assert(status);

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        status->motor[i].encoder = motor_get_encoder((motor_id_t)i);
        status->motor[i].switch_triggered = motor_is_home_triggered((motor_id_t)i);
    }
}

bool status_changed(status_t *status)
{
    return memcmp(&previous_status, status, sizeof(status_t)) != 0;
}

static void print_motor_status(status_t *status, sm_display_mode display_mode, motor_id_t motor_id)
{
    char str[15];
    char motor_name = (status->motor[motor_id].switch_triggered ? 'A' : 'a') + motor_id;

    if (display_mode == SM_DISPLAY_MODE_ENCODER) {
        log_write(F("%c:%d"), motor_name, motor_get_encoder(motor_id));
    } else if (display_mode == SM_DISPLAY_MODE_ANGLE) {
        dtostrf(motor_get_angle(motor_id), 3, 1, str);
        log_write(F("%c:%s"), motor_name, str);
    } else if (display_mode == SM_DISPLAY_MODE_PERCENT) {
        dtostrf(motor_get_percent(motor_id), 3, 2, str);
        log_write(F("%c:%s"), motor_name, str);
    }
}

void print_status(status_t *status)
{
    if (display_mode == SM_DISPLAY_MODE_ENCODER)
        log_write(F("enc "));
    else if (display_mode == SM_DISPLAY_MODE_ANGLE)
        log_write(F("deg "));
    else if (display_mode == SM_DISPLAY_MODE_PERCENT)
        log_write(F("pct "));

    for (int motor_id = 0; motor_id < MOTOR_ID_COUNT; motor_id++) {
        if (motor_get_enabled((motor_id_t)motor_id)) {
            print_motor_status(status, display_mode, motor_id);
            log_write(F(" "));
        } else if (motor_get_encoder(motor_id) != previous_status.motor[motor_id].encoder) {
            log_write(F("["));
            print_motor_status(status, display_mode, motor_id);
            log_write(F("] "));
        }
    }

    memcpy(&previous_status, status, sizeof(status_t));
}

static void process_break_only()
{
    const char ASCII_CTRL_C = 3;

    if (Serial.available()) {
        char input_char = Serial.read();

        if (input_char == ASCII_CTRL_C) {
            reset_prompt = true;
            log_writeln(F("<CTRL+C>"));
            if (current_state.break_handler)
                current_state.break_handler();
        } else {
            if (current_state.break_handler)
                log_writeln(F("Invalid input. Press <CTRL+C> to break."));
            else
                log_writeln(F("Invalid input. Please wait."));
        }
    }
}

static void maybe_print_command_prompt(menu_item_t const **prev_menu_item, int *command_args_nbytes, bool *have_command)
{
    static unsigned long previous_status_time_millis = 0;
    unsigned long current_time_millis = millis();

    status_t status;

    gather_status(&status);
    bool status_updated = status_changed(&status);

    if (reset_prompt ||
        (!*prev_menu_item &&
         status_updated &&
         (current_time_millis - previous_status_time_millis > 250))) {
        if (status_updated && !reset_prompt)
            log_writeln();

        *prev_menu_item = NULL;
        *command_args_nbytes = 0;
        *have_command = false;
        reset_prompt = false;

        if (strlen(config.robot_name) != 0)
            log_write(F("%s: "), config.robot_name);

        if (config_modified())
            log_write(F("*CONFIG MODIFIED* "));

        if (current_state.name != NULL) {
            // Don't print 'motors on', as it's redundant with the motor_name and encoder outputs in the for loop below.
            if ((current_state.run != sm_motors_on_enter) && (current_state.run != sm_motors_on_execute)) {
                log_write((const __FlashStringHelper *)current_state.name);
                log_write(F(" "));
            }
        } else {
            log_write(F("Unknown state "));
        }

        print_status(&status);

        log_write(F("> "));

        previous_status_time_millis = current_time_millis;
    }
}

static void process_all_input()
{
    const char ASCII_CTRL_C = 3;
    const char ASCII_BACKSPACE = 8;
    const char ASCII_BELL = 9;
    const char ASCII_RETURN = 13;
    const char ASCII_DELETE = 127;
    const int command_args_max_nbytes = 64;
    static char command_args[command_args_max_nbytes] = {};
    static int command_args_nbytes = 0;
    static menu_item_t const *prev_menu_item = NULL;
    static bool have_command = false;

    maybe_print_command_prompt(&prev_menu_item, &command_args_nbytes, &have_command);

    if (Serial.available()) {
        char input_char = Serial.read();

        if (input_char == ASCII_CTRL_C) {
            reset_prompt = true;
            log_writeln(F("<CTRL+C>"));
            if (current_state.break_handler)
                current_state.break_handler();
        } else if (!have_command) {
            if (input_char == ASCII_RETURN) {
                log_writeln();
                reset_prompt = true;
            } else if ((input_char == ASCII_BACKSPACE) || (input_char == ASCII_DELETE)) {
                log_write(F("\9"));  // Emit ASCII bell (flashes screen on some terminals)
            } else {
                const menu_item_t *menu_item = menu_item_by_command_char(input_char);
                prev_menu_item = menu_item;

                if (!menu_item) {
                    log_writeln(F("ERROR: Invalid command '%c'. Type '?' for help."), input_char);  // TODO: NAK?
                    reset_prompt = true;
                } else {
                    log_write((const __FlashStringHelper *)pgm_read_ptr(&menu_item->name));
                    void (*print_sub_menu_fn)() = pgm_read_ptr(&menu_item->print_sub_menu_fn);
                    if (print_sub_menu_fn)
                        print_sub_menu_fn();
                    if (pgm_read_ptr(&menu_item->has_args))
                        log_write(F(" "));  // Additional space for args.
                    have_command = true;
                }
            }
        } else {
            assert(prev_menu_item);
            if ((input_char == '\n') || (input_char == '\r')) {
                log_writeln();
                command_args[command_args_nbytes] = '\0';

                // Eat whitespace at start of arguments.
                size_t nbytes = parse_whitespace(command_args, command_args_nbytes);
                char *p = command_args + nbytes;
                command_args_nbytes -= nbytes;

                int (*function)(char *payload, size_t nbytes) = pgm_read_ptr(&prev_menu_item->function);  // Function to call after the command is typed.

                if (pgm_read_ptr(&prev_menu_item->sub_menu)) {
                    menu_set_current_menu(pgm_read_ptr(&prev_menu_item->sub_menu));
                    menu_help();
                } else if (function && (function(p, command_args_nbytes) != command_args_nbytes)) {
                    log_writeln(F("ERROR: Invalid command arguments. Type '?' for help."));  // TODO: NAK?
                }
                reset_prompt = true;
            } else if ((input_char == ASCII_BACKSPACE) || (input_char == ASCII_DELETE)) {
                // TODO: only 1 BS if we're entering args.
                int nbackspaces_count = strlen_P(prev_menu_item->name) + 2;  // +2 for " >"
                if (prev_menu_item->has_args)
                    nbackspaces_count++;  // Account for additional ' '.
                for (int i = 0; i < nbackspaces_count; i++) {
                    Serial.write(ASCII_BACKSPACE);
                    Serial.write(" ");
                    Serial.write(ASCII_BACKSPACE);
                }
                reset_prompt = true;
            } else if (pgm_read_byte(&prev_menu_item->has_args) && (command_args_nbytes < command_args_max_nbytes - 1)) {  // -1 to leave space for '\0' at end.
                log_write(F("%c"), input_char);
                command_args[command_args_nbytes++] = input_char;
            } else {
                log_writeln();
                log_writeln(F("ERROR: Too many characters in input buffer."));
                reset_prompt = true;
            }
        }
    }
}

// Transient state that performs initialization before transferring to the next state.
void sm_init(void)
{
    Serial.begin(38400);

    log_writeln(F("\n\rBooting Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories."));
    command_print_software_version("", 0);  // Pass empty args to mimic user typing version command.
    log_writeln();

    hardware_init();

    if (!config_read()) {
        log_writeln(F("ERROR: Invalid configuration. Re-initializing configuration data."));
        config_clear();
    } else {
        log_writeln(F("Read %d bytes of configuration data from Arduino EEPROM. Configuration is valid."), sizeof(config_t));
        log_write(F("Configured for '"));
        log_write((const __FlashStringHelper *)config_robot_name_by_id[config.robot_id]);
        log_writeln(F("'."));
    }

    config_init_gpio_pins();

    bool self_test_success = run_self_test();

    if (self_test_success) {
        config_print();
        log_writeln();
        motor_init();
    }

    menu_help();
    log_writeln(F("Ready."));

    gather_status(&previous_status);

    if (self_test_success)
        sm_set_next_state(sm_state_motors_off_enter);
    else
        sm_set_next_state(sm_state_error_enter);

    if (config.boot_mode == CONFIG_BOOT_MODE_EXECUTE_WAYPOINT_SEQUENCE)
        waypoint_run(0, -1);

    sm_execute();
}

sm_state_t sm_get_state()
{
    return current_state;
}

void sm_set_next_state(sm_state_t s)
{
    assert(s.run);
    assert(s.name);
    next_state = s;
}

void sm_execute(void)
{
    if (next_state.run) {
#ifdef DEBUG_STATE
        if (current_state.name) {
            log_write(F("Leaving state "));
            log_writeln((const __FlashStringHelper *)current_state.name);
        }
#endif
        current_state = next_state;

#ifdef DEBUG_STATE
        if (current_state.name) {
            log_write(F("Entering state "));
            log_writeln((const __FlashStringHelper *)current_state.name);
        }
#endif

        next_state = { 0 };
    }

    assert(current_state.run);
    current_state.run();

    if (current_state.process_break_only)
        process_break_only();
    else
        process_all_input();
}

void sm_motors_off_enter(void)
{
    motor_disable_all();

    for (int motor_id = 0; motor_id < MOTOR_ID_COUNT; motor_id++) {
        motor_set_enabled((motor_id_t)motor_id, false);
    }

    sm_set_next_state(sm_state_motors_off_execute);
}

void sm_motors_off_execute(void)
{
}

void sm_motors_on_enter(void)
{
    sm_set_next_state(sm_state_motors_on_execute);

    for (int motor_id = 0; motor_id < MOTOR_ID_COUNT; motor_id++) {
        bool enabled = enabled_motors_mask & (1 << motor_id);
        if (motor_get_enabled((motor_id_t)motor_id) != enabled)
            motor_set_enabled((motor_id_t)motor_id, enabled);
    }
}

void sm_motors_on_execute(void)
{
}

void sm_motors_on_exit(void)
{
    // motor_disable_all();
}

void sm_error_enter(void)
{
    next_state = { 0 };

    motor_disable_all();
    motor_set_error(MOTOR_ID_A, MOTOR_ERROR_OTHER);

    sm_set_next_state(sm_state_error_execute);
}

void sm_error_execute(void)
{
}

int sm_get_enabled_motors_mask(void)
{
    return enabled_motors_mask;
}

void sm_set_enabled_motors_mask(int mask)
{
    enabled_motors_mask = mask;
}

void sm_set_display_mode(sm_display_mode in_display_mode)
{
    display_mode = in_display_mode;
}
