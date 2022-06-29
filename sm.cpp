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

#define DEBUG_STATE

sm_state_t current_state;
sm_state_t next_state;

static int enabled_motors_mask = 0;
static bool reset_prompt = true;

static const char sm_state_motors_off_enter_name[] PROGMEM = "sm motors off enter";
static const char sm_state_motors_off_execute_name[] PROGMEM = "sm motors off execute";
static const char sm_state_motors_on_enter_name[] PROGMEM = "sm motors on enter";
static const char sm_state_motors_on_execute_name[] PROGMEM = "sm motors on execute";
static const char sm_state_error_name[] PROGMEM = "ERROR";

const sm_state_t sm_state_motors_off_enter = { .run = sm_motors_off_enter, .break_handler = NULL, .process_break_only = false, .name = sm_state_motors_off_enter_name, .data = NULL };
const sm_state_t sm_state_motors_off_execute = { .run = sm_motors_off_execute, .break_handler = NULL, .process_break_only = false, .name = sm_state_motors_off_execute_name, .data = NULL };
const sm_state_t sm_state_motors_on_enter = { .run = sm_motors_on_enter, .break_handler = NULL, .process_break_only = false, .name = sm_state_motors_on_enter_name, .data = NULL };
const sm_state_t sm_state_motors_on_execute = { .run = sm_motors_on_execute, .break_handler = NULL, .process_break_only = false, .name = sm_state_motors_on_execute_name, .data = NULL };
const sm_state_t sm_state_error_enter = { .run = sm_error_enter, .break_handler = NULL, .process_break_only = false, .name = sm_state_error_name, .data = NULL };
const sm_state_t sm_state_error_execute = { .run = sm_error_execute, .break_handler = NULL, .process_break_only = false, .name = sm_state_error_name, .data = NULL };

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

// Used for status messages.
typedef struct {
    float angle;
    bool  switch_triggered;
    bool  thermal_overload_detected;
} motor_status_t;

typedef struct {
    motor_status_t motor[MOTOR_ID_COUNT];
} status_t;

static void gather_status(status_t *pstatus)
{
    assert(pstatus);

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        pstatus->motor[i].angle = motor_get_angle((motor_id_t)i);
        pstatus->motor[i].switch_triggered = motor_is_home_triggered((motor_id_t)i);
        pstatus->motor[i].thermal_overload_detected = motor_get_thermal_overload_detected((motor_id_t)i);
        motor_clear_thermal_overload((motor_id_t)i);
    }
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
                current_state.break_handler(&current_state);
        } else {
            if (current_state.break_handler)
                log_writeln(F("Invalid input. Press <CTRL+C> to break."));
            else
                log_writeln(F("Invalid input. Please wait."));
        }
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

    bool status_updated = false;
    static status_t previous_status = { 0 };
    status_t status;

    gather_status(&status);

    if (memcmp(&previous_status, &status, sizeof(status_t)) != 0)
        status_updated = true;

    static unsigned long previous_status_time_millis = 0;
    unsigned long current_time_millis = millis();

    if (reset_prompt ||
        (!prev_menu_item &&
         status_updated &&
         (current_time_millis - previous_status_time_millis > 250))) {
        if (status_updated && !reset_prompt)
            log_writeln(F(""));

        prev_menu_item = NULL;
        command_args_nbytes = 0;
        have_command = false;
        reset_prompt = false;

        if (strlen(config.robot_name) != 0)
            log_write(F("%s "), config.robot_name);

        for (int i = 0; i < MOTOR_ID_COUNT; i++) {
            if (motor_get_enabled((motor_id_t)i)) {
                char angle_str[15];
                dtostrf(status.motor[i].angle, 3, 2, angle_str);
                char motor_name = (status.motor[i].switch_triggered ? 'A' : 'a') + i;

                log_write(F("%c:%s,%d,%d "), motor_name, angle_str, motor_get_current((motor_id_t)i), motor_state[i].pid_perror);
            }
        }

#ifdef DEBUG_STATE
        if (current_state.name != NULL)
            log_write((const __FlashStringHelper *)current_state.name);
        else
            log_write(F("Unknown state"));
#endif
        log_write(F("> "));

        memcpy(&previous_status, &status, sizeof(status_t));
        previous_status_time_millis = current_time_millis;
    }

    if (Serial.available()) {
        char input_char = Serial.read();

        if (input_char == ASCII_CTRL_C) {
            reset_prompt = true;
            log_writeln(F("<CTRL+C>"));
            if (current_state.break_handler)
                current_state.break_handler(&current_state);
        } else if (!have_command) {
            if (input_char == ASCII_RETURN) {
                log_writeln(F(""));
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
                    log_write((const __FlashStringHelper *)menu_item->name);
                    if (menu_item->print_sub_menu_fn)
                        menu_item->print_sub_menu_fn();
                    if (menu_item->has_args)
                        log_write(F(" "));  // Additional space for args.

                    have_command = true;
                }
            }
        } else {
            assert(prev_menu_item);
            if ((input_char == '\n') || (input_char == '\r')) {
                log_writeln(F(""));
                command_args[command_args_nbytes] = '\0';

                // Eat whitespace at start of arguments.
                size_t nbytes = parse_whitespace(command_args, command_args_nbytes);
                char *p = command_args + nbytes;
                command_args_nbytes -= nbytes;

                if (prev_menu_item->sub_menu) {
                    menu_set_current_menu(prev_menu_item->sub_menu);
                    menu_help();
                } else if (prev_menu_item->function && (prev_menu_item->function(p, command_args_nbytes) != command_args_nbytes)) {
                    log_writeln(F("ERROR: Invalid command arguments. Type '?' for help."));  // TODO: NAK?
                }
                reset_prompt = true;
            } else if ((input_char == ASCII_BACKSPACE) || (input_char == ASCII_DELETE)) {
                // TODO: only 1 BS if we're entering args.
                int nbackspaces_count = strlen(prev_menu_item->name) + 2;  // +2 for " >"
                if (prev_menu_item->has_args)
                    nbackspaces_count++;  // Account for additional ' '.
                for (int i = 0; i < nbackspaces_count; i++) {
                    Serial.write(ASCII_BACKSPACE);
                    Serial.write(" ");
                    Serial.write(ASCII_BACKSPACE);
                }
                reset_prompt = true;
            } else if (prev_menu_item->has_args && (command_args_nbytes < command_args_max_nbytes - 1)) {  // -1 to leave space for '\0' at end.
                log_write(F("%c"), input_char);
                command_args[command_args_nbytes++] = input_char;
            } else {
                log_writeln(F(""));
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
        log_writeln(F("Configured for '%s'."), config_robot_name_by_id[config.robot_id]);
    }

    bool self_test_success = run_self_test();

    if (self_test_success) {
        config_print();
        log_writeln();
        motor_init_all();
    }

    menu_help();
    log_writeln(F("Ready."));

    if (self_test_success)
        sm_set_next_state(sm_state_motors_off_enter);
    else
        sm_set_next_state(sm_state_error_enter);
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
    current_state.run(&current_state);

    if (current_state.process_break_only)
        process_break_only();
    else
        process_all_input();
}

void sm_motors_off_enter(sm_state_t *state)
{
    assert(state);
    motor_disable_all();

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        motor_set_enabled((motor_id_t)i, false);
    }

    sm_set_next_state(sm_state_motors_off_execute);
}

void sm_motors_off_execute(sm_state_t *state)
{
    assert(state);
}

void sm_motors_on_enter(sm_state_t *state)
{
    assert(state);

    sm_set_next_state(sm_state_motors_on_execute);

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        bool enabled = enabled_motors_mask & (1 << i);
        if (motor_get_enabled((motor_id_t)i) != enabled)
            motor_set_enabled((motor_id_t)i, enabled);
    }
}

void sm_motors_on_execute(sm_state_t *state)
{
    assert(state);
}

void sm_motors_on_exit(sm_state_t *state)
{
    assert(state);
    // motor_disable_all();
}

void sm_error_enter(sm_state_t *state)
{
    assert(state);
    next_state = { 0 };

    motor_disable_all();
    motor_set_user_error(true);

    sm_set_next_state(sm_state_error_execute);
}

void sm_error_execute(sm_state_t *state)
{
    assert(state);
}

int sm_get_enabled_motors_mask(void)
{
    return enabled_motors_mask;
}

void sm_set_enabled_motors_mask(int mask)
{
    enabled_motors_mask = mask;
}
