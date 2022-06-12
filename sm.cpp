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

sm_state_t current_state;
sm_state_t next_state;

static int enabled_motors_mask = 0;

/*
 * Self-test functions.
 */

typedef struct {
    bool (*test_function)();
    char *pname;
} test_case_t;

static test_case_t test_case[] = {
    { config_test, "config"              },
    { crc32c_test, "crc32c"              },
    { log_test,    "log"                 },
    { menu_test,   "menu"                },
    { parse_test,  "parse"               },
    { sm_test,     "sm (state smachine)" },
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
    bool  overcurrent_detected;
} motor_status_t;

typedef struct {
    motor_status_t motor[MOTOR_ID_COUNT];
} status_t;

static void gather_status(status_t *pstatus)
{
    assert(pstatus);

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        if (config.motor[i].configured) {
            pstatus->motor[i].angle = motor_get_angle(i);
            pstatus->motor[i].switch_triggered = motor_get_home_triggered(i);
            pstatus->motor[i].thermal_overload_detected = motor_get_thermal_overload_detected(i);
            pstatus->motor[i].overcurrent_detected = motor_get_overcurrent_detected(i);
            motor_clear_thermal_overload(i);
            motor_clear_overcurrent(i);
        } else {
            memset(&pstatus->motor[i], 0, sizeof(motor_status_t));
        }
    }
}

static void process_serial_input()
{
    const char ASCII_CTRL_C = 3;
    const char ASCII_BACKSPACE = 8;
    const char ASCII_BELL = 9;
    const char ASCII_RETURN = 13;
    const char ASCII_DELETE = 127;
    const int command_args_max_nbytes = 64;
    static char command_args[command_args_max_nbytes] = {};
    static int command_args_nbytes = 0;
    static menu_item_t *pprev_menu_item = NULL;
    static bool have_command = false;
    static bool reset_prompt = true;

    bool status_updated = false;
    static status_t previous_status = { 0 };
    status_t status;

    gather_status(&status);

    if (memcmp(&previous_status, &status, sizeof(status_t)) != 0)
        status_updated = true;

    static unsigned long previous_status_time_millis = 0;
    unsigned long current_time_millis = millis();

    if (reset_prompt ||
        (!pprev_menu_item &&
         status_updated &&
         (current_time_millis - previous_status_time_millis > 250))) {
        if (status_updated && !reset_prompt)
            log_writeln(F(""));

        // motor_dump_motor(MOTOR_ID_E);
        // motor_dump_motor(MOTOR_ID_F);

        pprev_menu_item = NULL;
        command_args_nbytes = 0;
        have_command = false;
        reset_prompt = false;

        for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
            if (motor_get_enabled(i)) {
                char angle_str[15];
                dtostrf(status.motor[i].angle, 3, 2, angle_str);
                char motor_name = (status.motor[i].switch_triggered ? 'A' : 'a') + i;

                log_write(F("%c:%s,%d,%d "), motor_name, angle_str, motor_get_current_draw((motor_id_t)i), motor_state[i].pid_perror);
            }
        }

        if (strlen(config.robot_name) != 0)
            log_write(F("%s "), config.robot_name);

        if (current_state.name != NULL)
            log_write(current_state.name);
        else
            log_write(F("Unknown state"));
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
                const menu_item_t *pmenu_item = menu_item_by_command_char(input_char);
                pprev_menu_item = pmenu_item;

                if (!pmenu_item) {
                    log_writeln(F("ERROR: Invalid command '%c'. Type '?' for help."), input_char);  // TODO: NAK?
                    reset_prompt = true;
                } else {
                    log_write((const __FlashStringHelper *)pmenu_item->name);
                    if (pmenu_item->print_sub_menu_fn)
                        pmenu_item->print_sub_menu_fn();
                    if (pmenu_item->has_args)
                        log_write(F(" "));  // Additional space for args.

                    have_command = true;
                }
            }
        } else {
            assert(pprev_menu_item);
            if ((input_char == '\n') || (input_char == '\r')) {
                log_writeln(F(""));
                command_args[command_args_nbytes] = '\0';

                // Eat whitespace at start of arguments.
                size_t nbytes = parse_whitespace(command_args, command_args_nbytes);
                char *p = command_args + nbytes;
                command_args_nbytes -= nbytes;

                if (pprev_menu_item->sub_menu) {
                    menu_set_current_menu(pprev_menu_item->sub_menu);
                    menu_help();
                } else if (pprev_menu_item->function && (pprev_menu_item->function(p, command_args_nbytes) != command_args_nbytes)) {
                    log_writeln(F("ERROR: Invalid command arguments. Type '?' for help."));  // TODO: NAK?
                }
                reset_prompt = true;
            } else if ((input_char == ASCII_BACKSPACE) || (input_char == ASCII_DELETE)) {
                // TODO: only 1 BS if we're entering args.
                int nbackspaces_count = strlen(pprev_menu_item->name) + 2;  // +2 for " >"
                if (pprev_menu_item->has_args)
                    nbackspaces_count++;  // Account for additional ' '.
                for (int i = 0; i < nbackspaces_count; i++) {
                    Serial.write(ASCII_BACKSPACE);
                    Serial.write(" ");
                    Serial.write(ASCII_BACKSPACE);
                }
                reset_prompt = true;
            } else if (command_args_nbytes < command_args_max_nbytes - 1) {  // -1 to leave space for '\0' at end.
                log_write(F("%c"), input_char);
                command_args[command_args_nbytes++] = input_char;
            } else {
                log_writeln(F(""));
                LOG_ERROR(F("Too many characters in input buffer."));
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

    bool config_read_success = config_read();

    if (!config_read_success) {
        log_writeln(F("ERROR: Invalid configuration. Re-initializing configuration data."));
        config_clear();
    } else {
        log_writeln(F("Read %d bytes of configuration data from Arduino EEPROM. Configuration is valid."), sizeof(config_t));
        log_writeln(F("Configured for '%s'."), config_robot_name_by_id[config.robot_id]);
    }

    hardware_init();
    bool self_test_success = run_self_test();

    if (self_test_success) {
        config_print();
        log_writeln();
        motor_init_all();
    }

    menu_help();
    log_writeln(F("Ready."));

    if (config_read_success && self_test_success) {
        sm_state_t s = { .run = sm_motors_off_enter, .break_handler = NULL, .name = F("sm_motors_off_enter"), .data = NULL };
        sm_set_next_state(s);
    } else {
        sm_state_t s = { .run = sm_error_enter, .break_handler = NULL, .name = F("sm_error_enter"), .data = NULL };
        sm_set_next_state(s);
    }
}

sm_state_t sm_get_state()
{
    return current_state;
}

void sm_set_next_state(sm_state_t s)
{
    assert(s.run);
    next_state = s;
}

void sm_execute(void)
{
    process_serial_input();

    if (next_state.run) {
        if (current_state.name) {
            log_write(F("Leaving state "));
            log_writeln(current_state.name);
        }
        current_state = next_state;

        if (current_state.name) {
            log_write(F("Entering state "));
            log_writeln(current_state.name);
        }

        next_state = { 0 };
    }

    assert(current_state.run);
    current_state.run(&current_state);
}

void sm_motors_off_enter(void)
{
    motor_disable_all();

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        motor_set_enabled(i, false);
    }

    sm_state_t s = { .run = sm_motors_off_execute, .break_handler = NULL, .name = F("motors off"), .data = NULL };

    sm_set_next_state(s);
}

void sm_motors_off_execute(void)
{
    process_serial_input();
}

void sm_motors_on_enter(void)
{
    sm_state_t s = { .run = sm_motors_on_execute, .break_handler = NULL, .name = F("motors on"), .data = NULL };

    sm_set_next_state(s);

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        bool enabled = enabled_motors_mask & (1 << i);
        if (motor_get_enabled(i) != enabled)
            motor_set_enabled(i, enabled);
    }
}

void sm_motors_on_execute(void)
{
    process_serial_input();
}

void sm_motors_on_exit(void)
{
    // motor_disable_all();
}

void sm_error_enter(void)
{
    next_state = { 0 };

    motor_disable_all();
    sm_state_t s = { .run = sm_error_execute, .break_handler = NULL, .name = F("ERROR"), .data = NULL };

    sm_set_next_state(s);
}

void sm_error_execute(void)
{
    process_serial_input();
}

int sm_get_enabled_motors_mask(void)
{
    return enabled_motors_mask;
}

void sm_set_enabled_motors_mask(int mask)
{
    enabled_motors_mask = mask;
}

bool sm_test()
{
    // TODO: Implement.

    // Confirm can't escape error state.
    return true;
}
