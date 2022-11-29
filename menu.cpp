/*
 * Implementation for menu functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include "command.h"
#include "config.h"
#include "log.h"
#include "menu.h"

static const int menu_item_max_count = 40;
static const int menu_item_max_name_nbytes = 40;

static void extended_menu_config_robot_id(void)
{
    log_writeln();
    log_write(F("Current robot ID: "));
    log_write((const __FlashStringHelper *)config_robot_name_by_id[config.robot_id]);
    log_writeln(F("."));
    log_writeln(F("Available robot IDs:"));
    for (int i = 0; i < CONFIG_ROBOT_ID_COUNT; i++) {
        log_write(F("  %d. "), i);
        log_write((const __FlashStringHelper *)config_robot_name_by_id[i]);
        log_writeln(F("."));
    }
    log_writeln(F("Select new robot ID, or press [RETURN] to keep current robot ID."));
    log_write(F(">"));
}

static void extended_menu_config_robot_serial(void)
{
    log_writeln();
    log_writeln(F("Current robot serial is '%s'."), config.robot_serial);
    log_writeln(F("Enter new robot serial, or press [RETURN] to keep current serial."));
    log_write(F(">"));
}

static void extended_menu_config_robot_name(void)
{
    log_writeln();
    log_writeln(F("Current robot name is '%s'."), config.robot_name);
    log_writeln(F("Enter new robot name shorter than %d characters, or press [RETURN] to keep current name."), CONFIG_ROBOT_NAME_NBYTES);
    log_write(F(">"));
}

static void extended_menu_config_min_max_encoders(void)
{
    log_writeln();
    log_writeln(F("Current min max encoder limits:"));
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        log_writeln(F("  %c: min %d, max %d"), 'A' + i, config.motor[i].min_encoder, config.motor[i].max_encoder);
    }
    log_writeln(F("Enter 'motorid min max' to configure or <CTRL+C> to exit."));
    log_write(F(">"));
}

static void extended_menu_config_home_encoder(void)
{
    log_writeln();
    log_writeln(F("Current min max encoder limits:"));
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        log_writeln(F("  %c: min %d, max %d"), 'A' + i, config.motor[i].min_encoder, config.motor[i].max_encoder);
    }
    log_writeln(F("Enter 'motorid encoder' to set encoder to the home/zero position (this adjusts the min, max limits). Or <CTRL+C> to exit."));
    log_write(F(">"));
}

static void extended_menu_config_encoders_per_degree(void)
{
    // TODO: The values below are incomplete and wrong. Fix this.
    log_writeln();
    log_writeln(F("Example encoders/degree:"));
    log_writeln(F("  XR-1, XR-2, XR-4 motor A: 1 encoders/degree."));
    log_writeln(F("  XR-1, XR-4 motor B: (5.51)(165.4/1) = 12.5 encoders/degree."));
    log_writeln(F("  XR-1, XR-2 motor E: (5.51)(165.4/1) = 36 encoders/degree."));
    log_writeln(F("  XR-4, motor E: (5.51)(165.4/1) = 35 encoders/degree."));
    log_writeln(F("  XR-1, XR2 motor F: (4.4)(66.1/1) = 29.5 encoders/degree."));
    log_writeln(F("  XR-4, motor F: (4.4)(66.1/1) = 17.5 encoders/degree."));

    log_writeln(F("Current encoders/degree:"));
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        log_writeln(F("  %c: %d"), 'A' + i, config.motor[i].encoders_per_degree);
    }
    log_writeln(F("Enter 'motorid encoders' to set the number of encoders per degree. Or <CTRL+C> to exit."));
    log_write(F(">"));
}

static void extended_menu_config_angle_offset(void)
{
    log_writeln();
    log_writeln(F("Current angle offsets:"));
    char str[15] = {};

    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
        dtostrf(config.motor[i].angle_offset, 3, 2, str);
        log_writeln(F("  %c: %s"), 'A' + i, str);
    }
    log_writeln(F("Enter 'motorid offset' to set the angle-offset from the home (0) encoder. Or <CTRL+C> to exit."));
    log_write(F(">"));
}

static void extended_menu_factory_reset(void)
{
    log_writeln();
    log_writeln(F("Type 'RESET' in all capital letters to clear the EEPROM and reboot the system or <CTRL+C> to exit."));
    log_write(F(">"));
}

static void extended_menu_reboot(void)
{
    log_writeln();
    log_writeln(F("Type 'REBOOT' in all capital letters to reboot the system or <CTRL+C> to exit."));
    log_write(F(">"));
}

static void extended_menu_append_waypoint(void)
{
    log_writeln();
    log_writeln(F("A <ENTER> -- Append waypoint to move motors to current positions."));
    log_writeln(F("B <ENTER> -- Append waypoint to move motors to within 1 encoder value of current positions."));
    log_writeln(F("C <ENTER> -- Append waypoint to move motors to within 30 encoder values of current positions."));
    log_writeln(F("D <ENTER> -- Append waypoint to move motors to within 200 encoder values current positions."));
    log_writeln(F("E [motorids] -- Append waypoint to enable motors. Disables all motors if none specified."));
    log_writeln(F("G step <ENTER> -- Append waypoint to goto step."));
    log_writeln(F("J pin step <ENTER> -- Append waypoint so if IO pin triggered, goto step."));
    log_writeln(F("K pin <ENTER> -- Append waypoint to wait for IO pin triggered."));
    log_writeln(F("L [motorids] <ENTER> -- Append waypoint to calibrate home switches and limits of motorids or enabled motors."));
    log_writeln(F("O [motorids] <ENTER> --  Append waypoint to calibrate home switches of motorids or enabled motors."));
    log_writeln(F("W milliseconds <ENTER> -- Append waypoint to wait milliseconds."));
    log_write(F(">"));
}

static void extended_menu_insert_waypoint(void)
{
    log_writeln();
    log_writeln(F("step A <ENTER> -- Insert waypoint before step to move motors to current positions."));
    log_writeln(F("step B <ENTER> -- Insert waypoint before step to move motors to within 1 encoder value of current positions."));
    log_writeln(F("step C <ENTER> -- Insert waypoint before step to move motors to within 30 encoder values of current positions."));
    log_writeln(F("step D <ENTER> -- Insert waypoint before step to move motors to within 200 encoder values current positions."));
    log_writeln(F("step E [motorids] -- Insert waypoint before step to enable motors. Disables all motors if none specified."));
    log_writeln(F("step G step <ENTER> -- Insert waypoint before step to goto step."));
    log_writeln(F("step J pin step <ENTER> -- Insert waypoint before step so if IO pin triggered, goto step."));
    log_writeln(F("step K pin <ENTER> -- Insert waypoint before step to wait for IO pin triggered."));
    log_writeln(F("step L [motorids] <ENTER> -- Insert waypoint before step to calibrate home switches and limits of motorids or enabled motors."));
    log_writeln(F("step O [motorids] <ENTER> -- Insert waypoint before step to calibrate home switches of motorids or enabled motors."));
    log_writeln(F("step W milliseconds <ENTER> -- Insert waypoint before step to wait milliseconds."));
    log_write(F(">"));
}

static void extended_menu_set_waypoint(void)
{
    log_writeln();
    log_writeln(F("step A <ENTER> -- Set waypoint step to move motors to exactly current positions."));
    log_writeln(F("step B <ENTER> -- Set waypoint step to move motors to within 1 encoder value of current positions."));
    log_writeln(F("step C <ENTER> -- Set waypoint step to move motors to within 30 encoder values of current positions."));
    log_writeln(F("step D <ENTER> -- Set waypoint step to move motors to within 200 encoder values current positions."));
    log_writeln(F("step E [motorids] -- Enable motors. Disables all motors if none specified."));
    log_writeln(F("step G step <ENTER> -- Set waypoint step to goto step."));
    log_writeln(F("step J pin step <ENTER> -- Set waypoint step so if IO pin triggered, goto step."));
    log_writeln(F("step K pin <ENTER> -- Set waypoint step to wait for IO pin triggered."));
    log_writeln(F("step L [motorids] <ENTER> -- Set waypoint step to calibrate home switches and limits of motorids or enabled motors."));
    log_writeln(F("step O [motorids] <ENTER> -- Set waypoint step to calibrate home switches of motorids or enabled motors."));
    log_writeln(F("step W milliseconds <ENTER> -- Set waypoint step to wait milliseconds."));
    log_write(F(">"));
}

// These strings are defined outside of the menu below, so they can be stored in flash with PROGMEM.
static const char MM_1[] PROGMEM = "print configuration";
static const char MH_1[] PROGMEM = "-- Print configuration.";
static const char MM_B[] PROGMEM = "reboot";
static const char MH_B[] PROGMEM = "-- Reboot system. Requires typing 'REBOOT'.";
static const char MM_C[] PROGMEM = "calibration menu";
static const char MH_C[] PROGMEM = "-- Print, run, and save home switch and motor calibration.";
static const char MM_E[] PROGMEM = "set enabled motors";
static const char MH_E[] PROGMEM = "[motorids] -- Enable/disable motors, list (e.g 'abce') or blank to disable all.";
static const char MM_H[] PROGMEM = "go home or open gripper";
static const char MH_H[] PROGMEM = "[motorids] -- Command motors to home position or gripper open (i.e. 0). Commands enabled motors if none specified.";
static const char MM_M[] PROGMEM = "print motor status";
static const char MH_M[] PROGMEM = "[motorids] -- Print motor status, list (e.g. 'abce') or blank for all.";
static const char MM_N[] PROGMEM = "set motor angle";
static const char MH_N[] PROGMEM = "motorid degrees -- Degrees is 0.0 to 360.0, +15, -20, +, -, ++, --.";
static const char MM_O[] PROGMEM = "configuration menu";
static const char MH_O[] PROGMEM = "-- Configure system and attached motors.";
static const char MM_P[] PROGMEM = "set motor encoder";
static const char MH_P[] PROGMEM = "motorid encoder -- Set motor encoder.";
static const char MM_R[] PROGMEM = "print software version";
static const char MH_R[] PROGMEM = "-- Print software version.";
static const char MM_T[] PROGMEM = "test motors";
static const char MH_T[] PROGMEM = "[motorids] -- Test motors. Test enabled motors if none specified.";
static const char MM_V[] PROGMEM = "close gripper";
static const char MH_V[] PROGMEM = "-- Close gripper [motorids].";
static const char MM_W[] PROGMEM = "waypoints menu";
static const char MH_W[] PROGMEM = "-- Edit and execute waypoints.";
static const char MM_Z[] PROGMEM = "poll header pins";
static const char MH_Z[] PROGMEM = "-- Poll header pins for changes in polarity. Useful for debugging buttons.";
static const char MM_PCT[] PROGMEM = "set motor percent";
static const char MH_PCT[] PROGMEM = "motorid percent -- Percent is 0.0 to 100.0, +15, -20, +, -, ++, --.";
static const char MM_STAR[] PROGMEM = "factory reset";
static const char MH_STAR[] PROGMEM = "-- Reset system to factory defaults (clears EEPROM, etc). Requires typing 'RESET'.";
static const char MM_BANG[] PROGMEM = "emergency stop";
static const char MH_BANG[] PROGMEM = "-- Execute hardware emergency stop (E-Stop). Enters 'Error' state. Requires reboot.";
static const char MM_HELP[] PROGMEM = "print help";
static const char MH_HELP[] PROGMEM = "-- Print this help message.";

extern const menu_item_t config_menu[];
extern const menu_item_t waypoint_menu[];
extern const menu_item_t calibration_menu[];

static const menu_item_t main_menu[] = {
    { '1', MM_1,    NULL,                        NULL,             false, command_print_config,            MH_1    },
    { 'B', MM_B,    extended_menu_reboot,        NULL,             true,  command_reboot,                  MH_B    },
    { 'C', MM_C,    NULL,                        calibration_menu, false, NULL,                            MH_C    },
    { 'E', MM_E,    NULL,                        NULL,             true,  command_set_enabled_motors,      MH_E    },
    { 'H', MM_H,    NULL,                        NULL,             true,  command_go_home_or_open_gripper, MH_H    },
    { 'M', MM_M,    NULL,                        NULL,             true,  command_print_motor_status,      MH_M    },
    { 'N', MM_N,    NULL,                        NULL,             true,  command_set_motor_angle,         MH_N    },
    { 'O', MM_O,    NULL,                        config_menu,      false, NULL,                            MH_O    },
    { 'P', MM_P,    NULL,                        NULL,             true,  command_set_motor_encoder,       MH_P    },
    // { 'Q', MM_Q,    NULL,                              NULL,             true,  command_run_test_sequence,         MH_Q    },  // TODO.
    { 'R', MM_R,    NULL,                        NULL,             false, command_print_software_version,  MH_R    },
    { 'T', MM_T,    NULL,                        NULL,             true,  command_test_motors,             MH_T    },
    { 'V', MM_V,    NULL,                        NULL,             true,  command_close_gripper,           MH_V    },
    { 'W', MM_W,    NULL,                        waypoint_menu,    false, NULL,                            MH_W    },
    { 'Z', MM_Z,    NULL,                        NULL,             false, command_poll_pins,               MH_Z    },
    { '%', MM_PCT,  NULL,                        NULL,             true,  command_set_motor_percent,       MH_PCT  },
    { '*', MM_STAR, extended_menu_factory_reset, NULL,             true,  command_factory_reset,           MH_STAR },
    { '!', MM_BANG, NULL,                        NULL,             false, command_emergency_stop,          MH_BANG },
    { '?', MM_HELP, NULL,                        NULL,             false, command_print_help,              MH_HELP },
    { 0 }  // Terminate menus with an entry filled with zeros.
};

static const char OM_2[] PROGMEM = "configure robot ID";
static const char OH_2[] PROGMEM = "Print or set configured robot ID.";
static const char OM_3[] PROGMEM = "configure robot serial";
static const char OH_3[] PROGMEM = "Print or set configured robot serial.";
static const char OM_4[] PROGMEM = "configure robot name";
static const char OH_4[] PROGMEM = "Print or set robot name.";
static const char OM_5[] PROGMEM = "reverse motor orientation";
static const char OH_5[] PROGMEM = "motorid -- Reverse orientation of a motor.";
static const char OM_6[] PROGMEM = "set motor min max encoder limits";
static const char OH_6[] PROGMEM = "motorid min max -- Print or set configured min max encoder limits.";
static const char OM_7[] PROGMEM = "set motor home encoder";
static const char OH_7[] PROGMEM = "motorid encoder -- Print or set configured home encoder to zero (adjusts min, max).";
static const char OM_8[] PROGMEM = "set motor encoders/degree";
static const char OH_8[] PROGMEM = "motorid encoders/degree -- Print or set configured encoders/degree).";
static const char OM_9[] PROGMEM = "set motor angle offset";
static const char OH_9[] PROGMEM = "motorid angle -- Print or set configured angle offset from encdoer 0.";
static const char OM_0[] PROGMEM = "write configuration";
static const char OH_0[] PROGMEM = "-- Write configuration data to EEPROM.";
static const char OM_X[] PROGMEM = "exit to main menu";
static const char OH_X[] PROGMEM = "-- Exit to main menu.";

const menu_item_t config_menu[] = {
    { '1', MM_1,    NULL,                                     NULL,      false, command_print_config,               MH_1    },
    { '2', OM_2,    extended_menu_config_robot_id,            NULL,      true,  command_config_robot_id,            OH_2    },
    { '3', OM_3,    extended_menu_config_robot_serial,        NULL,      true,  command_config_robot_serial,        OH_3    },
    { '4', OM_4,    extended_menu_config_robot_name,          NULL,      true,  command_config_robot_name,          OH_4    },
    { '5', OM_5,    NULL,                                     NULL,      true,  command_reverse_motor_orientation,  OH_5    },
    { '6', OM_6,    extended_menu_config_min_max_encoders,    NULL,      true,  command_config_min_max_encoders,    OH_6    },
    { '7', OM_7,    extended_menu_config_home_encoder,        NULL,      true,  command_config_home_encoder,        OH_7    },
    { '8', OM_8,    extended_menu_config_encoders_per_degree, NULL,      true,  command_config_encoders_per_degree, OH_8    },
    { '9', OM_9,    extended_menu_config_angle_offset,        NULL,      true,  command_config_angle_offset,        OH_9    },
    { '0', OM_0,    NULL,                                     NULL,      false, command_config_write,               OH_0    },
    { 'X', OM_X,    NULL,                                     main_menu, false, NULL,                               OH_X    },
    { '!', MM_BANG, NULL,                                     NULL,      false, command_emergency_stop,             MH_BANG },
    { '?', MM_HELP, NULL,                                     NULL,      false, command_print_help,                 MH_HELP },
    { 0 }  // Terminate menus with an entry filled with zeros.
};

static const char CM_C[] PROGMEM = "calibrate home switches and limits";
static const char CH_C[] PROGMEM = "[motorids [max-speed-percent]] -- Calibrate home switches and motor limits; calibrates enabled motors if none given.";
static const char CM_W[] PROGMEM = "calibrate home switches";
static const char CH_W[] PROGMEM = "[motorids [max-speed-percent]] -- Calibrate home switches; calibrates enabled motors if none given.";

const menu_item_t calibration_menu[] = {
    { '1', MM_1,    NULL,                 NULL,      false, command_print_config,              MH_1    },
    { '0', OH_0,    NULL,                 NULL,      false, command_config_write,              OH_0    },
    { 'B', MM_B,    extended_menu_reboot, NULL,      true,  command_reboot,                    MH_B    },
    { 'C', CM_C,    NULL,                 NULL,      true,  command_calibrate_home_and_limits, CH_C    },
    { 'E', MM_E,    NULL,                 NULL,      true,  command_set_enabled_motors,        MH_E    },
    { 'H', MM_H,    NULL,                 NULL,      true,  command_go_home_or_open_gripper,   MH_H    },
    { 'M', MM_M,    NULL,                 NULL,      true,  command_print_motor_status,        MH_M    },
    { 'N', MM_N,    NULL,                 NULL,      true,  command_set_motor_angle,           MH_N    },
    { 'P', MM_P,    NULL,                 NULL,      true,  command_set_motor_encoder,         MH_P    },
    { 'T', MM_T,    NULL,                 NULL,      true,  command_test_motors,               MH_T    },
    { 'V', MM_V,    NULL,                 NULL,      true,  command_close_gripper,             MH_V    },
    { 'W', CM_W,    NULL,                 NULL,      true,  command_calibrate_home,            CH_W    },
    { 'X', OM_X,    NULL,                 main_menu, false, NULL,                              OH_X    },
    { '%', MM_PCT,  NULL,                 NULL,      true,  command_set_motor_percent,         MH_PCT  },
    { '!', MM_BANG, NULL,                 NULL,      false, command_emergency_stop,            MH_BANG },
    { '?', MM_HELP, NULL,                 NULL,      false, command_print_help,                MH_HELP },
    { 0 }  // Terminate menus with an entry filled with zeros.
};

static const char WM_1[] PROGMEM = "print waypoints";
static const char WH_1[] PROGMEM = "-- Print waypoints.";
static const char WM_A[] PROGMEM = "append waypoint";
static const char WH_A[] PROGMEM = "-- Append waypoint to end of list.";
static const char WM_D[] PROGMEM = "delete waypoint";
static const char WH_D[] PROGMEM = "step -- Delete waypoint step.";
static const char WM_I[] PROGMEM = "insert waypoint";
static const char WH_I[] PROGMEM = "-- Insert waypoint.";
static const char WM_R[] PROGMEM = "run waypoint sequence";
static const char WH_R[] PROGMEM = "-- Run waypoint sequence.";
static const char WM_S[] PROGMEM = "set waypoint";
static const char WH_S[] PROGMEM = "-- Set waypoint.";

const menu_item_t waypoint_menu[] = {
    { '1', WM_1,    NULL,                          NULL,      false, command_waypoint_print,            WH_1    },
    { 'A', WM_A,    extended_menu_append_waypoint, NULL,      true,  command_waypoint_append,           WH_A    },
    { 'C', CM_C,    NULL,                          NULL,      true,  command_calibrate_home_and_limits, CH_C    },
    { 'D', WM_D,    NULL,                          NULL,      true,  command_waypoint_delete,           WH_D    },
    { 'E', MM_E,    NULL,                          NULL,      true,  command_set_enabled_motors,        MH_E    },
    { 'H', MM_H,    NULL,                          NULL,      false, command_go_home_or_open_gripper,   MH_H    },
    { 'I', WM_I,    extended_menu_insert_waypoint, NULL,      true,  command_waypoint_insert_before,    WH_I    },
    { 'N', MM_N,    NULL,                          NULL,      true,  command_set_motor_angle,           MH_N    },
    { 'P', MM_P,    NULL,                          NULL,      true,  command_set_motor_encoder,         MH_P    },
    { 'R', WM_R,    NULL,                          NULL,      false, command_waypoint_run,              WH_R    },
    { 'S', WM_S,    extended_menu_set_waypoint,    NULL,      true,  command_waypoint_set,              WH_S    },
    { 'T', MM_T,    NULL,                          NULL,      true,  command_test_motors,               MH_T    },
    { 'V', MM_V,    NULL,                          NULL,      true,  command_close_gripper,             MH_V    },
    { 'W', CM_W,    NULL,                          NULL,      true,  command_calibrate_home,            MH_W    },
    { 'X', OM_X,    NULL,                          main_menu, false, NULL,                              OH_X    },
    { 'Z', MM_Z,    NULL,                          NULL,      false, command_poll_pins,                 MH_Z    },
    { '%', MM_PCT,  NULL,                          NULL,      true,  command_set_motor_percent,         MH_PCT  },
    { '!', MM_BANG, NULL,                          NULL,      false, command_emergency_stop,            MH_BANG },
    { '?', MM_HELP, NULL,                          NULL,      false, command_print_help,                MH_HELP },
    { 0 }  // Terminate menus with an entry filled with zeros.
};

static const menu_item_t *current_menu = main_menu;

void menu_set_current_menu(const menu_item_t *menu)
{
    assert(menu);
    current_menu = menu;
}

const menu_item_t * menu_item_by_command_char(char ch)
{
    int n = 0;

    while (current_menu[n].command_char != 0) {
        assert(n < menu_item_max_count);
        if (toupper(ch) == toupper(current_menu[n].command_char))
            return &current_menu[n];
        n++;
    }

    return NULL;
}

void menu_help(void)
{
    log_writeln(F("Command list:"));

    int n = 0;
    int longest_name_nbytes = 0;

    while (current_menu[n].command_char != 0) {
        assert(n < menu_item_max_count);
        const menu_item_t *item = &current_menu[n];
        int len = strlen_P(item->name);
        longest_name_nbytes = len > longest_name_nbytes ? len : longest_name_nbytes;
        n++;
    }

    n = 0;
    while (current_menu[n].command_char != 0) {
        const menu_item_t *item = &current_menu[n];

        // Pad with spaces afer the name to align the help message.
        char spaces[menu_item_max_name_nbytes];  // Just a buffer of whitespace.
        int nspaces = longest_name_nbytes - strlen_P(item->name) + 1;
        nspaces = (nspaces >= menu_item_max_name_nbytes) ? menu_item_max_name_nbytes - 1 : nspaces;
        memset(spaces, ' ', nspaces);
        spaces[nspaces] = '\0';

        // log_writeln(F("  %c : %s %s %s"), item->command_char, item->name, spaces, item->help);
        log_write(F("  %c : "), item->command_char);
        log_write((const __FlashStringHelper *)item->name);
        log_write(F("%s"), spaces);
        log_writeln((const __FlashStringHelper *)item->help);
        n++;
    }
}

static bool menu_test_single_menu(const menu_item_t *menu_items)
{
    bool ret = true;
    int n = 0;

    while (menu_items[n].command_char != 0) {
        if (n >= menu_item_max_count) {
            LOG_ERROR(F("menu has more than %d items"), menu_item_max_count);
            ret = false;
            break;
        }

        const menu_item_t *item = &menu_items[n];
        if (!item->name) {
            LOG_ERROR(F("%d NULL name"), n);
            ret = false;
        }
        if (strlen_P(item->name) > menu_item_max_name_nbytes) {
            LOG_ERROR(F("%d name too long"), n);
            ret = false;
        }
        if ((!item->function) && (!item->sub_menu)) {
            LOG_ERROR(F("%d NULL function and NULL sub_menu"), n);
            ret = false;
        }
        if (!item->help) {
            LOG_ERROR(F("%d NULL help"), n);
            ret = false;
        }

        n++;
    }
    return ret;
}

bool menu_test(void)
{
    bool ret = true;

    ret = menu_test_single_menu(main_menu);
    ret = menu_test_single_menu(waypoint_menu) ? ret : false;
    ret = menu_test_single_menu(calibration_menu) ? ret : false;

    return ret;
}
