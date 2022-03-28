/*
 * Implementation for menu functions.
 */

#include "command.h"
#include "config.h"
#include "log.h"
#include "menu.h"

static const int menu_item_max_name_nbytes = 25;

static void extended_menu_robot_id()
{
    log_writeln(F(""));
    log_writeln(F("Current robot ID: %s."), config_robot_name_by_id[config.robot_id]);
    log_writeln(F("Available robot IDs:"));
    for (int i = CONFIG_ROBOT_ID_FIRST; i <= CONFIG_ROBOT_ID_LAST; i++) {
        log_writeln(F("  %d. %s."), i, config_robot_name_by_id[i]);
    }
    log_writeln(F("Select new robot ID, or press [RETURN] to keep current robot ID."));
    log_write(F(">"));
}

static void extended_menu_robot_serial()
{
    log_writeln(F(""));
    log_writeln(F("Current robot serial is '%s'."), config.robot_serial);
    log_writeln(F("Enter new robot serial, or press [RETURN] to keep current serial."));
    log_write(F(">"));
}

static void extended_menu_robot_name()
{
    log_writeln(F(""));
    log_writeln(F("Current robot name is '%s'."), config.robot_name);
    log_writeln(F("Enter new robot name shorter than %d characters, or press [RETURN] to keep current name."), CONFIG_ROBOT_NAME_NBYTES);
    log_write(F(">"));
}

static void extended_menu_factory_reset()
{
    log_writeln(F(""));
    log_writeln(F("Type 'RESET' in all capital letters to clear the EEPROM and reboot the system or <CTRL+C> to exit."));
    log_write(F(">"));
}

static void extended_menu_reboot()
{
    log_writeln(F(""));
    log_writeln(F("Type 'REBOOT' in all capital letters to reboot the system or <CTRL+C> to exit."));
    log_write(F(">"));
}

static const menu_item_t menu_item_by_index[] = {  // TODO: F()
    { '1', "print configuration",    NULL,                        true,  command_print_config,           "-- print configuration."                                                                      },
    { '2', "configure robot ID",     extended_menu_robot_id,      true,  command_config_robot_id,        "[id] print or set configured robot ID."                                                       },
    { '3', "configure robot serial", extended_menu_robot_serial,  true,  command_config_robot_serial,    "[string] -- print or set configured robot serial."                                            },
    { '4', "configure robot name",   extended_menu_robot_name,    true,  command_config_robot_name,      "[name] -- print or set configured robot name."                                                },
    { '0', "write configuration",    NULL,                        true,  command_config_write,           "Write configuration data to EEPROM."                                                          },
    { 'C', "run calibration",        NULL,                        true,  command_run_calibration,        "[motorid] -- calibrate motor and switch limits; calibrates all motors if none given."         },
    { 'D', "PID mode",               NULL,                        false, command_pid_mode,               "-- Enable/disable motors"                                                                     },
    { 'E', "emergency stop",         NULL,                        false, command_emergency_stop,         "-- execute hardware emergency stop (E-Stop). Enters 'error' state. Requires reboot to reset." },
    { 'G', "set gripper position",   NULL,                        true,  command_set_gripper_position,   "-- set current encoders as gripper?"                                                          },
    { 'H', "set home position",      NULL,                        false, command_set_home_position,      "-- set current encoders as home position."                                                    },
    { 'M', "print motor status",     NULL,                        false, command_print_motor_status,     "-- print motor status."                                                                       },
    { 'N', "set motor angle",        NULL,                        true,  command_set_motor_angle,        "motorid degrees -- degrees is 0.0 to 360.0, +15, -20, +, -, ++, --."                          },
    { 'P', "set motor encoder",      NULL,                        true,  command_set_motor_encoder,      "motorid encoder -- encoder is in the range X - Y."                                            }, // TODO
    { 'Q', "run test sequence",      NULL,                        false, command_run_test_sequence,      ""                                                                                             },
    { 'S', "start/stop motors",      NULL,                        false, command_start_stop_motors,      ""                                                                                             },
    { 'T', "test motors",            NULL,                        false, command_test_motors,            "-- test motors."                                                                              },
    { 'V', "print software version", NULL,                        false, command_print_software_version, "-- print software version."                                                                   },
    { 'W', "waypoint",               NULL,                        true,  command_waypoint,               ""                                                                                             },
    { '*', "factory reset",          extended_menu_factory_reset, false, command_factory_reset,          "RESET -- factory reset system, clearing EEPROM and rebooting."                                },
    { '!', "reboot",                 extended_menu_reboot,        true,  command_reboot,                 "REBOOT -- reboot system. Requires typing the 'REBOOT' keyword."                               },
    { '?', "print help",             NULL,                        false, command_print_help,             "-- print this help message."                                                                  },
};
#define MENU_ITEM_COUNT sizeof(menu_item_by_index) / sizeof(menu_item_by_index[0])

const menu_item_t * menu_item_by_command_char(char ch)
{
    for (int i = 0; i < MENU_ITEM_COUNT; i++) {
        if (toupper(ch) == toupper(menu_item_by_index[i].command_char))
            return &menu_item_by_index[i];
    }
    return NULL;
}

void menu_help()
{
    log_writeln(F("Command list:"));

    int longest_name_nbytes = 0;

    for (int i = 0; i < MENU_ITEM_COUNT; i++) {
        menu_item_t *pitem = &menu_item_by_index[i];
        int n = strlen(pitem->pname);
        longest_name_nbytes = n > longest_name_nbytes ? n : longest_name_nbytes;
    }

    for (int i = 0; i < MENU_ITEM_COUNT; i++) {
        menu_item_t *pitem = &menu_item_by_index[i];
        // Pad with spaces afer the name to align the help message.
        char spaces[menu_item_max_name_nbytes];  // Just a buffer of whitespace.
        int nspaces = longest_name_nbytes - strlen(pitem->pname);
        nspaces = (nspaces >= menu_item_max_name_nbytes) ? menu_item_max_name_nbytes - 1 : nspaces;
        memset(spaces, ' ', nspaces);
        spaces[nspaces] = '\0';
        log_writeln(F("%c: %s %s %s"), pitem->command_char, pitem->pname, spaces, pitem->phelp);
    }
}

bool menu_test()
{
    bool ret = true;

    for (int i = 0; i < MENU_ITEM_COUNT; i++) {
        menu_item_t *pitem = &menu_item_by_index[i];
        if (!pitem->pname) {
            LOG_ERROR(F("%d NULL pname"), i);
            ret = false;
        }
        if (strlen(pitem->pname) > menu_item_max_name_nbytes) {
            LOG_ERROR(F("%d pname too long"), i);
            ret = false;
        }
        if (!pitem->pfunction) {
            LOG_ERROR(F("%d NULL pfunction"), i);
            ret = false;
        }
        if (!pitem->phelp) {
            LOG_ERROR(F("%d NULL phelp"), i);
            ret = false;
        }
    }

    return ret;
}
