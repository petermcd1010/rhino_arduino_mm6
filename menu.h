#pragma once
/*
 * Declarations for menu functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

typedef struct _menu_item_t {
    char                command_char; // Character typed for this menu item.
    const PROGMEM char *name;       // Name of this menu item.
    void (*print_sub_menu_fn)();  // Function called if there's a sub menu.
    const _menu_item_t *sub_menu;
    bool                has_args; // True if this command has arguments, false otherwise.
    int (*function)(char *payload, size_t nbytes);  // Function to call after the command is typed.
    const PROGMEM char *help;       // Help message for this menu item.
} menu_item_t;

void menu_set_current_menu(const menu_item_t *);
const menu_item_t * menu_item_by_command_char(char ch);
void menu_help(void);
bool menu_test(void);
