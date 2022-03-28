#pragma once

/*
 * Declarations for menu functions.
 */

#include <Arduino.h>

typedef struct {
    char  command_char; // Character typed for this menu item.
    char *pname;  // Name of this menu item.
    void (*print_sub_menu_fn)();  // Function called if there's a sub menu.
    bool  has_args; // True if this command has arguments, false otherwise.
    int (*pfunction)(char *payload, size_t nbytes);  // Function to call after the command is typed.
    char *phelp;  // Help message for this menu item.
} menu_item_t;

const menu_item_t * menu_item_by_command_char(char ch);
void menu_help();
bool menu_test();
