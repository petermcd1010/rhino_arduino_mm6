#pragma once

/*
 * Declarations for menu functions.
*/

#include <Arduino.h>

typedef struct {
  char command_char;
  char *pname;
  void(*print_sub_menu_fn)();
  bool has_args;
  int (*pfunction)(char *payload, size_t nbytes);
  char *phelp;
} menu_item_t;

const menu_item_t* menu_item_by_command_char(char ch);
void menu_help();
bool menu_test();
