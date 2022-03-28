#pragma once

/*
 * Declarations for state machine functions.
 */

#include <Arduino.h>

typedef void (*sm_state_func)(void);

void sm_init(void);
sm_state_func sm_get_state();
void sm_set_next_state(sm_state_func);
void sm_set_exit_current_state(sm_state_func);
void sm_execute(void);

void sm_set_state_name(const __FlashStringHelper *name);

void sm_motors_off_enter(void);
void sm_motors_off_execute(void);
void sm_motors_on_enter(void);
void sm_motors_on_execute(void);
void sm_motors_on_exit(void);
void sm_error_enter(void);
void sm_error_execute(void);

bool sm_test();
