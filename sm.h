#pragma once
/*
 * Declarations for state machine functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

typedef struct _sm_state_t {
    void (*run)(void);
    void (*break_handler)(void);
    bool                process_break_only; // When true, only processes CTRL+C.
    const PROGMEM char *name;
} sm_state_t;

extern const sm_state_t sm_state_error_enter;
extern const sm_state_t sm_state_motors_off_enter;
extern const sm_state_t sm_state_motors_on_enter;

typedef void (*sm_state_func)(void);

void sm_init(void);
sm_state_t sm_get_state(void);
void sm_set_next_state(sm_state_t state);
void sm_execute(void);

void sm_motors_off_enter(void);
void sm_motors_off_execute(void);
void sm_motors_on_enter(void);
void sm_motors_on_execute(void);
void sm_motors_on_exit(void);
void sm_error_enter(void);
void sm_error_execute(void);

int sm_get_enabled_motors_mask(void);
void sm_set_enabled_motors_mask(int mask);

bool sm_test(void);
