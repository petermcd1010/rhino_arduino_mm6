#pragma once
/*
 * Declarations for state machine functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

typedef struct _sm_state_t {
    void (*run)(struct _sm_state_t *state);
    void (*break_handler)(struct _sm_state_t *state);  // May be NULL.
    const __FlashStringHelper *name;
    void *                     data;
} sm_state_t;

typedef void (*sm_state_func)(void);

void sm_init(void);
sm_state_t sm_get_state(void);
void sm_set_next_state(sm_state_t state);
void sm_execute(void);

void sm_motors_off_enter(sm_state_t *state);
void sm_motors_off_execute(sm_state_t *state);
void sm_motors_on_enter(sm_state_t *state);
void sm_motors_on_execute(sm_state_t *state);
void sm_motors_on_exit(sm_state_t *state);
void sm_error_enter(sm_state_t *state);
void sm_error_execute(sm_state_t *state);

int sm_get_enabled_motors_mask(void);
void sm_set_enabled_motors_mask(int mask);

bool sm_test(void);
