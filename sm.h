#pragma once
/*
 * Declarations for state machine functions.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>

// Used for status messages.
typedef struct {
    struct {
        int  encoder;
        bool switch_triggered;
    } motor[MOTOR_ID_COUNT];
} status_t;

void gather_status(status_t *status);
bool status_changed(status_t *status);
void print_status(status_t *status);

typedef struct _sm_state_t {
    void (*run)(void);
    void (*break_handler)(void);
    bool                process_break_only; // When true, only processes CTRL+C.
    const PROGMEM char *name;
} sm_state_t;

typedef enum {
    SM_DISPLAY_MODE_ENCODER = 0,
    SM_DISPLAY_MODE_ANGLE,
    SM_DISPLAY_MODE_PERCENT
} sm_display_mode;

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
void sm_set_display_mode(sm_display_mode display_mode);

bool sm_test(void);
