#pragma once

/*
 * Declarations for state machine functions.
 */

typedef enum {
  SM_STATE_FIRST = 0,
  SM_STATE_INIT = SM_STATE_FIRST,
  SM_STATE_MOTORS_OFF,
  SM_STATE_MOTORS_ON,
  SM_STATE_ERROR,
  SM_STATE_COUNT,
  SM_STATE_LAST = SM_STATE_COUNT - 1,
} sm_state_t;

extern sm_state_t sm_state_current;

const char* sm_get_state_name(sm_state_t state);
sm_state_t sm_execute(sm_state_t current_state);
bool sm_test();