/*
 * Implementation for MegaMotor6 hardware functionality.
 */

#define __ASSERT_USE_STDERR
#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "motor.h"

// MM6 motor I/O lines.
typedef struct {
  unsigned short out_direction;  // Digital. LOW = forward direction. HIGH = reverse direction.
  unsigned short out_pwm;  // Digital.
  unsigned short out_brake;  // Digital. LOW = disable brake. HIGH = enable brake.
  unsigned short in_current_draw;  // Analog. 377uA/A. What's the resistance?
  unsigned short in_thermal_overload;  // Digital. Becomes active at 145C. Chip shuts off at 170C.
  unsigned short in_switch;  // Digital. LOW = switch triggered. HIGH = switch not triggered.
  unsigned short in_quadrature_encoder_a;  // Digital.
  unsigned short in_quadrature_encoder_b;  // Digital.
} motor_pinout_t;

static const motor_pinout_t motor_pinout[MOTOR_ID_COUNT] = {
  {  11,  10,  12,  A5,  14,  A8,  47,  46 },  // Motor A.
  {  A9,   7,  39,  A0, A11,  26,  32,  33 },  // Motor B.
  {   3,   5,   4,  A6,   6,  28,  45,  44 },  // Motor C.
  {  A1,   8,  A2,  A4,  A3,  30,  34,  35 },  // Motor D.
  {  17,   2,  16,  A7,  15,  40,  43,  42 },  // Motor E.
  {  51,   9,  50, A10,  52,  38,  36,  37 },  // Motor F.
};

typedef enum {
  CAL_STATE_INIT = 0,
  CAL_STATE_SEARCH,
  CAL_STATE_SWITCH_FORWARD_ON,
  CAL_STATE_SWITCH_FORWARD_OFF,
  CAL_STATE_SWITCH_REVERSE_ON,
  CAL_STATE_SWITCH_REVERSE_OFF,
  CAL_STATE_DONE,
} cal_state_t;

static const char* cal_state_name_by_index[] = {
  "CAL_STATE_INIT",
  "CAL_STATE_SEARCH",
  "CAL_STATE_SWITCH_FORWARD_ON",
  "CAL_STATE_SWITCH_FORWARD_OFF",
  "CAL_STATE_SWITCH_REVERSE_ON",
  "CAL_STATE_SWITCH_REVERSE_OFF",
  "CAL_STATE_DONE",
};

typedef enum {
  CAL_ERROR_NONE = 0,
  CAL_ERROR_NOT_CONFIGURED,
  CAL_ERROR_UNEXPECTED_MIN_ENCODER,
  CAL_ERROR_UNEXPECTED_MAX_ENCODER,
  CAL_ERROR_MOTOR_ERROR,
} cal_error_t;

typedef struct {
  motor_id_t motor_id;
  cal_state_t state; 
  cal_error_t error;
  int delta;
  int target;
  int stuck_check_start_encoder;
  int stuck_check_start_ms;
  bool found_min_encoder;
  bool found_max_encoder;
  int min_encoder;
  int max_encoder;
  bool switch_triggered;
  int switch_forward_on_encoder;
  int switch_forward_off_encoder;
  int switch_reverse_on_encoder;
  int switch_reverse_off_encoder;
} cal_data_t;

// For motor direction pin.
static const int motor_direction_forward = LOW;
static const int motor_direction_reverse = HIGH;

static motor_state_t motor_state[MOTOR_ID_COUNT] = {};

static const int motor_min_pwm = 55;
static int tracking = 0;
static int tracked[] = { 0, 0, 0, 0, 0, 0 }; // Last value while tracking.
static int Motor_PID[MOTOR_ID_COUNT] = {0, 0, 0, 0, 0, 0}; // PID on or off.
static int Gripper_StallC = 0;
static int Gripper_StallE = 0;
static int Gripper_StallX = 0;
static int SyncMove_Status = 0;
static int Forward_Logic[] = {0,0,0,0,0,0}; // Forward Logic - The value for the Direction IO Line when the motor needs to move forward to sync with encoders.
static int Reverse_Logic[] = {1,1,1,1,1,1}; // Reverse Logic - The value for the Direction IO Line when the motor needs to move Reverse to sync with encoders.

// Configuration stored in RAM and saved across reset/reboot that don't include a power-cycle of the board.
const int noinit_data_version = 2;
const int noinit_data_magic = 0xABCD1234;
typedef struct {
  // nbytes, version, magic are used to verify valid data.
  size_t nbytes;
  int version;
  int magic;
  int previous_quadrature_encoder[MOTOR_ID_COUNT];
  int encoder[MOTOR_ID_COUNT];
} noinit_data_t;

static noinit_data_t noinit_data __attribute__ ((section (".noinit")));  // NOT reset to 0 when the CPU is reset.

static void check_noinit_data()
{
  // Zero out saved variables on power cycle. On reset, these values are NOT erased.

  if ((noinit_data.nbytes != sizeof(noinit_data_t) ||
      (noinit_data.version != noinit_data_version) ||
      (noinit_data.magic != noinit_data_magic))) {
    log_writeln(F("Initializing noinit data."));
    noinit_data.nbytes = sizeof(noinit_data_t);
    noinit_data.version = noinit_data_version;
    noinit_data.magic = noinit_data_magic;
    for (int i = 0; i < MOTOR_ID_COUNT; i++) {
      noinit_data.encoder[i] = 0;
      noinit_data.previous_quadrature_encoder[i] = 0;
    }
  } else {
    log_writeln(F("Reset without power cycle detected. Reusing stored motor encoder values."));
  }
}

static void track_report(motor_id_t motor_id) {
  if (tracking > 0) {
    int Position = motor_get_encoder(motor_id);
    if (tracked[motor_id]!=Position) {
      Serial.print("@");
      if (tracking==1) {
        Serial.print(char(motor_id+65));
        Serial.print(motor_get_encoder(motor_id));
      } else if (tracking==2) {
        Serial.print(char(motor_id+97));
        Serial.print(motor_get_angle(motor_id));
      }
      Serial.print(":HS");
      Serial.print(motor_state[motor_id].switch_previously_triggered);
      Serial.println(":");
      tracked[motor_id]=Position;
    }
  }
}

static void motor_init(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  // Configure and initialize outputs.
  pinMode(motor_pinout[motor_id].out_brake, OUTPUT);
  pinMode(motor_pinout[motor_id].out_direction, OUTPUT);
  pinMode(motor_pinout[motor_id].out_pwm, OUTPUT);
  digitalWrite(motor_pinout[motor_id].out_direction, motor_direction_forward);

  // Configure inputs.
  pinMode(motor_pinout[motor_id].in_switch, INPUT_PULLUP);
  pinMode(motor_pinout[motor_id].in_thermal_overload, INPUT_PULLUP);
  pinMode(motor_pinout[motor_id].in_quadrature_encoder_a, INPUT_PULLUP);
  pinMode(motor_pinout[motor_id].in_quadrature_encoder_b, INPUT_PULLUP);

  motor_set_pid_enable(motor_id, false);
}

void motor_init_all() 
{
  // Timer setup: Allows preceise timed measurements of the quadrature encoder.
  cli();  // Disable interrupts.

  // Configure timer1 interrupt at 1kHz.
  TCCR1A = 0;  // Set entire TCCR1A register to 0.
  TCCR1B = 0;  // Same for TCCR1B.
  TCNT1  = 0;  // Initialize counter value to 0.

  // Set timer count for 2khz increments.
  OCR1A = 1000;  // = (16*10^6) / (2000*8) - 1

  TCCR1B |= (1 << WGM12);  // Turn on CTC mode.
  TCCR1B |= (1 << CS11);  // Set CS11 bit for 8 prescaler.
  TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt.
  sei();  // Enable interrupts.

  // Get the Angle Offsets and Forward_Logic for ALL motors
  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++){    
    int logic = 0;
    
    // The Angle Offset is used in the Position-to-Angle and Angle-to-Position calculations
    //   Each Rhino Robot may have mechanical differences in the positions of the home swithes, 
    //   so the encoder count when the arm is straight up is stored as an "AngleOffset" so that the
    //   MegaMotor6 Angle Values will work the actual physical position of the robot
    //   while the Position Values work with positions relative to the home switches.
    //     The values for the AngleOffets come from the "~" Command.

    // EEPROM.get(AngleOffsetELoc[iMotor], AngleOffset[iMotor]);

    // The Forward and Reverse Locic is used to turn the motors in the right direction to sync with the encoders.
    //   Each Rhino Robot may have the wires to the motors reversed.
    //   So the Forward and Reverse Logic is used to correct that.    
    //     The values for the Direction Logic come from the "t" command.
    //       Since the Forward and Reverse Locic are used for an I/O line the values are 0 or 1'
#if 0
    logic = config.motor[i].direction_logic;  // 0 = forward, 1 = reverse.
    if (logic != 0) 
      logic = !0;  
    Reverse_Logic[i] = logic; // Reverse Logic - The value for the Direction IO Line when the motor needs to move Reverse. Defaults to 1.
    Forward_Logic[i] = !logic; // Forward Logic - The value for the Direction IO Line when the motor needs to move forward. Defaults to 0.
#endif

    // The Motor Locic is used to contol which way the motors turn in responce to the Positions.
    //   Each Rhino Robot may have the motor assembled on either side - which winds up reversing the motors direction mechanically.
    //   So the motor locic is used to correct that.    
    //     The values for the Motor Logic are set by the setup.
    //       Since the Forward and Reverse Locic are used to invert the position the values are 1 or -1
    motor_state[i].logic = config.motor[i].orientation; 
  }  

  check_noinit_data();
  for (int motor_id = MOTOR_ID_FIRST; motor_id <= MOTOR_ID_LAST; motor_id++) {    
    motor_init(motor_id);
  }
}

static bool get_thermal_overload_active(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  /* 
   * From the LMD18200 datasheet:
   * Pin 9, THERMAL FLAG Output: This pin provides the thermal warning flag output signal. 
   * Pin 9 becomes active- low at 145°C (junction temperature). However the chip will not 
   * shut itself down until 170°C is reached at the junction.
  */
  return digitalRead(motor_pinout[motor_id].in_thermal_overload) == 0;
}

bool motor_get_thermal_overload_detected(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  if (motor_state[motor_id].error_flags & MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED)
    return true;
}

bool motor_get_thermal_overload_detected()
{
  for (int motor_id = MOTOR_ID_FIRST; motor_id <= MOTOR_ID_LAST; motor_id++) {
    if (motor_get_thermal_overload_detected(motor_id))
      return true;
  }

  return false;
}

void motor_clear_thermal_overload(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  motor_state[motor_id].error_flags &= ~MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED;
}

static bool get_overcurrent_active(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  // TODO: Implement get_ovecurrent_active().
  return false;
}

bool motor_get_overcurrent_detected(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  if (motor_state[motor_id].error_flags & MOTOR_ERROR_FLAG_OVERCURRENT_DETECTED)
    return true;
  else
    return false;
}

bool motor_get_overcurrent_detected()
{
  for (int motor_id = MOTOR_ID_FIRST; motor_id <= MOTOR_ID_LAST; motor_id++) {
    if (motor_get_overcurrent_detected(motor_id))
      return true;
  }

  return false;
}

void motor_clear_overcurrent(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  motor_state[motor_id].error_flags &= ~MOTOR_ERROR_FLAG_OVERCURRENT_DETECTED;
}

int motor_get_current_draw(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  // Ignores motor_configured(motor_id).

  // LMD18200 datasheet says 377uA/A. What's the resistance?
  return analogRead(motor_pinout[motor_id].in_current_draw);  // 0 - 1023.
}

static void set_brake(motor_id_t motor_id, bool enable)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  digitalWrite(motor_pinout[motor_id].out_brake, enable ? HIGH : LOW);
}

bool motor_set_pid_enable(motor_id_t motor_id, bool enable)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  // Will turn off a motor if it's not configured, regardless of enable's value.

  if (enable && config.motor[motor_id].configured) {
    set_brake(motor_id, false);
    motor_state[motor_id].pid_enabled = true;
    return true;
  } else {
    set_brake(motor_id, true);
    digitalWrite(motor_pinout[motor_id].out_pwm, LOW);  // Set PWM speed to zero.  TODO: Redundant with set_speed below?
    motor_set_speed(motor_id, 0);
    motor_state[motor_id].pid_enabled = false;
    return !enable;  // Return false if motor wasn't configured but enble == true. Return true otherwise.
  }
}

void motor_set_pid_enable(bool enable)
{
  static bool motor_enabled = false;
  int success_count = 0;

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    if (motor_set_pid_enable(i, enable))
      success_count++;
  }

  // motor_sync_move_enabled = enable;
  motor_enabled = enable;
  log_writeln(F("Motor electronics for %d motors %s."), success_count, enable ? "enabled" : "disabled");
}

bool motor_get_pid_enable(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  return motor_state[motor_id].pid_enabled;
}

bool motor_configured(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  return config.motor[motor_id].configured;
}

void motor_set_target_encoder(motor_id_t motor_id, int encoder)
{  
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  // TODO: assert valid encoder?
  if (!motor_get_pid_enable(motor_id))
    return;

  motor_state[motor_id].target_encoder = encoder * motor_state[motor_id].logic;
  motor_state[motor_id].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET;  
}

int motor_get_encoder(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  if (!motor_get_pid_enable(motor_id))
    return 0;
  
  return noinit_data.encoder[motor_id] * motor_state[motor_id].logic;
}

void motor_print_encoders() 
{
  log_writeln(F("Current Positions: "));
  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    log_write(F("%c=%d%c"), 'A' + i, motor_get_encoder(i), (i < MOTOR_ID_LAST) ? ',' : ' ');
  }
  log_writeln();
}

float motor_get_encoder_steps_per_degree(motor_id_t motor_id) 
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  if (!motor_configured(motor_id))
    return 0.0f;

  switch (motor_id) {
    case MOTOR_ID_F: 
      if (config.robot_id == CONFIG_ROBOT_ID_RHINO_XR_4)
        return 17.5;  // (4.4)(66.1/1) XR4        
      else
        return 29.5;  // (4.4)(66.1/1) XR3
      break;
    case MOTOR_ID_E:  // Fallthrough.
    case MOTOR_ID_D:  // Fallthrough.
    case MOTOR_ID_C:
      if (config.robot_id == CONFIG_ROBOT_ID_RHINO_XR_4)
        return 35;  // (8.8)(66.1/1) XR4
      else
        return 36;  // (8.8)(66.1/1) XR3
      break;
    case MOTOR_ID_B:
      return 12.5;  // (5.51)(165.4/1) XR4
      break;
    case MOTOR_ID_A:
      return 1;
  }  

  assert(false);
}

int motor_angle_to_encoder(motor_id_t motor_id, float angle) {
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  if (!motor_configured(motor_id))
    return 0;
  
  return (angle * motor_get_encoder_steps_per_degree(motor_id)) + config.motor[motor_id].angle_offset;
}

void motor_set_target_angle(motor_id_t motor_id, float angle) 
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  if (!motor_configured(motor_id)) 
    return;

  // TODO: assert valid angle?
  int encoder = motor_angle_to_encoder(motor_id, angle);
  motor_set_target_encoder(motor_id, encoder);
}

float motor_get_angle(motor_id_t motor_id) 
{  
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  if (!motor_configured(motor_id))
    return 0.0f;
  
  return (motor_get_encoder(motor_id) - config.motor[motor_id].angle_offset) / motor_get_encoder_steps_per_degree(motor_id);
}

void motor_set_position_to_home(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  
  motor_state[motor_id].target_encoder = 0;
  log_writeln(F("Setting motor %c current position to home."), 'A' + motor_id);
}

void motor_set_speed(motor_id_t motor_id, int speed)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  assert((speed >= motor_min_speed) && (speed <= motor_max_speed));

  if (!config.motor[motor_id].configured || !motor_state[motor_id].pid_enabled) {
    motor_state[motor_id].pwm = 0;
    analogWrite(motor_pinout[motor_id].out_pwm, 0);          
    return;
  }

  // Convert speed's +/- 255 value to PWM and Direction.
  if (speed == 0) {
    motor_state[motor_id].pwm = 0;
  } else {
    int pwm = abs(speed);
    motor_state[motor_id].pwm = pwm < motor_min_pwm ? motor_min_pwm : pwm;
  }
  digitalWrite(motor_pinout[motor_id].out_direction, speed >= 0 ? motor_direction_forward : motor_direction_reverse);          
  analogWrite(motor_pinout[motor_id].out_pwm, motor_state[motor_id].pwm);    
  motor_state[motor_id].speed = speed;
}

bool motor_get_switch_triggered(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  if (!motor_get_pid_enable(motor_id))
    return false;
  return motor_state[motor_id].switch_triggered;
}

static void print_motor_delta(int delta)
{
  if (delta == 0) {
    log_write(F("0"));
  } else if (delta > 0) {
    log_write(F("+%d"), delta);
  } else {
    log_write(F("%d"), delta);
  }
}

static void motor_test(motor_id_t motor_id) 
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  const int test_speed = 255 - motor_min_pwm;
  const int delay_ms = 50;

  // Mark motor as configured, so motor-control commands will execute.
  bool was_configured = config.motor[motor_id].configured;
  config.motor[motor_id].configured = true;
  motor_state[motor_id].pid_enabled = true;

  set_brake(motor_id, false);
  motor_set_speed(motor_id, 0);
    
  log_write(F("  %c: Reverse "), 'A' + motor_id);  
  int position1 = motor_get_encoder(motor_id);
  motor_set_speed(motor_id, -test_speed);
  log_write(F("on, "));
  delay(delay_ms);  // Allow motor to move.  
    
  motor_set_speed(motor_id, 0); 
  log_write(F("off. "));
  delay(delay_ms);  // Allow motor to move.  

  int position2 = motor_get_encoder(motor_id); 
  int reverse_delta = position2 - position1;

  log_write(F("Reverse delta: "));
  print_motor_delta(reverse_delta);
  log_write(F(". "));

  log_write(F("Forward "));
  //Serial.print("Moving Motor ");
  //Serial.print(char(m+65));
  //Serial.print(" forward ");

  motor_set_speed(motor_id, test_speed);
  log_write(F("on, "));
  delay(delay_ms);         // Short Delay to allow the motor to move.  
    
  motor_set_speed(motor_id, 0);
  log_write(F("off. "));
  delay(delay_ms);         // Short Delay to allow the motor to stop.  

  int position3 = motor_get_encoder(motor_id);
  int forward_delta = position3 - position2;

  log_write(F("Forward delta: "));
  print_motor_delta(forward_delta);

#if 0
  bool done = false;
  do {
    if (((position2 - position1) > 0 ) && ((position3 - position2) < 0)) {
      log_write(F("Reversing. "));
      Forward_Logic[motor_id] = !Forward_Logic[motor_id];
      Reverse_Logic[motor_id] = !Reverse_Logic[motor_id];
      config_set_direction_logic(motor_id, Reverse_Logic[motor_id]);
      done = true;
      // TODO: Test motor[i] again.
    } else {
      config_set_direction_logic(motor_id, Forward_Logic[motor_id]);
      done = true;
    }
  } while (!done);
#endif
  
  const __FlashStringHelper *pfailure_message = NULL;

  if (position1 == position2) {
    pfailure_message = F("Failed to move when commanded");
  } else if ((forward_delta < 0) == (reverse_delta < 0)) {
    // To Test, execute pinMode(51, INPUT_PULLUP) to disable Motor F's direction pin.
    pfailure_message = F("Failed to switch direction when commanded");
  } else if ((reverse_delta > 0) && (forward_delta < 0)) {
    pfailure_message = F("Motor +/- wired backwards");
  }    

  config.motor[motor_id].configured = was_configured;
  config_set_motor_configured(motor_id, pfailure_message == NULL);
  motor_set_pid_enable(motor_id, config.motor[motor_id].configured);
  if (pfailure_message) {
    log_write(F(" ... FAILED ("));
    log_write(pfailure_message);
    log_writeln(F(")."));
  } else {
    config_set_motor_orientation(motor_id, MOTOR_ORIENTATION_NOT_INVERTED);
    config_set_motor_polarity(motor_id, MOTOR_POLARITY_NOT_REVERSED);
    log_writeln(F(" ... Passed."));
  }
}

void motor_test_all() {
  log_writeln(F("Testing motors"));

  for (int motor_id = MOTOR_ID_FIRST; motor_id <= MOTOR_ID_LAST; motor_id++) {    
    bool was_pid_enabled = motor_get_pid_enable(motor_id);
    motor_set_pid_enable(motor_id, true);
    delay(25);
    motor_test(motor_id);
    motor_set_pid_enable(motor_id, was_pid_enabled);
  }
  
  log_writeln(F("Done testing motors."));
}

static bool motor_interrogate_limit_switch_a() {  
  if (!config.motor[MOTOR_ID_A].configured)
    return false;

  motor_set_target_encoder(MOTOR_ID_A, 9999);
  delay(2000);
  int CurF = analogRead(motor_pinout[MOTOR_ID_A].in_current_draw);
  int EncF = noinit_data.encoder[MOTOR_ID_A];
  int SwcF = digitalRead(motor_pinout[MOTOR_ID_A].in_switch);
  motor_set_target_encoder(MOTOR_ID_A, -9999);
  delay(1500);
  int CurR = analogRead(motor_pinout[MOTOR_ID_A].in_current_draw);
  int EncR = noinit_data.encoder[MOTOR_ID_A];
  int SwcR = digitalRead(motor_pinout[MOTOR_ID_A].in_switch);  
  motor_set_target_encoder(MOTOR_ID_A, 9999);
  delay(1500);
  motor_set_target_encoder(MOTOR_ID_A, -9999);
  delay(1500);
    //Serial.print("  For Cur=");
    //Serial.println(CurF);
    //Serial.print("  For Enc=");
    //Serial.println(EncF);
    //Serial.print("  For Swt=");
    //Serial.println(SwcF);
    //Serial.print("  Rev Cur=");
    //Serial.println(CurR);
    //Serial.print("  Rev Enc=");
    //Serial.println(EncR);
    //Serial.print("  Rev Swt=");
    //Serial.println(SwcR);  
    //Serial.print("  motor_state[MOTOR_ID_A].switch_forward_on=");
    //Serial.println(motor_state[MOTOR_ID_A].switch_forward_on);        
  if (SwcF == 0) {
    // Encoder goes Positive towards switch.
    int OverSwitch = ((motor_state[MOTOR_ID_A].switch_forward_on + EncF ) / 2);      
    motor_set_target_encoder(MOTOR_ID_A, OverSwitch);
    do {track_report(MOTOR_ID_A);} while (noinit_data.encoder[MOTOR_ID_A] != OverSwitch);
    noinit_data.encoder[MOTOR_ID_A] = 0;  
    motor_state[MOTOR_ID_A].target_encoder  = 0;
    motor_state[MOTOR_ID_A].logic = -1;
    // config_set_motor_orientation(MOTOR_ID_A, motor_state[MOTOR_ID_A].orientation);
    config_set_gripper_open_encoder(-140);
    config_set_gripper_close_encoder(-310);
    Serial.println("Done");
  } else {
    // Encoder goes Negative towards switch.
    int OverSwitch = ((motor_state[MOTOR_ID_A].switch_reverse_on + EncR ) / 2);      
    motor_set_target_encoder(MOTOR_ID_A, OverSwitch);
    do {track_report(MOTOR_ID_A);} while (noinit_data.encoder[MOTOR_ID_A] != OverSwitch);
    noinit_data.encoder[MOTOR_ID_A] = 0;  
    motor_state[MOTOR_ID_A].target_encoder  = 0;
    motor_state[MOTOR_ID_A].logic = 1;
    // config_set_motor_orientation(MOTOR_ID_A, motor_state[MOTOR_ID_A].orientation);
    config_set_gripper_open_encoder(140);
    config_set_gripper_close_encoder(310);
    Serial.println("Done");
  }
  return true;
}

static void motor_track_report(motor_id_t motor_id) 
{
  if (tracking > 0) {
    int position = motor_get_encoder(motor_id);
    if (tracked[motor_id] != position) {
      log_write(F("@"));
      if (tracking == 1) {
        log_write(F("%c:%d"), 'A' + motor_id, motor_get_encoder(motor_id));
      } else if (tracking == 2) {
        log_write(F("%c:%d"), 'A' + motor_id, motor_get_angle(motor_id));
      }    
      log_write(F(":HS%d:"), motor_state[motor_id].switch_previously_triggered);
      tracked[motor_id] = position;
    }
  }
}

static void calculate_mean_and_variance(float value, int nvalues, float *pM2, float *pmean, float *pvariance)
{
  // Welford's algorithm adapted from:
  // https://stackoverflow.com/questions/17052395/calculate-the-running-standard-deviation/17053010
  float delta = value - *pmean;
  *pmean += delta / nvalues;
  *pM2 += delta * (value - *pmean);
  *pvariance = *pM2 / nvalues;
}

// Track per-motor mean and variance for current
// Track per-motor mean and variance for qe-transitions/second

/*
 * CAL_LIMIT_FORWARD
 * CAL_LIMIT_BACKWARD
 * CAL_SWITCH_FORWARD
 * CAL_SWITCH_BACKWARD
 * CAL_DONE
 */

static cal_state_t get_cal_state(cal_data_t *pcal_data, bool forward, bool switch_on, bool stuck)
{
  return CAL_STATE_INIT;
}

static bool is_stuck(cal_data_t *pcal_data)
{
  bool ret = false;

  const int stuck_check_interval_ms = 500;
  const int stuck_check_encoder_count = 5;

  int encoder = noinit_data.encoder[pcal_data->motor_id];
  int ms = millis();

  if ((ms - pcal_data->stuck_check_start_ms >= stuck_check_interval_ms)) {
    // log_writeln(F("ms:%d, encoder:%d, stuck_check_start_encoder:%d"), ms, encoder, pcal_data->stuck_check_start_encoder);
    if (abs(encoder - pcal_data->stuck_check_start_encoder) <= stuck_check_encoder_count) {
      LOG_DEBUG(F("stuck"));
      ret = true;
    }

    pcal_data->stuck_check_start_encoder = encoder;
    pcal_data->stuck_check_start_ms = ms;
  }

  return ret;
}

static void set_zero_position(motor_id_t motor_id) 
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  noinit_data.encoder[motor_id] = 0;
  motor_state[motor_id].target_encoder = 0;
  log_writeln(F("Motor %c position set to zero."), motor_id);
}

static bool calibrate_found_encoder_extents(cal_data_t *pcal_data) {
  assert(pcal_data);
  return pcal_data->found_min_encoder && pcal_data->found_max_encoder;
}

static bool calibrate(cal_data_t *pcal_data) {
  assert(pcal_data);
  // Returns true while calibration is running, false when complete.
  
  static cal_state_t prev_state = CAL_STATE_INIT;
  if (prev_state != pcal_data->state) {
    log_writeln(F("%s"), cal_state_name_by_index[pcal_data->state]);
    prev_state = pcal_data->state;
  }

  motor_id_t motor_id = pcal_data->motor_id;
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  if (!config.motor[motor_id].configured) {
    pcal_data->error = CAL_ERROR_NOT_CONFIGURED;
    return false;
  }

  if (motor_state[motor_id].error_flags != 0) {
    LOG_ERROR(F("Motor %c signaled error during calibration"), motor_id);
    motor_log_errors(motor_id);
    pcal_data->error = CAL_ERROR_MOTOR_ERROR;
    return false;
  }

  int encoder = noinit_data.encoder[motor_id];

  switch (pcal_data->state) {
    case CAL_STATE_INIT:
      pcal_data->state = CAL_STATE_SEARCH;
      pcal_data->delta = +10000;
      pcal_data->found_min_encoder = false;
      pcal_data->found_max_encoder = false;
      pcal_data->min_encoder = 0;
      pcal_data->max_encoder = 0;
      pcal_data->switch_triggered = false;

      pcal_data->stuck_check_start_encoder = noinit_data.encoder[pcal_data->motor_id];
      pcal_data->stuck_check_start_ms = millis();
      pcal_data->target = encoder + pcal_data->delta;
      motor_set_target_encoder(motor_id, pcal_data->target);      
      break;
    case CAL_STATE_SEARCH:
      if (encoder < pcal_data->min_encoder) {
        if (pcal_data->found_min_encoder) {
          LOG_ERROR(F("Unexpected minimum encoder %d lower than previous minimum encoder %d"), encoder, pcal_data->min_encoder);
          pcal_data->error = CAL_ERROR_UNEXPECTED_MIN_ENCODER;
        }
        pcal_data->min_encoder = encoder;
      }

      if (encoder > pcal_data->max_encoder) {
        if (pcal_data->found_max_encoder) {
          LOG_ERROR(F("Unexpected maximum encoder %d higher than previous maximum encoder %d"), encoder, pcal_data->max_encoder);
          pcal_data->error = CAL_ERROR_UNEXPECTED_MAX_ENCODER;
        }
        pcal_data->max_encoder = encoder;
      }

#if 0
      if (is_stuck(pcal_data)) {
        if (pcal_data->delta > 0) {
          log_writeln(F("Motor %c found maximum encoder %d"), 'A' + pcal_data->motor_id, encoder);
          pcal_data->found_max_encoder = true;
        } else {
          log_writeln(F("Motor %c found minimum encoder %d"), 'A' + pcal_data->motor_id, encoder);
          pcal_data->found_min_encoder = true;
        }
      } 
#endif

      if (pcal_data->switch_triggered != motor_state[motor_id].switch_previously_triggered) {
        pcal_data->state = get_cal_state(pcal_data, pcal_data->delta > 0, pcal_data->switch_triggered, false);
        pcal_data->switch_triggered = motor_state[motor_id].switch_previously_triggered;

        log_writeln(F("Motor %c switch %d at encoder %d"), 'A' + motor_id, pcal_data->switch_triggered, encoder);
#if FALSE
        if (pcal_data->delta > 0) {
          if (pcal_data->switch_triggered) {
            pcal_data->state = CAL_STATE_SWITCH_FORWARD_ON;
          } else {
            pcal_data->state = CAL_STATE_SWITCH_FORWARD_OFF;
          }
        } else {
          if (pcal_data->switch_triggered) {
            pcal_data->state = CAL_STATE_SWITCH_REVERSE_ON;
          } else {
            pcal_data->state = CAL_STATE_SWITCH_REVERSE_OFF;
          }
        }
#endif
      }

      if ((pcal_data->delta < 0) && 
          ((encoder <= pcal_data->target) || 
              pcal_data->found_min_encoder)) {
        // Reached encoder target in negative direction.
        if (!pcal_data->found_max_encoder) {
          pcal_data->delta *= -1;
          pcal_data->target = pcal_data->max_encoder + pcal_data->delta;                      
        } else {
          pcal_data->target = encoder + pcal_data->delta;                      
        }
        LOG_DEBUG(F("target: %d"), pcal_data->target);
        motor_set_target_encoder(motor_id, pcal_data->target);
      } else if ((pcal_data->delta > 0) && 
          ((encoder >= pcal_data->target) || 
              pcal_data->found_max_encoder)) {
        // Reached encoder target in positive direction.
        if (!pcal_data->found_min_encoder) {
          pcal_data->delta = -pcal_data->delta;
          pcal_data->target = pcal_data->min_encoder + pcal_data->delta;                      
        } else {
          pcal_data->target = pcal_data->target + pcal_data->delta;                      
        }
        LOG_DEBUG(F("target: %d"), pcal_data->target);
        motor_set_target_encoder(motor_id, pcal_data->target);
      }

      if (calibrate_found_encoder_extents(pcal_data)) 
        pcal_data->state = CAL_STATE_DONE;
        
      break;
    case CAL_STATE_SWITCH_FORWARD_ON:
      // Move forward at full speed until off.     
      return false;
      break;
    case CAL_STATE_SWITCH_FORWARD_OFF:    
      // Forward a bit, then reverse.
      return false;
      break;
    case CAL_STATE_SWITCH_REVERSE_ON:
      // Reverse until off.
      return false;
      break;
    case CAL_STATE_SWITCH_REVERSE_OFF:    
      return false;
      break;
    case CAL_STATE_DONE:
      log_writeln(F("Motor %c calibration complete."), 'A' + pcal_data->motor_id);
      return false;
      break;
    default:
      assert(false);
      return true;
  }

  return true;

  // if (motor_id == MOTOR_ID_A)
  //  return motor_interrogate_limit_switch_a();

  bool encoder_min_found = false;
  bool encoder_max_found = false;
  int encoder_min = noinit_data.encoder[motor_id];
  int encoder_max = encoder_min;
  const int delta = 100;
  int encoder_target = noinit_data.encoder[motor_id] + delta;

  int motor_current_draw = 0;
  int motor_current_draw_nvalues = 0;
  float motor_current_draw_M2 = 0;
  float motor_current_draw_mean = 0;
  float motor_current_draw_variance = 0;

#if 0
  float encoders_per_second_M2 = 0;
  float encoders_per_second_mean = 0;
  float encoders_per_second_variance = 0;
#endif

  int start_time_millis = millis();
  int start_encoder = noinit_data.encoder[motor_id];

  bool switch_triggered = motor_state[motor_id].switch_previously_triggered;

  log_writeln(F("Motor %c encoder start:%d, switch_triggered=%d"), 
      'A' + motor_id, noinit_data.encoder[motor_id], switch_triggered);

#if 0
  // Find the switch and limits.
  do {
    int encoder = noinit_data.encoder[motor_id];

    encoder_min = encoder < encoder_min ? encoder: encoder_min;
    encoder_max = encoder > encoder_max ? encoder : encoder_max;

    if ((encoder_delta < 0) && ((encoder <= encoder_target) || encoder_min_found)) {
      // Reached encoder target in negative direction.
      if (!encoder_max_found) {
        encoder_delta = -encoder_delta;
        encoder_target = encoder_max + encoder_delta;                      
      } else {
        encoder_target = encoder + encoder_delta;                      
      }
    } else if ((encoder_delta > 0) && ((encoder >= encoder_target) || encoder_max_found)) {
      // Reached encoder target in positive direction.
      if (!encoder_min_found) {
        encoder_delta = -encoder_delta;
        encoder_target = encoder_min + encoder_delta;                      
      } else {
        encoder_target = encoder + encoder_delta;                      
      }
    }

    int ms = millis();
    if (ms - start_time_millis > 1000) {
      if (abs(encoder - start_encoder) < 5) {
        /*
        if (encoder_delta < 0) {
          encoder_min_found = true;
          log_writeln(F("Motor %c minimum found at position %d"), 'A' + motor_id, encoder);
        } else if (encoder_delta > 0) {
          encoder_max_found = true;
          log_writeln(F("Motor %c maximum found at position %d"), 'A' + motor_id, encoder);
        }
        */
      }

      start_encoder = encoder;
      start_time_millis = ms;
    }
    // TODO: Deal with apparent PWM overflow when calibrating motor B.
    // TODO: Find switch centers, other switch values.
    // TODO: Zero motor angles to switch centers.
    // TODO: Make robust for A-motor when fixing fasterner is loose.
    // TODO: Fail calibration if switch not found.
    // TODO: Consider checkpointing?

    if (switch_triggered != motor_state[motor_id].switch_previously_triggered) {
      switch_triggered = motor_state[motor_id].switch_previously_triggered;
      log_writeln(F("Motor %c switch %d at encoder %d"), 'A' + motor_id, switch_triggered, encoder);
    }

#if 0
    if (motor_state[motor_id].current_draw != motor_current_draw) {
      motor_current_draw = motor_state[motor_id].current_draw;
      // log_writeln(F("current: %d"), motor_current_draw);
    }
    
    int motor_current_draw = motor_get_current_draw(motor_id); // motor_state[motor_id].current;
    calculate_mean_and_variance(motor_current_draw, ++motor_current_draw_nvalues, 
        &motor_current_draw_M2, &motor_current_draw_mean, &motor_current_draw_variance);

    char motor_current_draw_mean_str[15] = {};
    dtostrf(motor_current_draw_mean, 3, 2, motor_current_draw_mean_str);
    char motor_current_draw_variance_str[15] = {};
    dtostrf(motor_current_draw_variance, 3, 2, motor_current_draw_variance_str);
    // log_writeln(F("current:%d, mean:%s, variance:%s"), motor_current_draw, motor_current_draw_mean_str, motor_current_draw_variance_str);

    int mcmul = (int)(motor_current_draw_mean * 2.5);
    if ((motor_current_draw_nvalues > 100) && (motor_current_draw > mcmul)) {
      log_writeln(F("high motor_current mean = %s, (%d > %d)"), motor_current_draw_mean_str,
          motor_current_draw, mcmul);
    }
#endif

    motor_set_target_encoder(motor_id, encoder_target);
  } while (!(encoder_min_found && encoder_max_found)); // (!motor_state[motor_id].switch_previously_triggered);

  log_writeln(F("Motor %c encoder_min=%d, encoder_max=%d"), 'A' + motor_id, encoder_min, encoder_max);
#endif

  return false;  // Calibration complete.

#if 0

  int r = 0;
  int f = 0;

  if (motor_state[motor_id].switch_previously_triggered) {
    //Serial.print(" Centering Motor ");
    //Serial.println(char(m+65));
    //Serial.print("   Moving to: ");
    noinit_data.encoder[motor_id] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
    motor_state[motor_id].target_encoder = 0;
    
    // Move to one side of switch and wait for the switch to be unpressed.
    log_write(F("  "));
    motor_set_target_encoder(motor_id, r - 130);
    do { 
      motor_track_report(motor_id); 
    } while (motor_state[motor_id].switch_previously_triggered);

    // Move to the other side of switch and wait for the switch to be pressed and then unpressed.
    log_write(F("  "));
    motor_set_target_encoder(motor_id, f + 130);
    do {
      motor_track_report(motor_id);
    } while (!motor_state[motor_id].switch_previously_triggered);

    do {
      motor_track_report(motor_id);
    } while (motor_state[motor_id].switch_previously_triggered);

    //do {motor_track_report(m);} while (motor_get_encoder(m) < f);

    // Move back to first side of switch and wait for the switch to be pressed and then unpressed.        
    log_write(F("  "));
    motor_set_target_encoder(motor_id, r - 130);
    do {
      motor_track_report(motor_id);
    } while (!motor_state[motor_id].switch_previously_triggered);

    do {
      motor_track_report(motor_id);
    } while (motor_state[motor_id].switch_previously_triggered);
    //do {motor_track_report(m);} while (motor_get_encoder(m) > r);

    // Calculate center of switches and then move to that place.
    int center_encoder = 
        ((motor_state[motor_id].switch_forward_off + motor_state[motor_id].switch_reverse_on + 
          motor_state[motor_id].switch_forward_on + motor_state[motor_id].switch_reverse_off) / 4);
    log_writeln(F("   Switch positions: %d, %d, %d, %d. Centering at position %d"),
        motor_state[motor_id].switch_reverse_off,
        motor_state[motor_id].switch_forward_on,
        motor_state[motor_id].switch_reverse_on,
        motor_state[motor_id].switch_forward_off,
        center_encoder);
    motor_set_target_encoder(motor_id, center_encoder);
    do { 
      track_report(motor_id); 
    } while (noinit_data.encoder[motor_id] != center_encoder);

    // Set Encoder and Target Values to 0.
    noinit_data.encoder[motor_id] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
    motor_state[motor_id].target_encoder = 0;
  } else {
    log_writeln(F(" Motor %c home switch not closed - skipping."), 'A' + motor_id);
  }
#endif
}

bool motor_calibrate(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));

  if (!config.motor[motor_id].configured) {
    log_writeln(F("ERROR: Motor %c not configured. Calibration failed."), 'A' + motor_id);
    return false;
  }

  cal_data_t cal_data = {};
  cal_data.motor_id = motor_id;
  log_writeln(F("Calibrating motor %c ..."), 'A' + motor_id);
  while (calibrate(&cal_data)) {};
  if (cal_data.error != CAL_ERROR_NONE) {
    log_writeln(F("calibration of motor %c failed with error %d."), 'A' + motor_id, cal_data.error);
    return false;
  }
    
  log_writeln(F("calibration of motor %c passed."), 'A' + motor_id);        
  return true;
}

bool motor_calibrate_all()
{
  bool ret = true;
  for (int i = MOTOR_ID_A; i <= MOTOR_ID_A; i++) {
    if (!config.motor[i].configured) {
      log_writeln(F("ERROR: Motor %c not configured. Skipping calibration."), 'A' + i);
      continue;
    }

    if (!motor_calibrate(i))
      ret = false;
  }
  return ret;
}

void motor_dump(motor_id_t motor_id) 
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  if (!motor_configured(motor_id))
    return;

  log_writeln(F("%c: encoder:%d qe_prev:%d, speed:%d target_speed:%d logic:%d prev_dir:%d pid_dvalue:%d pid_perror:%d target_encoder:%d current:%d progress:%d"),
    'A' + motor_id,
    noinit_data.encoder[motor_id],
    noinit_data.previous_quadrature_encoder[motor_id],  
    motor_state[motor_id].speed,
    motor_state[motor_id].target_speed,
    motor_state[motor_id].logic,
    motor_state[motor_id].previous_direction,
    motor_state[motor_id].pid_dvalue,
    motor_state[motor_id].pid_perror,
    motor_state[motor_id].target_encoder,
    motor_state[motor_id].current_draw,
    motor_state[motor_id].progress);
}

void motor_log_errors(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  int ef = motor_state[motor_id].error_flags;
  if (ef == 0)
    return;

  if (ef & MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED)
    log_writeln(F("Motor %c thermal overload detected."), 'A' + motor_id);
  if (ef & MOTOR_ERROR_FLAG_OVERCURRENT_DETECTED)
    log_writeln(F("Motor %c overcurrent detected."), 'A' + motor_id);
  if (ef & MOTOR_ERROR_FLAG_INVALID_ENCODER_TRANSITION)
    log_writeln(F("Motor %c invalid quadrature encoder transition."), 'A' + motor_id);
  if (ef & MOTOR_ERROR_FLAG_OPPOSITE_DIRECTION)
    log_writeln(F("Motor %c direction opposite of expectation."), 'A' + motor_id);
  if (ef & MOTOR_ERROR_FLAG_UNEXPECTED_SWITCH_ENCODER)
    log_writeln(F("Motor %c unexpected switch encoder."), 'A' + motor_id);
  if (ef & MOTOR_ERROR_FLAG_ENCODER_OVERFLOW)
    log_writeln(F("Motor %c encoder overflow."), 'A' + motor_id);
  if (ef & MOTOR_ERROR_FLAG_ENCODER_UNDERFLOW)
    log_writeln(F("Motor %c encoder underflow."), 'A' + motor_id);
}

static void isr_blink_led(bool pid_enabled) 
{
  static int led_counter = 0;
  led_counter++; 

  if (pid_enabled) {
    if (!hardware_get_led())
      led_counter += 4;  // if the PID is on, then blink faster.
    led_counter++; // Count the Interrupts   
  }

  if (led_counter > 1000) {
    bool led_state = !hardware_get_led();
    hardware_set_led(led_state);
    hardware_set_speaker(led_state);
    led_counter = 0;
  }
}

// Interrupt routine that interrupts the main program at freq of 2kHz.
ISR(TIMER1_COMPA_vect) 
{
  static int tb = 0;
  static int tc = 0;
  static int td = 0;
  static int te = 0;
  static int tf = 0;
  static motor_id_t motor_id = 0;

  const int max_error = 255 - motor_min_pwm;
  const int min_error = -(255 - motor_min_pwm);
  const int qe_inc_states[] = {1, 3, 0, 2};  // 01 -> 11 -> 00 -> 10.
  const int qe_dec_states[] = {2, 0, 3, 1};  // 10 -> 00 -> 11 -> 01.

  for (motor_id_t qe_motor_id = MOTOR_ID_FIRST; qe_motor_id <= MOTOR_ID_LAST; qe_motor_id = motor_id_t(qe_motor_id + 1)) {
    // Quadrature Encoders - read at rate of 2kHz.
    int qe_value_a = digitalRead(motor_pinout[qe_motor_id].in_quadrature_encoder_a);
    int qe_value_b = digitalRead(motor_pinout[qe_motor_id].in_quadrature_encoder_b);
    int qe_state = qe_value_a + (qe_value_b << 1);  // 0-3.

    if (qe_state == noinit_data.previous_quadrature_encoder[qe_motor_id]) {
      // Same state as last time through loop.
      motor_state[qe_motor_id].pid_dvalue++;
      if (motor_state[qe_motor_id].pid_dvalue > 10000) {
        motor_state[qe_motor_id].pid_dvalue = 10000;
      }
    } else if (qe_state == qe_inc_states[noinit_data.previous_quadrature_encoder[qe_motor_id]]) {
      // Quadrature encoder reading indicates moving in positive direction.
      motor_state[qe_motor_id].previous_direction = 1;
      if (noinit_data.encoder[qe_motor_id] == INT_MAX) {
        motor_state[qe_motor_id].error_flags |= MOTOR_ERROR_FLAG_ENCODER_OVERFLOW;
      }
      noinit_data.encoder[qe_motor_id]++;  // Wraps from 65535 to 0.
      motor_state[qe_motor_id].pid_dvalue = 0;
    } else if (qe_state == qe_dec_states[noinit_data.previous_quadrature_encoder[qe_motor_id]]) {
      // Quadrature encoder reading indicates moving in negative direction.
      motor_state[qe_motor_id].previous_direction = -1;
      if (noinit_data.encoder[qe_motor_id] == INT_MIN) {
        motor_state[qe_motor_id].error_flags |= MOTOR_ERROR_FLAG_ENCODER_UNDERFLOW;
      }
      noinit_data.encoder[qe_motor_id]--;  // Wraps from 0 to 65535.
      motor_state[qe_motor_id].pid_dvalue = 0;
    } else {
      motor_state[qe_motor_id].error_flags |= MOTOR_ERROR_FLAG_INVALID_ENCODER_TRANSITION;
    }    
    noinit_data.previous_quadrature_encoder[qe_motor_id] = qe_state;

#if 1  // Switch logic.
    /* 
     * See if Limit/Home switch has changed from on to off or off to on.
     * There are 4 different motor positions stored for the Home Switches.
     * -The On and Off locations when the motor is moving forward.
     * -The On and Off locations when the motor is moving reverse.
     * The average value of these 4 positions is used as the center of "Home".
     */
    bool switch_triggered = !digitalRead(motor_pinout[qe_motor_id].in_switch);
    if (motor_state[qe_motor_id].switch_triggered == switch_triggered) {
      if (switch_triggered != motor_state[qe_motor_id].switch_previously_triggered) {
        motor_state[qe_motor_id].switch_previously_triggered = switch_triggered;
        if (motor_state[qe_motor_id].previous_direction == 1) {
          if (!switch_triggered) {
            motor_state[qe_motor_id].switch_forward_on = noinit_data.encoder[qe_motor_id]; 
          } else {
            motor_state[qe_motor_id].switch_forward_off = noinit_data.encoder[qe_motor_id]; 
          }        
        } else if (motor_state[qe_motor_id].previous_direction == -1) {
          if (!switch_triggered) {
            motor_state[qe_motor_id].switch_reverse_on = noinit_data.encoder[qe_motor_id];
          } else {
            motor_state[qe_motor_id].switch_reverse_off = noinit_data.encoder[qe_motor_id];
          }                
        }      
      }    
    }
    motor_state[qe_motor_id].switch_triggered = switch_triggered;
#endif

    if (get_thermal_overload_active(qe_motor_id)) {
      motor_state[qe_motor_id].error_flags |= MOTOR_ERROR_FLAG_THERMAL_OVERLOAD_DETECTED;
    } 

    if (get_overcurrent_active(qe_motor_id)) {
      motor_state[qe_motor_id].error_flags |= MOTOR_ERROR_FLAG_OVERCURRENT_DETECTED;
    }     
  }

  isr_blink_led(motor_get_pid_enable(MOTOR_ID_A));

  //==========================================================
  // Calculate Motor status values.
  //==========================================================
  // Calculate only one motor per interupt - rate of 333Hz
  // This is done by stepping motor_id once per interupt. 
  // motor_id is then used to specifiy which motor to do 
  // calculations on.
  //==========================================================
  motor_id = motor_id_t(motor_id + 1); 
  if (motor_id > MOTOR_ID_LAST) 
    motor_id = MOTOR_ID_FIRST;

  //==========================================================
  // See if the Motor PID needs to be turned on.
  if (Motor_PID[motor_id] == 1230) {
    // The IntMotor PID is NOT running.
    // ( =1230 instead of =0 is just to kill the routine for now because it isn't doing what it needs to do.)
    motor_state[motor_id].pid_dvalue = 0;  // Clear the PID DValue for this motor.
    int intMDiff = abs(motor_state[motor_id].target_encoder - noinit_data.encoder[motor_id]);
    if (intMDiff > 3)
      Motor_PID[motor_id] = 1;  // Turn on the PID for this Motor.
  } else {
    // Brakes are not on - so the IntMotor is running.
    //==========================================================
    // Check for stall.
    // Note that the Gripper is excluded because it needs to 
    // apply tension on whatever it is gripping.
    //==========================================================

#if 1
    if (motor_id > MOTOR_ID_A) {    
      // For Motors other than the gripper, High Current means 
      // that the motor is in a stall situation.  To unstall,
      // the target position is set back a bit from the
      // current position.      
      
#if 0
      if (motor_state[motor_id].current > 200) {
        if (motor_state[motor_id].previous_direction == 1) {
          motor_state[motor_id].target_encoder = noinit_data.encoder[motor_id] - 50;
          motor_state[motor_id].current = 0;  // TODO: Why is this zero?
        } else if (motor_state[motor_id].previous_direction == -1) {
          motor_state[motor_id].target_encoder = noinit_data.encoder[motor_id] + 50;
          motor_state[motor_id].current = 0;  // TODO: Why is this zero?
        }
      }
#endif
    } else {
      if (motor_state[motor_id].current_draw > 100) {
        // Motor A is a special case where High Current 
        // means that the Gripper is gripping someting.
        // Set gripper tension on MotorA by setting
        // the Target Position to the Currernt Position.
        // This will cause the PWM to drop to off.
        //   AND if the relaxed gripper opens a little,
        //     it will turn back on but at a much lower
        //       PWM duty cycle.
        Gripper_StallC = motor_state[motor_id].current_draw;
        Gripper_StallE = noinit_data.encoder[motor_id];
        Gripper_StallX++;
      }
    }
#endif
    //==========================================================
    // Calculate PID Proportional Error
    //==========================================================
    int PIDPError = (motor_state[motor_id].target_encoder - noinit_data.encoder[motor_id]);

#if 0
    // TODO: Make this work.
    if ((motor_state[motor_id].pid_perror != 0) &&
        (abs(PIDPError) > abs(motor_state[motor_id].pid_perror))) {
      LOG_DEBUG(F("%d %d"), PIDPError, motor_state[motor_id].pid_perror);
      // Motor is getting further away from target, not closer.
      motor_state[motor_id].error_flags |= MOTOR_ERROR_FLAG_OPPOSITE_DIRECTION;
    }
#endif
    motor_state[motor_id].pid_perror = PIDPError;  // Save.
      
    //==========================================================
    // Calc the Target Speed from the Proportional Error
    // The target speed is just the differnce between the 
    //  Current Position and the Target Position (with limits).
    // Results in a speed of +/- 255.
    //==========================================================
    if (PIDPError > max_error) {
      motor_state[motor_id].target_speed = max_error;
      // Set the Status that indicates that the Motor is more than 200 clicks from target.
      motor_state[motor_id].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET;      
    } else if (PIDPError < min_error) {
      motor_state[motor_id].target_speed = min_error;
      // Set the Status that indicates that the Motor is more than 200 clicks from target.
      motor_state[motor_id].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET;      
    } else if (PIDPError > 0) {  // TODO: Refactor to combine PIDPerror > 0 and < 0 cases.
      motor_state[motor_id].target_speed = motor_state[motor_id].pid_perror + (motor_state[motor_id].pid_dvalue / 6);
      if (PIDPError < 2) {
        // Set the Status that indicates that the Motor is 1 click from target
        motor_state[motor_id].progress = MOTOR_PROGRESS_BESIDE_TARGET;  
      } else if (PIDPError < 30) {
        // Set the Status that indicates that the Motor is 2-29 clicks from target
        motor_state[motor_id].progress = MOTOR_PROGRESS_NEAR_TARGET;
      } else {
        // Set the Status that indicates that the Motor is 30-200 clicks from target
        motor_state[motor_id].progress = MOTOR_PROGRESS_APPROACHING_TARGET;
      }
    } else if (PIDPError < 0) {
      motor_state[motor_id].target_speed = motor_state[motor_id].pid_perror - (motor_state[motor_id].pid_dvalue / 6), -255; // TODO: Check.
      if (PIDPError > -2) {
        // Set the Status that indicates that the Motor is 1 click from target
        motor_state[motor_id].progress = MOTOR_PROGRESS_BESIDE_TARGET;  
      } else if (PIDPError > -30) {
        // Set the Status that indicates that the Motor is 2-29 clicks from target
        motor_state[motor_id].progress = MOTOR_PROGRESS_NEAR_TARGET;
      } else {
        // Set the Status that indicates that the Motor is 30-200 clicks from target
        motor_state[motor_id].progress = MOTOR_PROGRESS_APPROACHING_TARGET;
      }
    } else {
      motor_state[motor_id].target_speed = 0; 
      motor_state[motor_id].progress = MOTOR_PROGRESS_AT_TARGET;  // Clear the flag that indicates that the Motor is in motion.
      // Motor_PID[motor_id] = 0;  // Turn off motor_id's PID.
    }
  
    //==========================================================
    // PID (Currenty Just the P)
    //==========================================================
    if (motor_state[motor_id].pid_enabled) {
      //============================================
      // Ramp Up/Down Current Speed to Target Speed.
      // Prevents the motors from jumping from dead 
      //  stop to full speed and vice-versa
      //============================================
      int CurrentSpeedChanged = 0;
      if (motor_state[motor_id].target_speed > motor_state[motor_id].speed) {
        if (motor_state[motor_id].speed < (255 - motor_min_pwm)) {
          motor_state[motor_id].speed++; // if the target is higher, then inc up to the target.
          if (motor_state[motor_id].target_speed > motor_state[motor_id].speed) {
            if (motor_state[motor_id].speed < (255 - motor_min_pwm)) {
              motor_state[motor_id].speed++; // if the target is higher, then inc up to the target a second time.
            }
          }
          CurrentSpeedChanged = 1;
        }
      } else if (motor_state[motor_id].target_speed < motor_state[motor_id].speed) {
        if (motor_state[motor_id].speed > -(255 - motor_min_pwm)) {
          motor_state[motor_id].speed--; // if the target is lower, then inc down to the target.
          if (motor_state[motor_id].target_speed < motor_state[motor_id].speed) {
            if (motor_state[motor_id].speed > -(255 - motor_min_pwm)) {
              motor_state[motor_id].speed--; // if the target is lower, then inc down to the target a second time.
            }
          }
          CurrentSpeedChanged = 1;
        }
      }
  
      if (CurrentSpeedChanged == 1) {
        motor_set_speed(motor_id, motor_state[motor_id].speed);    
      }        
    }
  
  #if 0
    //==========================================================
    // Sync Move.
    // as it stands, the synchronized move does not work well
    // with the current PID.  
    // The current PID only regulates encoder position.
    // For the Sync Move to work well, it needs to regulate speed.
    //==========================================================
    static int TravelSoFar = 0;
    static int TravelSoFarPrev = 0;

    if (motor_sync_move_enabled > 0) {    
      TravelSoFar =abs(noinit_data.encoder[LeadMotor] - Start[LeadMotor]+1);
      if (TravelSoFar != TravelSoFarPrev) {
        TravelSoFarPrev = TravelSoFar;
        float TravelSoFarFloat = TravelSoFar;
        for (int sync_motor_id = MOTOR_ID_B; sync_motor_id <= MOTOR_ID_LAST; sync_motor_id++) {
          if (sync_motor_id != LeadMotor) {
            float RP = TravelSoFarFloat*Ratio[sync_motor_id];
            int RI = int(RP);
            int TG = Start[sync_motor_id]+RI;
            motor_state[sync_motor_id].target_encoder  = Start[sync_motor_id]+RI;
          }
        } 
        tb = abs(End[MOTOR_ID_B] - noinit_data.encoder[MOTOR_ID_B]);
        tc = abs(End[MOTOR_ID_C] - noinit_data.encoder[MOTOR_ID_C]);
        td = abs(End[MOTOR_ID_D] - noinit_data.encoder[MOTOR_ID_D]);
        te = abs(End[MOTOR_ID_E] - noinit_data.encoder[MOTOR_ID_E]);
        tf = abs(End[MOTOR_ID_F] - noinit_data.encoder[MOTOR_ID_F]);            
        if ((tb == 0) && (tc == 0) && (td == 0) && (te == 0) && (tf == 0)) {
          // Set the Status that indicates that the Motor is 1 click from target
          SyncMove_Status = MOTOR_PROGRESS_AT_TARGET;
          motor_sync_move_enabled = false;
        } else if ((tb<2) && (tc<2) && (td<2) && (te<2) && (tf<2)) {
          SyncMove_Status = MOTOR_PROGRESS_BESIDE_TARGET;
        } else if ((tb<30) && (tc<30) && (td<30) && (te<30) && (tf<30)) {
          // Set the Status that indicates that the Motor is 2-29 clicks from target
          SyncMove_Status = MOTOR_PROGRESS_NEAR_TARGET;
        } else if ((tb<200) && (tc<200) && (td<200) && (te<200) && (tf<200)) {
          // Set the Status that indicates that the Motor is 2-29 clicks from target
          SyncMove_Status = MOTOR_PROGRESS_APPROACHING_TARGET;
        } else {
          // Set the Status that indicates that the Motor is 30-200 clicks from target
          SyncMove_Status = MOTOR_PROGRESS_ON_WAY_TO_TARGET;
        }        
      }
    }
#endif
  }
}