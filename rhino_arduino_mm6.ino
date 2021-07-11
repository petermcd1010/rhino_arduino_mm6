/*
 * Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories
 * Written by Scott Savage, Peter McDermott
 * Feb 2017-21 GNU General Public License (GPL)
 *
 * Software change log at the end of file.
 *
 * See also:
 *   https://www.ti.com/lit/gpn/lmd18200 for the MegaMotor6's motor drivers.
 *
 * TODO:
 *   Analyze for JPL/etc. C coding rules.
 *   Run through a static analyzer?
 *   Fix all TODOs.
 */

#define __ASSERT_USE_STDERR
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <EEPROM.h>

#include "log.h"

static const float software_version = 2.00;  
static const char* const psoftware_date = "June 30, 2021";

// Used to select per-robot config information.
typedef enum {
  ROBOT_ID_FIRST = 0,
  ROBOT_ID_UNKNOWN = ROBOT_ID_FIRST,
  ROBOT_ID_RHINO_XR_1,
  ROBOT_ID_RHINO_XR_2,
  ROBOT_ID_RHINO_XR_3,
  ROBOT_ID_RHINO_XR_4,
  ROBOT_ID_RHINO_SCARA,
  ROBOT_ID_RHINO_LINEAR_SLIDE_TABLE,
  ROBOT_ID_RHINO_XY_SLIDE_TABLE,
  ROBOT_ID_RHINO_TILT_CAROUSEL,
  ROBOT_ID_RHINO_CONVEYOR_BELT,
  ROBOT_ID_COUNT,
  ROBOT_ID_LAST = ROBOT_ID_COUNT - 1,
  ROBOT_ID_DEFAULT = ROBOT_ID_RHINO_XR_2,
} robot_id_t;

static const char* const robot_name_by_robot_id[ROBOT_ID_COUNT] = { 
  "unknown robot id",
  "Rhino XR-1 6-axis arm",
  "Rhino XR-2 6-axis arm",
  "Rhino XR-3 6-axis arm",
  "Rhino XR-4 6-axis arm",
  "Rhino SCARA 5-axis arm",
  "Rhino linear slide table",
  "Rhino XY slide table",
  "Rhino tilt carousel",
  "Rhino conveyor belt"
};

// Motor IDs are used to select an element from the arrays that hold motor information.
typedef enum {
  MOTOR_ID_FIRST = 0,
  MOTOR_ID_A = MOTOR_ID_FIRST,
  MOTOR_ID_B,
  MOTOR_ID_C,
  MOTOR_ID_D,
  MOTOR_ID_E,
  MOTOR_ID_F,
  MOTOR_ID_COUNT,
  MOTOR_ID_LAST = MOTOR_ID_COUNT - 1
} motor_id_t;

// MM6 motor I/O lines.
typedef struct {
  unsigned short out_direction;  // Digital.
  unsigned short out_pwm;  // Digital.
  unsigned short out_brake;  // Digital.
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

static const int expansion_io_pinout[] = { A15, A14, A13, A12, 53, 49, 48, 41 };

// State for  motor.
typedef enum {
  MOTOR_PROGRESS_AT_TARGET = 0,
  MOTOR_PROGRESS_BESIDE_TARGET,  // Within 1 click.
  MOTOR_PROGRESS_NEAR_TARGET,  // between 2 and 30 clicks.
  MOTOR_PROGRESS_APPROACHING_TARGET,  // between 30 and 200 clicks.
  MOTOR_PROGRESS_ON_WAY_TO_TARGET, // More than 200 clicks away.
} motor_progress_t;

typedef enum {
  MOTOR_ERROR_NO_ERROR = 0,
  MOTOR_ERROR_INVALID_ENCODER_TRANSITION,  // only 0->1->3->2 and 0->2->3->1 are valid.
} motor_error_t;

typedef struct {
  int speed;
  int target_speed;
  int logic;
  int previous_direction;
  int pid_dvalue;
  int pid_perror;  // Proportional Error (Difference between Current and Target)
  int target_encoder;
  int current;  // Motor Current Consumption. TODO: units? counts?
  motor_progress_t progress;

  bool switch_previously_triggered;  // Alignment switch previous value used for debounce.
  bool switch_triggered;  // Alignment switch previous value used for debounce.
  int switch_forward_on;  // Alighment switch forward direction high value.
  int switch_forward_off;  // Alignment switch forward direction low value.
  int switch_reverse_on;  // Alignment switch reverse direction high value.
  int switch_reverse_off;  // Alignment switch reverse direction low value.

  motor_error_t motor_error;
} motor_state_t;

static motor_state_t motor_state[MOTOR_ID_COUNT] = {}; 

static const int motor_min_speed = 55;
static bool motor_sync_move_enabled = false;

static bool mm6_pid_enabled = false;
static const int OPRLED = 13;
static int Motor_PID[] = {0,0,0,0,0,0}; // PID on or off.

int End[] = {0,0,0,0,0,0};
int Gripper_StallC = 0;
int LeadMotor = 0;
int Start[] = {0,0,0,0,0,0};
float Ratio[] = {0,0,0,0,0,0};
int Gripper_StallE = 0;
int Gripper_StallX = 0;
int SyncMove_Status = 0;
int current_pwm[] = {0,0,0,0,0,0};
int Forward_Logic[] = {0,0,0,0,0,0}; // Forward Logic - The value for the Direction IO Line when the motor needs to move forward to sync with encoders.
int Reverse_Logic[] = {1,1,1,1,1,1}; // Reverse Logic - The value for the Direction IO Line when the motor needs to move Reverse to sync with encoders.

// Configuration stored in RAM and saved across reset/reboot that don't include a power-cycle of the board.
const int noinit_data_version = 2;
const int noinit_data_magic = 0xABCD1234;
typedef struct {
  // nbytes, version, magic are used to verify valid data.
  size_t nbytes;
  int version;
  int magic;

  int motor_encoder[MOTOR_ID_COUNT];  // Present Motor Encoder Positions.
  int motor_qe_prev[MOTOR_ID_COUNT];  // Quadrature Encoder Previous Read.
} noinit_data_t;

static noinit_data_t noinit_data __attribute__ ((section (".noinit")));  // NOT reset to 0 when the CPU is reset.

// Configuration stored in EEPROM.
static const int config_base_address = 4000;
static const int config_version = 1;
static const int config_magic = 0x5678FEAD;
static const int config_robot_serial_nbytes = 15;
static const int config_robot_name_nbytes = 20;

typedef struct __attribute__((packed)) {
  bool configured;
  int angle_offset;
  int motor_logic;
  int direction_logic;
} motor_config_t;

typedef struct __attribute__((packed)) {
  // nbytes, version, magic, and crc are used to verify a valid config.
  size_t nbytes;
  int version;
  long magic;
  long crc;

  robot_id_t robot_id;
  char robot_serial[config_robot_serial_nbytes];  // To help user confirm board/robot match.
  char robot_name[config_robot_name_nbytes];  // Optional robot name for user.
  motor_config_t motor[MOTOR_ID_COUNT];
  int gripper_open_location;
  int gripper_close_location;
} config_t;

config_t config = {};

typedef struct {
  char command_char;
  char *pname;
  void(*print_sub_menu_fn)();
  bool has_args;
  int (*pfunction)(char *payload, size_t nbytes);
  char *phelp;
} menu_item_t;

// Used in the state machine.
typedef enum {
  STATE_FIRST = 0,  // TODO: remove when config is crc'd?
  STATE_INIT = STATE_FIRST,
  STATE_MOTORS_OFF,
  STATE_MOTORS_ON,
  STATE_ERROR,
  STATE_COUNT,
  STATE_LAST = STATE_COUNT - 1,
} state_t;

static char* state_name_by_state[] = { 
  "init", 
  "motors_off", 
  "motors_on", 
  "error" 
};
static state_t state = STATE_INIT;

// Used for status messages.
typedef struct {
  float angle;
  float current_draw;
  bool switch_triggered;
  bool thermal_overload_active;
  bool overcurrent_active;
} motor_status_t;

typedef struct {
  state_t state;
  motor_status_t motor[MOTOR_ID_COUNT];
} status_t;

/*
 * MegaMotor6 functions.
 */

void mm6_init()
{
  // TODO: integrate with mm6_enable_motors.
  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    // Configure and initialize outputs.
    pinMode(motor_pinout[i].out_brake, OUTPUT);
    pinMode(motor_pinout[i].out_direction, OUTPUT);
    pinMode(motor_pinout[i].out_pwm, OUTPUT);
    digitalWrite(motor_pinout[i].out_direction, HIGH); //  Set direction to forward.

    // Configure inputs.
    pinMode(motor_pinout[i].in_switch, INPUT_PULLUP);
    pinMode(motor_pinout[i].in_thermal_overload, INPUT_PULLUP);
    pinMode(motor_pinout[i].in_quadrature_encoder_a, INPUT_PULLUP);
    pinMode(motor_pinout[i].in_quadrature_encoder_b, INPUT_PULLUP);
  }  
  
  mm6_enable_motors(false);
}

void mm6_enable_motors(bool enable)
{
  static bool mm6_enabled = false;
  if (mm6_enabled == enable)   
    return;

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    if (enable && config.motor[i].configured) {
      digitalWrite(motor_pinout[i].out_brake, LOW);  // Turn brakes off.
    } else {
      digitalWrite(motor_pinout[i].out_brake, HIGH);  // Enable brakes.
      digitalWrite(motor_pinout[i].out_pwm, LOW);  // Set PWM speed to zero.  TODO: analogWrite() below?
      analogWrite(motor_pinout[i].out_pwm, 0);  // Set PWM speed to 0.
    }
  }

  motor_sync_move_enabled = enable;
  mm6_pid_enabled = enable;  // TODO: What about mm6_pid_enable()?
  mm6_enabled = enable;
  log_writeln(F("All motor electronics %s."), mm6_enabled ? "enabled" : "disabled");
}

bool mm6_get_switch_triggered(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  return motor_state[motor_id].switch_triggered;
}

int mm6_get_position(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  return noinit_data.motor_encoder[motor_id] * motor_state[motor_id].logic;
}

void mm6_set_position(motor_id_t motor_id, int position) 
{  
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  // TODO: assert valid position?

  motor_state[motor_id].target_encoder = position * motor_state[motor_id].logic;
  motor_state[motor_id].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET; //Set the flag that indicates that the motor has been put in motion.
}

float mm6_get_angle(motor_id_t motor_id) 
{  
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  return (mm6_get_position(motor_id) - config.motor[motor_id].angle_offset) / EncoderStepsPerDegree(motor_id);
}

void mm6_set_angle(motor_id_t motor_id, float angle) 
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  // TODO: assert valid angle?
  int position = AngleToPosition(motor_id, angle);
  mm6_set_position(motor_id, position);
}

bool mm6_get_thermal_overload_active(motor_id_t motor_id)
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

bool mm6_get_overcurrent_active(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  return false;
}

float mm6_get_current_draw(motor_id_t motor_id)
{
  assert((motor_id >= MOTOR_ID_FIRST) && (motor_id <= MOTOR_ID_LAST));
  // LMD18200 datasheet says 377uA/A. What's the resistance?
  float f = analogRead(motor_pinout[motor_id].in_current_draw) * (5.0f / 1023.0f);
}

void isr_blink_led(bool pid_enabled) 
{
  static int led_counter = 0;
  led_counter++; 

  if (pid_enabled){
    // if the PID is on, then blink faster.
    if (!hardware_get_led())
      led_counter += 4;  // TODO: Why.
    led_counter++; // Count the Interrupts   
  }

  if (led_counter > 1000) {
    bool led_state = !hardware_get_led();
    hardware_set_led(led_state);
    digitalWrite(expansion_io_pinout[0], led_state);  // Also output the LED onto expansion_io_pinout 1 which can be wired to a speaker.
    led_counter = 0;
  }
}

// Interrupt routine that interrupts the main program at freq of 2kHz 
ISR(TIMER1_COMPA_vect) 
{
  static int tb = 0;
  static int tc = 0;
  static int td = 0;
  static int te = 0;
  static int tf = 0;
  static motor_id_t motor_id = 0;

  const int MaxError = 255 - motor_min_speed;
  const int MinError = -(255 - motor_min_speed);
  const int QE_Inc_States[] = {1, 3, 0, 2};
  const int QE_Dec_States[] = {2, 0, 3, 1};

  //==========================================
  // Quadrature Encoders - read at rate of 2kHz 
  //==========================================

  for (motor_id_t qe_motor_id = MOTOR_ID_A; qe_motor_id <= MOTOR_ID_LAST; qe_motor_id = (motor_id_t)((int)(qe_motor_id) + 1)){
    bool switch_triggered = !digitalRead(motor_pinout[qe_motor_id].in_switch);
    int QE_Value_A = digitalRead(motor_pinout[qe_motor_id].in_quadrature_encoder_a);
    int QE_Value_B = digitalRead(motor_pinout[qe_motor_id].in_quadrature_encoder_b) * 2;
    int QE_State = QE_Value_A + QE_Value_B;
    if (QE_State == QE_Inc_States[noinit_data.motor_qe_prev[qe_motor_id]]) {
      motor_state[qe_motor_id].previous_direction = 1;
      noinit_data.motor_encoder[qe_motor_id]++;
      motor_state[qe_motor_id].pid_dvalue = 0;
    } else if (QE_State == QE_Dec_States[noinit_data.motor_qe_prev[qe_motor_id]]) {
      motor_state[qe_motor_id].previous_direction = -1;
      noinit_data.motor_encoder[qe_motor_id]--;      
      motor_state[qe_motor_id].pid_dvalue = 0;
    } else {
      motor_state[qe_motor_id].pid_dvalue++;
      if (motor_state[qe_motor_id].pid_dvalue > 10000) {
        motor_state[qe_motor_id].pid_dvalue = 10000;
      }
    }    
    noinit_data.motor_qe_prev[qe_motor_id] = QE_State;

    //==========================================================
    // See if Limit/Home switch has changed from on to off or off to on.
    // There are 4 different motor positions stored for the Home Switches.
    // -The On and Off locations when the motor is moving forward.
    // -The On and Off locations when the motor is moving reverse.
    // The average value of these 4 positions is used as the center of "Home".
    //==========================================================
    if (motor_state[qe_motor_id].switch_triggered == switch_triggered) {
      if (switch_triggered != motor_state[qe_motor_id].switch_previously_triggered) {
        motor_state[qe_motor_id].switch_previously_triggered = switch_triggered;
        if (motor_state[qe_motor_id].previous_direction == 1) {
          if (!switch_triggered) {
            motor_state[qe_motor_id].switch_forward_on = noinit_data.motor_encoder[qe_motor_id]; 
          } else {
            motor_state[qe_motor_id].switch_forward_off = noinit_data.motor_encoder[qe_motor_id]; 
          }        
        } else if (motor_state[qe_motor_id].previous_direction == -1) {
          if (!switch_triggered) {
            motor_state[qe_motor_id].switch_reverse_on = noinit_data.motor_encoder[qe_motor_id];
          } else {
            motor_state[qe_motor_id].switch_reverse_off = noinit_data.motor_encoder[qe_motor_id];
          }                
        }      
      }    
    }
    motor_state[qe_motor_id].switch_triggered = switch_triggered;
  }

  isr_blink_led(mm6_pid_enabled);

  //==========================================================
  // Calculate Motor status values.
  //==========================================================
  // Calculate only one motor per interupt - rate of 333Hz
  // This is done by stepping motor_id once per interupt. 
  // motor_id is then used to specifiy which motor to do 
  // calculations on.
  //==========================================================
  motor_id = (motor_id_t)((int)motor_id + 1); 
  if (motor_id == MOTOR_ID_LAST) 
    motor_id = MOTOR_ID_FIRST;

  //==========================================================
  // See if the Motor PID needs to be turned on.
  if (Motor_PID[motor_id]==1230) {
    // The IntMotor PID is NOT running.
    // ( =1230 instead of =0 is just to kill the routine for now because it isn't doing what it needs to do.)
    motor_state[motor_id].pid_dvalue = 0; //Clear the PID DValue for this motor
    int intMDiff = abs(motor_state[motor_id].target_encoder - noinit_data.motor_encoder[motor_id]);
    if (intMDiff>3) {
      Motor_PID[motor_id] = 1; // Turn the PID On for this Motor.
    }
  } else {
    // Brakes are not on - so the IntMotor is running.
    //==========================================================
    // Check for stall.
    // Note that the Gripper is excluded because it needs to 
    // apply tension on whatever it is gripping.
    //==========================================================
    if (motor_id > MOTOR_ID_A){    
      // For Motors other than the gripper, High Current means 
      // that the motor is in a stall situation.  To unstall,
      // the target position is set back a bit from the
      // current position.      
      
      if (motor_state[motor_id].current > 200) {
        if (motor_state[motor_id].previous_direction == 1) {
          motor_state[motor_id].target_encoder = noinit_data.motor_encoder[motor_id] - 50;
          motor_state[motor_id].current = 0;  // TODO: Why is this zero?
        } else if (motor_state[motor_id].previous_direction == -1) {
          motor_state[motor_id].target_encoder = noinit_data.motor_encoder[motor_id] + 50;
          motor_state[motor_id].current = 0;  // TOOD: Why is this zero?
        }
      }
           
    } else {
      if (motor_state[motor_id].current > 100) {
        // Motor A is a special case where High Current 
        // means that the Gripper is gripping someting.
        // Set gripper tension on MotorA by setting
        // the Target Position to the Currernt Position.
        // This will cause the PWM to drop to off.
        //   AND if the relaxed gripper opens a little,
        //     it will turn back on but at a much lower
        //       PWM duty cycle.
        Gripper_StallC = motor_state[motor_id].current;
        Gripper_StallE = noinit_data.motor_encoder[motor_id];
        Gripper_StallX++;
      }
    }
  
    //==========================================================
    // Calculate PID Proportional Error
    //==========================================================
    int PIDPError = (motor_state[motor_id].target_encoder - noinit_data.motor_encoder[motor_id]);
    motor_state[motor_id].pid_perror = PIDPError; // Save
      
    //==========================================================
    // Calc the Target Speed from the Proportional Error
    // The target speed is just the differnce between the 
    //  Current Position and the Target Position (with limits).
    // Results in a speed of +/- 255.
    //==========================================================
    if (PIDPError > MaxError) {
      motor_state[motor_id].target_speed = MaxError;
      // Set the Status that indicates that the Motor is more than 200 clicks from target.
      motor_state[motor_id].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET;      
    } else if (PIDPError < MinError) {
      motor_state[motor_id].target_speed = MinError;
      // Set the Status that indicates that the Motor is more than 200 clicks from target.
      motor_state[motor_id].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET;      
    } else if (PIDPError > 0) {
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
      //Motor_PID[motor_id]=0; // Turn motor_id's PID off.      
    }
  
    //==========================================================
    // PID (Currenty Just the P)
    //==========================================================
    if (mm6_pid_enabled) {
      //============================================
      // Ramp Up/Down Current Speed to Target Speed.
      // Prevents the motors from jumping from dead 
      //  stop to full speed and vice-versa
      //============================================
      int CurrentSpeedChanged = 0;
      if (motor_state[motor_id].target_speed > motor_state[motor_id].speed) {
        if (motor_state[motor_id].speed < (255 - motor_min_speed)){
          motor_state[motor_id].speed++; // if the target is higher, then inc up to the target.
          if (motor_state[motor_id].target_speed > motor_state[motor_id].speed) {
            if (motor_state[motor_id].speed < (255 - motor_min_speed)){
              motor_state[motor_id].speed++; // if the target is higher, then inc up to the target a second time.
            }
          }
          CurrentSpeedChanged = 1;
        }
      } else if (motor_state[motor_id].target_speed < motor_state[motor_id].speed) {
        if (motor_state[motor_id].speed > -(255 - motor_min_speed)){
          motor_state[motor_id].speed--; // if the target is lower, then inc down to the target.
          if (motor_state[motor_id].target_speed < motor_state[motor_id].speed) {
            if (motor_state[motor_id].speed > -(255 - motor_min_speed)){
              motor_state[motor_id].speed--; // if the target is lower, then inc down to the target a second time.
            }
          }
          CurrentSpeedChanged = 1;
        }
      }
  
      if (CurrentSpeedChanged == 1){
        mm6_set_pwm(motor_id);    
      }        
    }
  
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
      TravelSoFar =abs(noinit_data.motor_encoder[LeadMotor] - Start[LeadMotor]+1);
      if (TravelSoFar != TravelSoFarPrev) {
        TravelSoFarPrev = TravelSoFar;
        float TravelSoFarFloat = TravelSoFar;
        for (int sync_motor_id = MOTOR_ID_B; sync_motor_id <= MOTOR_ID_LAST; sync_motor_id++){
          if (sync_motor_id != LeadMotor){
            float RP = TravelSoFarFloat*Ratio[sync_motor_id];
            int RI = int(RP);
            int TG = Start[sync_motor_id]+RI;
            motor_state[sync_motor_id].target_encoder  = Start[sync_motor_id]+RI;
          }
        } 
        tb = abs(End[MOTOR_ID_B] - noinit_data.motor_encoder[MOTOR_ID_B]);
        tc = abs(End[MOTOR_ID_C] - noinit_data.motor_encoder[MOTOR_ID_C]);
        td = abs(End[MOTOR_ID_D] - noinit_data.motor_encoder[MOTOR_ID_D]);
        te = abs(End[MOTOR_ID_E] - noinit_data.motor_encoder[MOTOR_ID_E]);
        tf = abs(End[MOTOR_ID_F] - noinit_data.motor_encoder[MOTOR_ID_F]);            
        if ((tb==0) && (tc==0) && (td==0) && (te==0) && (tf==0)) {
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
  }
}

void mm6_pid_enable(bool enable)
{
  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    if (enable && config.motor[i].configured) {
      digitalWrite(motor_pinout[i].out_brake, LOW);  // Turn off the brakes.
    } else {
      analogWrite(motor_pinout[i].out_pwm, 0);  // Set the speed to 0.
      digitalWrite(motor_pinout[i].out_brake, LOW);  // Turn on the brakes.
    }
  }
  
  mm6_pid_enabled = enable;
  motor_sync_move_enabled = enable;

  log_writeln(F("Motors are now %s."), enable ? "enabled" : "disabled");
}

void mm6_set_pwm(motor_id_t motor_id)
{
  //==========================================================
  // Calculate the PWM and Direction for the Speed
  // Converts the speed's +/- 255 value to PWM and Direction.
  //==========================================================  
  if (motor_state[motor_id].speed > 0) {      
    current_pwm[motor_id] = motor_state[motor_id].speed + motor_min_speed;
    analogWrite(motor_pinout[motor_id].out_pwm, current_pwm[motor_id]);  
    digitalWrite(motor_pinout[motor_id].out_direction, Forward_Logic[motor_id]);          
  } else if (motor_state[motor_id].speed < 0) {      
    current_pwm[motor_id] = abs(motor_state[motor_id].speed) + motor_min_speed;
    analogWrite(motor_pinout[motor_id].out_pwm, current_pwm[motor_id]);  
    digitalWrite(motor_pinout[motor_id].out_direction, Reverse_Logic[motor_id]);        
  } else {
    current_pwm[motor_id] = 0;
    analogWrite(motor_pinout[motor_id].out_pwm, 0);          
  }  
}

/*
 * Non-MegaMotor6-specific hardware functions.
 */

void hardware_erase_eeprom()
{
  log_write(F("Erasing EEPROM... "));
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);    
  }
  log_writeln(F("Completed. Zeroed %d bytes."), EEPROM.length());
}

void hardware_emergency_stop()
{
  // TODO: call hardware_emergency_stop when entering ERROR state.

  mm6_enable_motors(false);

  // Avoid extra log messages if emergency stop has already been executed.
  static bool stopped = false;
  if (stopped) {
    log_writeln(F("Emergency stop completed."));
    stopped = true;
  }

  state = STATE_ERROR;
}

void hardware_reset()
{
  hardware_emergency_stop();
  hardware_erase_eeprom();
  hardware_reboot();
}

void (*hardware_really_reboot)(void) = 0;  // Call hardware_really_reboot() to reset the board.
void hardware_reboot() 
{
  hardware_emergency_stop();
  delay(1000);  // Wait 1s for log output to complete writing.
  hardware_really_reboot();
}

void hardware_set_led(bool enable)
{
  digitalWrite(OPRLED, enable);
}

bool hardware_get_led()
{
  return digitalRead(OPRLED) != 0;
}

/*
 * Configuration functions.
 */

bool buffer_contains(char *pbuffer, size_t buffer_nbytes, char c)
{
  assert(pbuffer);

  for (int i = 0; i < buffer_nbytes; i++) {
    if (pbuffer[i] == c)
      return true;
  }

  return false;
}

/*
 * Parsing functions.
 */

size_t parse_whitespace(char *pbuf, size_t buf_nbytes)
{
  assert(pbuf);

  char *p = pbuf;
  while (isspace(*p))
    p++;

  return p - pbuf;
}

size_t parse_bool(char *pbuf, size_t buf_nbytes, bool *pout_bool)
{
  assert(pbuf);
  assert(pout_bool);

  const char* bool_strings[] = { 
    "on",
    "off",
    "true",
    "false",
    "1",
    "0",
  };
  #define BOOL_STRINGS_COUNT sizeof(bool_strings) / sizeof(bool_strings[0])

  int entry_num = -1;
  size_t nbytes = parse_string_in_table(pbuf, buf_nbytes, bool_strings, BOOL_STRINGS_COUNT, &entry_num);
  if (nbytes > 0) {
    if ((entry_num & 1) == 0) 
      *pout_bool = true;
    else 
      *pout_bool = false;
  }

  return nbytes;
}

size_t parse_char(char *pbuf, size_t buf_nbytes, char *pout_char)
{
  assert(pbuf);
  assert(pout_char);

  if (buf_nbytes == 0)
    return 0;

  if ((*pbuf <= ' ') || (*pbuf >= 127)) {
    return 0;
  }

  *pout_char = *pbuf;
  return 1;
}

size_t parse_int(char *pbuf, size_t buf_nbytes, int *pout_int)
{
  assert(pbuf);
  assert(pout_int);
  int sign = 1;
  bool is_valid = false;
  char *p = pbuf;
  int i = 0;

  while (*p && (p - pbuf < buf_nbytes)) {
    if (isspace(*p))
      break;

    if ((*p == '-') && ((p - pbuf) == 0)) {
      sign = -1.0f;
    } else if ((*p == '+') && ((p - pbuf) == 0)) {
      sign = 1.0f;
    } else if (isdigit(*p)) {
      is_valid = true;
      i *= 10;
      i += (*p - '0');
    } else {
      is_valid = false;
      break;
    }

    p++;
  }

  if (!is_valid) {
    return 0;
  }

  *pout_int = i * sign;
  return p - pbuf;
}

size_t parse_string(char *pbuf, size_t buf_nbytes, char *pout_string, size_t out_string_nbytes)
{
  assert(pbuf);
  assert(pout_string);
  bool is_valid = false;
  char *p = pbuf;

  size_t nbytes = strlen(pbuf);
  if (nbytes >= out_string_nbytes) {
    log_writeln(F("ERROR: string too long."));
    return 0;
  }

  memcpy(pout_string, pbuf, nbytes + 1);

  return nbytes;
}

size_t parse_float(char *pbuf, size_t buf_nbytes, float *pout_float)
{
  assert(pbuf);
  assert(pout_float);
  *pout_float = 0.0f;

  bool is_reading_significand = true;
  bool is_valid = false;
  float sign = 1.0f;
  float f = 0;
  float fraction_div = 10.0f;
  char *p = pbuf;

  while (*p && (p - pbuf < buf_nbytes)) {
    if (isspace(*p)) {
      break;
    } else if (is_reading_significand) {
      if ((*p == '-') && ((p - pbuf) == 0)) {
        sign = -1.0f;
      } else if ((*p == '+') && ((p - pbuf) == 0)) {
        sign = 1.0f;
      } else if (*p == '.') {
        is_reading_significand = false;
      } else if (isdigit(*p)) {
        is_valid = true;
        f *= 10;
        f += (*p - '0');
      } else {
        is_valid = false;
      }
    } else {
      if (isdigit(*p)) {
        f += (*p - '0') / fraction_div;
        fraction_div *= 10;
      } else {
        is_valid = false;
      }
    }

    p++;
  }

  if (!is_valid) {
    return 0;
  }

  *pout_float = sign * f;
  return p - pbuf;
}

typedef struct {
  char *pstring;
  int expected_nbytes;
  float expected_value;
} parse_float_test_case;

parse_float_test_case parse_float_test_case_by_index[] = {
  { "", 0, 0.0f },
  { "0", 1, 0.0f },
  { "1", 1, 1.0f },
  { "+0", 2, 0.0f },
  { "1.0", 3, 1.0f },
  { "-1.0", 4, -1.0f },
  { "1.1", 3, 1.1f },
  { "-1.1", 4, -1.1f },
  { "1.23", 4, 1.23f },
  { "-3.456", 6, -3.456f },
};
#define PARSE_FLOAT_TEST_CASE_BY_INDEX_COUNT sizeof(parse_float_test_case_by_index) / sizeof(parse_float_test_case_by_index[0])

bool test_parse_float()
{
  bool ret = true;

  for(int i = 0; i < PARSE_FLOAT_TEST_CASE_BY_INDEX_COUNT; i++) {
    parse_float_test_case *ptest_case = &parse_float_test_case_by_index[i];
    float f = -1.0f;
    size_t nbytes = parse_float(ptest_case->pstring, strlen(ptest_case->pstring), &f);
    if (ptest_case->expected_nbytes != nbytes) {
      LOG_E(F("Expected nbytes=%d, but got %d"), ptest_case->expected_nbytes, nbytes);
      ret = false;
    }

    if ((nbytes != 0 ) && (ptest_case->expected_value != f)) {
      LOG_E(F("Expected value=%f, but got %f"), ptest_case->expected_value, f);
      ret = false;
    }
  }

  return ret;
}

size_t parse_motor_id(char *pbuf, size_t buf_nbytes,  motor_id_t *pout_motor_id)
{
  assert(pbuf);
  assert(pout_motor_id);

  char id;
  if (parse_char(pbuf, buf_nbytes, &id) == 0)
    goto error;

  id = toupper(id);
  if ((id < 'A') || (id > 'F'))
    goto error;

  *pout_motor_id = id - 'A';
  return 1;

error:
  log_writeln(F("ERROR: Invalid motor ID. Expected 'A'-'F'."));
  return 0;
}

size_t parse_string_in_table(char *pbuf, size_t buf_nbytes, char *ptable[], int ntable_entries, int *pout_entry_num)
{
  assert(pbuf);
  assert(ptable);
  assert(pout_entry_num);

  char *pend = pbuf + buf_nbytes;

  for (int i = 0; i < ntable_entries; i++) {
    size_t nbytes = strlen(ptable[i]);    
    if (nbytes == (pend - pbuf) && 
        (strncasecmp(ptable[i], pbuf, pend - pbuf) == 0)) {
      *pout_entry_num = i;
      return nbytes;
    }
  }

  return 0;
}

size_t parse_motor_angle_or_position(char *pargs, size_t args_nbytes, float *pvalue)
{
  assert(pargs);
  assert(pvalue);
  char *p = pargs;
  
  float new_value = 0.0f;
  size_t nbytes = parse_float(p, args_nbytes, &new_value);
  if (nbytes > 0) {
    *pvalue = new_value;
    args_nbytes -= nbytes;
    p += nbytes;
  } else if (nbytes == 0) {
    new_value = *pvalue;
    // Not a float, check for +/++/-/--.
    char first_plus_or_minus = '\0';
    nbytes = parse_char(p, args_nbytes, &first_plus_or_minus);
    args_nbytes -= nbytes;
    p += nbytes;
    if ((nbytes == 0) || ((first_plus_or_minus != '+') && (first_plus_or_minus != '-'))) {
      goto error; // Something other than a + or -
    }

    char second_plus_or_minus = '\0';
    nbytes = parse_char(p, args_nbytes, &second_plus_or_minus);
    args_nbytes -= nbytes;
    p += nbytes;

    if (nbytes == 0) {
      if (first_plus_or_minus == '+') {
        new_value += 10;
      } else {
        new_value -= 10;
      }      
    } else if (nbytes > 0) {
      if ((second_plus_or_minus != '+') && (second_plus_or_minus != '-')) {
        goto error; // Something other than a + or -
      }

      if (first_plus_or_minus != second_plus_or_minus)
        goto error;  // Not ++ or --
      
      if (first_plus_or_minus == '+') {
        new_value += 100;
      } else {
        new_value -= 100;
      }
    }
  }

  *pvalue = new_value;

  return p - pargs;

error:
  log_writeln(F("ERROR: Invalid floating point number. Expected [+/-]digits.digits."));
  return 0;
}

bool test_parse()
{
  float ret = true;

  // TODO.
  // int parse_bool(buf, nbytes, out_bool) <= true/false/on/off/1/0
  // int parse_char(buf, nbytes, out_char) <= returns 1 if > 32 && < 127 

  ret = test_parse_float() ? ret : false;

  // TODO: parse_motor_angle_or_position

  return ret;  
}

/*
 * Menu functions.
*/

static const int menu_item_max_name_nbytes = 25;

void extended_menu_robot_id()
{
  log_writeln(F(""));  
  log_writeln(F("Current robot ID is '%s'."), robot_name_by_robot_id[config.robot_id]);
  log_writeln(F("Available robot IDs:"));
  for (int i = ROBOT_ID_FIRST; i < ROBOT_ID_COUNT; i++) {
    log_writeln(F("  %d. %s."), i, robot_name_by_robot_id[i]);
  }
  log_writeln(F("Select new robot ID, or press [RETURN] to keep current robot ID."));
  log_write(F(">"));
}

void extended_menu_robot_serial()
{
  log_writeln(F(""));  
  log_writeln(F("Current robot serial is '%s'."), config.robot_serial);
  log_writeln(F("Enter new robot serial, or press [RETURN] to keep current serial."));
  log_write(F(">"));
}

void extended_menu_robot_name()
{
  log_writeln(F(""));  
  log_writeln(F("Current robot name is '%s'."), config.robot_name);
  log_writeln(F("Enter new robot name shorter than %d characters, or press [RETURN] to keep current name."), config_robot_name_nbytes);
  log_write(F(">"));
}

void extended_menu_factory_reset()
{
  log_writeln(F(""));    
  log_writeln(F("Type 'RESET' in all capital letters to clear the EEPROM and reboot the system or <CTRL+C> to exit."));
  log_write(F(">"));
}

void extended_menu_reboot()
{
  log_writeln(F(""));  
  log_writeln(F("Type 'REBOOT' in all capital letters to reboot the system or <CTRL+C> to exit.")) ;
  log_write(F(">"));
}

const menu_item_t menu_item_by_index[] = {  // TODO: F()
  { '1', "display configuration", NULL, true, command_config_display, "-- print configuration." },
  { '2', "configure robot id", extended_menu_robot_id, true, command_config_robot_id, "[id] print or set configured robot id." },
  { '3', "configure robot serial", extended_menu_robot_serial, true, command_config_robot_serial, "[string] -- print or set configured robot serial." },  
  { '4', "configure robot name", extended_menu_robot_name, true, command_config_robot_name, "[name] -- print or set configured robot name." },
  { '5', "run calibration", NULL, true, command_config_calibrate, "[CALIBRATE] -- print or run robot calibration configuration data." },
  { '0', "write configuration", NULL, true, command_config_write, "Write configuration data to EEPROM." },
  { 'B', "heartbeat", NULL, true, command_heartbeat, "off / on -- enable heartbeat (only emitted when input mode is packet)." },
  { 'D', "PID mode", NULL, false, command_pid_mode, "-- Enable/disable motors" },
  { 'E', "emergency stop", NULL, false, command_emergency_stop, "-- execute hardware emergency stop (E-Stop). Enters 'error' state. Requires reboot to reset."},
  { 'G', "gripper position", NULL, true, command_gripper_position, "" },
  { 'H', "home position", NULL, false, command_home_position, "-- set current positions to home." },
  { 'I', "input mode", NULL, true, command_input_mode, "packet / keyboard."},
  { 'K', "clock", NULL, true, command_clock, "get / set [YYYY-MM-DD] HH:MM[:SS].fraction]]." },
  { 'M', "motor status", NULL, false, command_motor_status, "-- display motor status." },
  { 'N', "motor angle", NULL, true, command_motor_angle, "motorid degrees -- degrees is 0.0 to 360.0, +15, -20, +, -, ++, --." },
  { 'P', "motor position", NULL, true, command_motor_position, "motorid position -- position is in the range X - Y." },  // TODO
  { 'Q', "run test sequence", NULL, false, command_run_test_sequence, "" },
  { 'S', "interrogate switches", NULL, false, command_interrogate_switches, "-- interrogate switch positions." },
  { 'T', "test motors", NULL, false, command_test_motors, "" },
  { 'W', "waypoint", NULL, true, command_waypoint, "" },
  { 'X', "expansion I/O", NULL, true, command_expansion_io, ""},
  // { 'M', "motor config", true, command_motor_config, ""},
  { 'Z', "zero position", NULL, false, command_zero_position, "-- set current positions to zero." },
  { '*', "factory reset", extended_menu_factory_reset, false, command_factory_reset, "RESET -- factory reset system, clearing EEPROM and rebooting."},
  { '!', "reboot", extended_menu_reboot, true, command_reboot, "REBOOT -- reboot system. Requires typing the 'REBOOT' keyword." }, 
  { '?', "help", NULL, false, command_help, "-- display this help message." },
};
#define MENU_ITEM_COUNT sizeof(menu_item_by_index) / sizeof(menu_item_by_index[0])

menu_item_t* menu_item_by_command_char(char ch)
{
  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    if (toupper(ch) == toupper(menu_item_by_index[i].command_char)) {
      return &menu_item_by_index[i];
    }
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

bool test_menu()
{
  bool ret = true;

  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    menu_item_t *pitem = &menu_item_by_index[i];
    if (!pitem->pname) {
      LOG_E(F("%d NULL pname"), i);
      ret = false;
    }
    if (strlen(pitem->pname) > menu_item_max_name_nbytes) {
      LOG_E(F("%d pname too long"), i);
      ret = false;
    }
    if (!pitem->pfunction) {
      LOG_E(F("%d NULL pfunction"), i);
      ret = false;
    }
    if (!pitem->phelp) {
      LOG_E(F("%d NULL phelp"), i);
      ret = false;
    }
  }

  return ret;
}

/*
 * Self-test functions.
 */

typedef struct {
  bool (*test_function)();
  char *pname;
} test_case_t;

test_case_t test_case[] = {
  { test_time, "time" }, 
  { test_log, "log" },
  { test_crc_calculate, "crc_calculate" },
  { test_config, "config" },
  { test_parse, "parse" },
  { test_menu, "menu" }
};
#define TEST_CASE_COUNT sizeof(test_case) / sizeof(test_case[0])

bool run_self_test() {
  bool ret = true;
  int failure_count = 0;
  log_writeln(F("Running self test:"));
  for (int i = 0; i < TEST_CASE_COUNT; i ++) {
    if (test_case[i].test_function()) {
      log_writeln(F("  %d. %s ... pass."), i, test_case[i].pname);
    } else {
      log_writeln(F("  %d. %s ... fail."), i, test_case[i].pname);
      ret = false;
      failure_count++;
    }
  }

  log_writeln(F("%d test cases run, %d passed, %d failed."), TEST_CASE_COUNT, TEST_CASE_COUNT - failure_count, failure_count);

  return ret;
}

/*
 * Command processing functions.
 */

int command_clock(char *pargs, unsigned nybtes_args)
{
  // TODO.
  assert(false);
}

int command_emergency_stop(char *pargs, size_t args_nbytes)
{
  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (args_nbytes != nbytes) {
    return -1;
  } 

  hardware_emergency_stop();

  return 0;
}

int command_gripper_position(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_config_display(char *pargs, unsigned args_nbytes)
{
  assert(pargs);  

  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (args_nbytes != nbytes) {
    return -1;
  } 

  config_display();
  
  return 0;
}

int command_config_robot_id(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  char *p = pargs;

  size_t nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes == 0) {
    log_writeln(F("Maintaining robot ID as '%s'."), robot_name_by_robot_id[config.robot_id]);    
    return p - pargs;
  }  

  robot_id_t robot_id = ROBOT_ID_FIRST - 1;
  nbytes = parse_int(p, args_nbytes, (int*)(&robot_id));
  if (nbytes < 0)  // TODO: is this ever true?
    return -1;
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if ((robot_id <= ROBOT_ID_FIRST) || (robot_id >= ROBOT_ID_LAST)) {
    log_writeln(F("ERROR: Invalid robot ID."));
    goto error;
  }
  
  if (args_nbytes > 0)
    goto error;

  log_writeln(F("Setting robot ID to '%s'."), robot_name_by_robot_id[robot_id]);

  config_set_robot_id(robot_id); // TODO: return value?

  return p - pargs;

error:
  return -1;
}

int command_config_robot_serial(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  char *p = pargs;

  size_t nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes == 0) {
    log_writeln(F("Maintaining robot serial '%s'."), config.robot_serial);
    return p - pargs;
  }  

  char robot_serial[config_robot_serial_nbytes];
  nbytes = parse_string(p, args_nbytes, robot_serial, config_robot_serial_nbytes);
  if (nbytes < 0)  // TODO: is this ever true?
    return -1;
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes > 0)
    goto error;

  log_writeln(F("Setting robot serial to '%s'."), robot_serial);

  config_set_robot_serial(robot_serial);  // TODO: return value?

  return p - pargs;

error:
  return -1;
}

int command_config_robot_name(char *pargs, size_t args_nbytes)  // TODO: should these return a size_t?
{
  assert(pargs);

  char *p = pargs;

  size_t nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes == 0) {
    log_writeln(F("Maintaining robot name '%s'."), config.robot_name);
    return p - pargs;
  }  

  char robot_name[config_robot_name_nbytes];
  nbytes = parse_string(p, args_nbytes, robot_name, config_robot_name_nbytes);
  if (nbytes < 0)  // TODO: is this ever true?
    return -1;
  args_nbytes -= nbytes;
  p += nbytes;
  
  if (args_nbytes > 0)
    goto error;

  log_writeln(F("Setting robot name to '%s'."), robot_name);

  config_set_robot_name(robot_name); // TODO: return value?

  return p - pargs;

error:
  return -1;
}

int command_config_calibrate(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  assert(false);
}

int command_config_write(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  } 

  assert(config_write());
  log_writeln(F("%d bytes of configuration data written to EEPROM."), sizeof(config_t));
  return 0;
}

int command_heartbeat(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_help(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  // TODO: confirm pargs is only whitespace.
  menu_help();
  
  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  } 

  return 0;
}

int command_home_position(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  } 

  SetPositionToHome();

  return nbytes;
}

int command_interrogate_switches(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  } 
  
  InterrogateLimitSwitches();
  return nbytes;
}

int command_input_mode(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);
}

int command_motor_status()
{  
  log_writeln(F("PID:%s"), mm6_pid_enabled ? "enabled" : "disabled");

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {    
    char angle_str[15] = {};
    dtostrf(mm6_get_angle(i), 3, 2, angle_str);
    log_writeln(F("%c: home:%d sta:%d enc:%d tar:%d err:%d spd:%d PWM:%d cur:%d hs:%d,%d,%d,%d->%d angle:%s"), 
        'A' + i, 
        motor_state[i].switch_previously_triggered,  // home.
        motor_state[i].progress,  // sta. Report whether or not the Motor has reached the target location.
        /* mm6_get_position(iMotor), */  // pos.
        noinit_data.motor_encoder[i] * motor_state[i].logic,  // enc.
        motor_state[i].target_encoder * motor_state[i].logic,  // tar.
        motor_state[i].pid_perror * motor_state[i].logic,  // err.
        motor_state[i].speed * motor_state[i].logic,  // spd.
        /* motor_state[i].target_speed, */  // tspd.
        current_pwm[i],  // pwm.
        motor_state[i].current,  // cur.
        motor_state[i].switch_reverse_off,  // hs.
        motor_state[i].switch_forward_on,  // hs.
        motor_state[i].switch_reverse_on,  // hs.
        motor_state[i].switch_forward_off,  // hs.
        (motor_state[i].switch_forward_off + motor_state[i].switch_reverse_on + motor_state[i].switch_forward_on + motor_state[i].switch_reverse_off) / 4, // hs.
        angle_str);
  } 

  return 0;
}

int command_motor_angle(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  motor_id_t motor_id = MOTOR_ID_A;
  char *p = pargs;
  size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);  // TODO: all nbytes to size_t.
  if (nbytes <- 0)
    goto error;  // parse_motor_id will emit message if error.   
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  float angle = mm6_get_angle(motor_id);
  nbytes = parse_motor_angle_or_position(p, args_nbytes, &angle);
  if (nbytes <= 0)
    goto error;  // parse_motor_angle_or_position will emit message if error.   
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes > 0)
    goto error;

  char angle_str[15] = {};
  dtostrf(angle, 3, 2, angle_str);
  log_writeln(F("Move Motor %c to an angle of %s degrees."), 'A' + motor_id, angle_str);

  mm6_set_angle(motor_id, angle);

  return p - pargs;

error:
  LOG_E(F(""));
  return -1;
}

int command_motor_position(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  motor_id_t motor_id = MOTOR_ID_A;
  char *p = pargs;
  size_t nbytes = parse_motor_id(p, args_nbytes, &motor_id);  // TODO: all nbytes to size_t.
  if (nbytes <= 0)
    goto error;  // parse_motor_id will emit message if error.   
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  float position = mm6_get_position(motor_id);
  nbytes = parse_motor_angle_or_position(p, args_nbytes, &position);
  if (nbytes <= 0)
    return -1;
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes > 0)
    goto error;

  char position_str[15] = {};
  dtostrf(position, 3, 2, position_str);
  log_writeln(F("Move Motor %c to position %s."), 'A' + motor_id, position_str);

  mm6_set_position(motor_id, position);

  return p - pargs;

error:
  LOG_E(F(""));
  return -1;
}

int command_pid_mode(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_run_test_sequence(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_test_motors(char *pargs, size_t args_nbytes)
{
  assert(pargs);
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (args_nbytes != nbytes) {
    return -1;
  } 

  TestMotors();

  return nbytes;
}

int command_factory_reset(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  char *p = pargs;
  
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;
  if (args_nbytes == 0)
    return -1;

  int entry_num = -1;
  char *reboot_table[] = { "RESET" };
  nbytes = parse_string_in_table(pargs, args_nbytes, reboot_table, 1, &entry_num);
  args_nbytes -= nbytes;
  p += nbytes;

  nbytes = parse_whitespace(p, args_nbytes);
  args_nbytes -= nbytes;
  p += nbytes;

  if (args_nbytes > 0)
    goto error;

  assert(entry_num == 0);
  hardware_reset();
  hardware_reboot();

  return 0;

error:

  return -1;
}

int command_reboot(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  pargs += nbytes;
  args_nbytes -= nbytes;
  if (args_nbytes == 0)
    return -1;

  int entry_num = -1;
  char *reboot_table[] = { "REBOOT" };
  nbytes = parse_string_in_table(pargs, args_nbytes, reboot_table, 1, &entry_num);
  // TODO: check entry_num == -1?
  // TODO: allow whitespace after REBOOT.
  if (nbytes != args_nbytes)
    return -1;

  assert(entry_num == 0);
  hardware_reboot();

  return 0;
}

int command_waypoint(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_expansion_io(char *pargs, size_t args_nbytes)
{
  // TODO.
  assert(false);

  return -1;
}

int command_zero_position(char *pargs, size_t args_nbytes)
{
  assert(pargs);

  // Confirm arguments are empty.
  size_t nbytes = parse_whitespace(pargs, args_nbytes);
  if (nbytes != args_nbytes) {
    return -1;
  } 
  
  ZeroPositions();

  return nbytes;
}

//----------------------------------------------------------------------

// Define other I/O lines.
int Home_IO = 51;
// #define ESERT 18 // Serial Transmit
// #define ESERR 19 // Serial Receive
// #define EI2CD 20 // I2C Data
// #define EI2CC 21 // I2C Clock

//*************************************
// Motor Status / Motion Control Vars.
//*************************************
int P1[] = {0,0,0,0,0,0}; // Testing Positions
int P2[] = {0,0,0,0,0,0}; // Testing Positions
int P3[] = {0,0,0,0,0,0}; // Testing Positions
int Distance[] = {0,0,0,0,0,0};
int Tracking = 0;
int Tracked[] = {0,0,0,0,0,0}; // Last Value send while tracking.
robot_id_t robot_id = 0;
int Gripper_OpenLoc = -130;
int Gripper_CloseLoc = -310;
//*************************************
// Waypoint Structure and vars
//*************************************
struct sWayPoint {
  byte Number;
  char Command;
  float A;
  float B;
  float C;
  float D;
  float E;
  float F;  
};
  sWayPoint WayPoint = {
    0,0,
    0,0,0,0,0,0
  };
String StringSplits[8];  //Used to store waypoints
//-----------------------------------------

String InBuffer = "";
char padbuffer [50];
int Command_Motor = MOTOR_ID_E;

void check_noinit_data()
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
      noinit_data.motor_encoder[i] = 0;
      noinit_data.motor_qe_prev[i] = 0;
    }
  } else {
    log_writeln(F("Reset without power cycle detected. Reusing stored motor encoder values."));
  }
}

void hardware_init() {
  mm6_init();
  check_noinit_data();
  pinMode(OPRLED, OUTPUT);
  pinMode(expansion_io_pinout[0], OUTPUT); // Tone
  pinMode(Home_IO, INPUT_PULLUP);

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

  //********************************************************
  // Get the Angle Offsets and Forward_Logic for ALL motors
  //********************************************************
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){    
    int Logic = 0;
    
    // The Angle Offset is used in the Position-to-Angle and Angle-to-Position calculations
    //   Each Rhino Robot may have mechanical differences in the positions of the home swithes, 
    //   so the encoder count when the arm is straight up is stored as an "AngleOffset" so that the
    //   MegaMotor6 Angle Values will work the actual physical position of the robot
    //   while the Position Values work with positions relative to the home switches.
    //     The values for the AngleOffets come from the "~" Command.

    // EEPROM.get(AngleOffsetELoc[iMotor], AngleOffset[iMotor]);

    // The Forward and Reverse Locic is used to turn the motors in the right direction to sync with the encoders.
    //   Each Rhino Robot may have the wires to the motors reversed.
    //   So the Forward and Reverse Locic is used to correct that.    
    //     The values for the Direction Logic come from the "m" command.
    //       Since the Forward and Reverse Locic are used for an I/O line the values are 0 or 1'
    Logic = config.motor[iMotor].direction_logic;
    if (Logic != 0) 
      Logic = !0;  
    Reverse_Logic[iMotor] = Logic; // Reverse Logic - The value for the Direction IO Line when the motor needs to move Reverse. Defaults to 1.
    Forward_Logic[iMotor] = !Logic; // Forward Logic - The value for the Direction IO Line when the motor needs to move forward. Defaults to 0.
    
    // The Motor Locic is used to contol which way the motors turn in responce to the Positions.
    //   Each Rhino Robot may have the motor assembled on either side - which winds up reversing the motors direction mechanically.
    //   So the Moto Locic is used to correct that.    
    //     The values for the Motor Logic are set by the setup.
    //       Since the Forward and Reverse Locic are used to invert the position the values are 1 or -1'
    Logic = config.motor[iMotor].motor_logic;
    if (Logic !=1 ) 
      Logic=-1;
    motor_state[iMotor].logic = Logic; // Motor Logic - The value for inverting the Position when the motor needs to move Reverse. 
  }
  
  //******************************************
  // Get the Type of Rhino Robot being used.
  // XR1, XR2, XR3, or XR4
  //******************************************
  robot_id = config.robot_id;
  //******************************************
  // Get the gripper open and close  
  //******************************************
  Gripper_OpenLoc = config.gripper_open_location;
  Gripper_CloseLoc = config.gripper_close_location;
  if ((robot_id < ROBOT_ID_FIRST) || (robot_id > ROBOT_ID_LAST)) {
    // If the EEPROM reported an invalid robot id,
    //  then the EEPROM data wasn't set yet.
    // Set EEPROM robot_id to default.
    // AND set all EEPROM Angle Offsets to 0.
    config_set_robot_id(ROBOT_ID_DEFAULT);
    config_set_angle_offsets(0, 0, 0, 0, 0);
  } else {
    log_writeln(F("Configured for '%s'."), robot_name_by_robot_id[robot_id]);
  }

  ShowPositions();
}

void gather_status(status_t *pstatus)
{
  assert(pstatus);

  pstatus->state = state;
  for (int i = 0; i < MOTOR_ID_COUNT; i++) {
    pstatus->motor[i].angle = mm6_get_angle(i);  // TODO: Or active?
    pstatus->motor[i].current_draw = mm6_get_current_draw(i);
    pstatus->motor[i].switch_triggered = mm6_get_switch_triggered(i);
    pstatus->motor[i].thermal_overload_active = mm6_get_thermal_overload_active(i);
    pstatus->motor[i].overcurrent_active = mm6_get_overcurrent_active(i);
  }
}

void process_serial_input()
{
  const char ASCII_CTRL_C = 3;
  const char ASCII_BACKSPACE = 8;
  const char ASCII_BELL = 9;
  const char ASCII_RETURN = 13;
  const char ASCII_DELETE = 127;
  const int command_args_max_nbytes = 64;
  static char command_args[command_args_max_nbytes] = {};
  static int command_args_nbytes = 0;
  static menu_item_t *pprev_menu_item = NULL;
  static bool have_command = false;
  static bool reset_prompt = true;

  bool status_updated = false;
  static status_t previous_status = {};
  status_t status;
  gather_status(&status);

  if (memcmp(&previous_status, &status, sizeof(status_t)) != 0) {
    status_updated = true;
  }

  static unsigned long previous_status_time_millis = 0;
  unsigned long current_time_millis = millis();

  if (reset_prompt || 
      (!pprev_menu_item &&
       status_updated && 
       (current_time_millis - previous_status_time_millis > 250)))
  {
    if (status_updated && !reset_prompt)
      log_writeln(F(""));

    pprev_menu_item = NULL;
    command_args_nbytes = 0;
    have_command = false;
    reset_prompt = false;

    for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
      if (!config.motor[i].configured) {
        log_write(F("%c:-.-- "), 'A' + i);        
      } else {
        char angle_str[15];
        dtostrf(status.motor[i].angle, 3, 2, angle_str);
        char motor_name = (status.motor[i].switch_triggered ? 'A' : 'a') + i;
        log_write(F("%c:%s "), motor_name, angle_str);  // TODO: F()
      }
    }

    if (strlen(config.robot_name) != 0)
      log_write(F("%s "), config.robot_name);

    log_write(F("%s> "), state_name_by_state[state]);  // TODO: F()

    memcpy(&previous_status, &status, sizeof(status_t));
    previous_status_time_millis = current_time_millis;
  }

  if (Serial.available()) {
    char input_char = Serial.read();           

    if (input_char == ASCII_CTRL_C) {
      log_writeln(F("<CTRL+C>"));
      reset_prompt = true;
    } else if (!have_command) {
      if (input_char == ASCII_RETURN) {
        log_writeln(F(""));
        reset_prompt = true;
      } else if ((input_char == ASCII_BACKSPACE) || (input_char == ASCII_DELETE)) {
        log_write(F("\9"));  // Emit ASCII bell (flashes screen on some terminals)
      } else {
        menu_item_t *pmenu_item = menu_item_by_command_char(input_char);
        pprev_menu_item = pmenu_item;

        if (!pmenu_item) {
          log_writeln(F("ERROR: Invalid command '%c'. Type '?' for help."), input_char);  // TODO: NAK?
          reset_prompt = true;
        } else {
          log_write(F("%s"), pmenu_item->pname);
          if (pmenu_item->print_sub_menu_fn)
            pmenu_item->print_sub_menu_fn();
          if (pmenu_item->has_args)
            log_write(F(" "));  // Additional space for args.

          have_command = true;
        }
      }
    } else {
      assert(pprev_menu_item);
      if ((input_char == '\n') || (input_char == '\r')) {
        log_writeln(F(""));
        command_args[command_args_nbytes] = '\0';
        
        // Eat whitespace at start of arguments.
        size_t nbytes = parse_whitespace(command_args, command_args_nbytes);
        char *p = command_args + nbytes;
        command_args_nbytes -= nbytes;

        if (pprev_menu_item->pfunction(p, command_args_nbytes) != command_args_nbytes) {
          log_writeln(F("ERROR: Invalid command arguments. Type '?' for help."));  // TODO: NAK?
        }
        reset_prompt = true;
      } else if ((input_char == ASCII_BACKSPACE) || (input_char == ASCII_DELETE)) {
        // TODO: only 1 BS if we're entering args.
        int nbackspaces_count = strlen(pprev_menu_item->pname) + 2;  // +2 for " >"
        if (pprev_menu_item->has_args) 
          nbackspaces_count++;  // Account for additional ' '.
        for (int i = 0; i < nbackspaces_count; i++) {
          Serial.write(ASCII_BACKSPACE);
          Serial.write(" ");
          Serial.write(ASCII_BACKSPACE);
        }
        reset_prompt = true;
      } else if (command_args_nbytes < command_args_max_nbytes - 1) {  // -1 to leave space for '\0' at end.
        log_write(F("%c"), input_char);
        command_args[command_args_nbytes++] = input_char;
      } else {
        log_writeln(F(""));
        LOG_E(F("Too many characters in input buffer."));
        reset_prompt = true;
      }
    }
  }
}

bool thermal_overload_detected()
{
  bool detected = false;

  static bool thermal_overload_state[MOTOR_ID_COUNT] = {};

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    bool thermal_overload = mm6_get_thermal_overload_active(i);
    if (thermal_overload)
      detected = true;

    if (thermal_overload != thermal_overload_state[i]) {
      if (thermal_overload)
        log_writeln(F("ERROR: Thermal overload error detected on motor %c."), 'A' + i);
      else
        log_writeln(F("Thermal overload error cleared on motor %c."), 'A' + i);
      thermal_overload_state[i] = thermal_overload;
    }
  }

  return detected;
}

bool overcurrent_detected()
{
  const float current_limit = 0.1f;
  bool detected = false;

  static float max_current = 0.0f;

  static bool overcurrent_state[MOTOR_ID_COUNT] = {};

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    float f = mm6_get_current_draw(i);
    if (f > current_limit)
      detected = true;

    if (f > max_current) {
      char str[15];
      dtostrf(f, 7, 3, str);
      LOG_D(F("max_current: %s"), str);  // TODO: Why does this never print?
    }

    if ((f > current_limit) != overcurrent_state[i]) {
      if (f > current_limit)
        log_writeln(F("ERROR: Overcurrent error detected on motor %c."), 'A' + i);
      else 
        log_writeln(F("Overcurrent error cleared on motor %c."), 'A' + i);
      overcurrent_state[i] = f > current_limit;
    }
  }

  return detected;
}

state_t state_init_execute()
{
  Serial.begin(38400);

  log_writeln(F("\n\rBooting Arduino Mega 2560 MegaMotor6 controller for Rhino Robots arms and accessories."));
  char version_str[15] = {};
  dtostrf(software_version, 3, 2, version_str);
  log_writeln(F("Version: %s (%s)."), version_str, psoftware_date);  
  log_string("\n\r");

  bool config_read_success = config_read();
  if (!config_read_success) {
    log_writeln(F("ERROR: Invalid configuration. Re-initializing configuration data."));
    config_clear();
  } else {
    log_writeln(F("Read %d bytes of configuration data from EEPROM. Configuration is valid."), sizeof(config_t));
  }

  config_display();
  hardware_init();

  bool self_test_success = run_self_test();

  menu_help();
  log_writeln(F("Ready."));
  
  // return (config_read_success && self_test_success) ? STATE_MOTORS_OFF : STATE_ERROR; TODO2
  return STATE_MOTORS_ON;
}

bool state_motors_off_enter()
{
  // mm6_enable_motors(false);  !! TODO
  return true;
}

state_t state_motors_off_execute()
{
  process_serial_input();
  state = STATE_MOTORS_OFF;
  return state;
}

bool state_motors_on_enter()
{
  mm6_enable_motors(true);
  mm6_pid_enable(true);
  return true;
}

state_t state_motors_on_execute()
{
  process_serial_input();
  return STATE_MOTORS_ON;
}

bool state_motors_on_exit()
{
  mm6_pid_enable(false);
  mm6_enable_motors(false);
  return true;
}

bool state_error_enter()
{
  hardware_emergency_stop();
  return true;
}

state_t state_error_execute()
{
  process_serial_input();
  return STATE_ERROR;
}

typedef struct {
  bool (*enter)();
  state_t (*execute)();
  bool (*exit)();
} state_machine_entry;

state_machine_entry state_machine_entry_by_state[] = {
  { NULL, state_init_execute, NULL },
  { state_motors_off_enter, state_motors_off_execute, NULL },
  { state_motors_on_enter, state_motors_on_execute, state_motors_on_enter },
  { state_error_enter, state_error_execute, NULL }  
};

void state_machine_enter_state(state_t start_state)
{
  if (!state_machine_entry_by_state[start_state].enter()) {
    state = STATE_ERROR;
  } else {
    state = start_state;
  }
}

state_t state_machine_execute(state_t current_state)
{
  assert(current_state >= STATE_FIRST);
  assert(current_state <= STATE_LAST);

  static state_t previous_state = STATE_FIRST - 1;

  if (previous_state != current_state) {
    if (previous_state != STATE_FIRST - 1) {
      LOG_D(F("%s_exit()"), state_name_by_state[previous_state]);
      if (state_machine_entry_by_state[previous_state].exit &&
          !state_machine_entry_by_state[previous_state].exit())
        current_state = STATE_ERROR;

      LOG_D(F("%s_enter()"), state_name_by_state[current_state]);
      if (state_machine_entry_by_state[current_state].enter &&
          !state_machine_entry_by_state[current_state].enter())
        current_state = STATE_ERROR;
    }
    previous_state = current_state;
  }

  assert(state_machine_entry_by_state[current_state].execute);
  return state_machine_entry_by_state[current_state].execute();
}

void setup() 
{
  /* 
   * The setup() function will only run once, after each powerup or reset of the Arduino board.
   * https://www.arduino.cc/reference/en/language/structure/sketch/setup/
   */

  state = state_machine_execute(STATE_INIT);
}

bool machine_check()
{
  bool ok = config_check();

  if (config.robot_id == ROBOT_ID_UNKNOWN) {
    log_writeln(F("ERROR: robot_id == ROBOT_ID_UNKNOWN. Configure robot and restart."));
    ok = false;
  }

  ok = (thermal_overload_detected() || overcurrent_detected()) ? false : ok;

  // TODO: Detect error if any encoders don't transition from 0 -> 1 -> 3 -> 4 (or reverse).

  return ok;
}

void loop()
{
  /*
   * After creating a setup() function, which initializes and sets the initial values, the loop() 
   * function does precisely what its name suggests, and loops consecutively, allowing your 
   * program to change and respond. Use it to actively control the Arduino board.
   * https://www.arduino.cc/reference/en/language/structure/sketch/loop/
  */

  static bool previous_machine_check_ok = true;
  if (previous_machine_check_ok && !machine_check()) {
    previous_machine_check_ok = false;
    state = STATE_ERROR;
    log_writeln(F("ERROR: Machine check failed."));
  }

  state = state_machine_execute(state);

  return;

/*
  static int lMotor = 0;
  lMotor++; if (lMotor==6) lMotor=0;// Move to the next motor  
  motor_state[lMotor].current  = analogRead(motor_pinout[lMotor].in_current_draw);

  if (Tracking>0) {
    TrackReport(lMotor);
  } 
*/
/*  int HomeButton = digitalRead(Home_IO);
  if (HomeButton == 0) {
    for (int hMotor=MOTOR_IF_A; hMotor<=MOTOR_IF_F; hMotor++){
      //motor_state[hMotor].target_encoder = 0;
    }
  }*/

  if (Gripper_StallX>80) {
    // Motor A has been stalled for over 1/4 second.
    Serial.print("Gripper Stall @");
    Serial.print(Gripper_StallE);
    Serial.print("  Cur=");
    Serial.print(Gripper_StallC);
    Serial.print("  Open=");
    Serial.print(Gripper_OpenLoc);
    Serial.print("  Closed=");
    Serial.print(Gripper_CloseLoc);   
    Serial.println(".");
    Gripper_StallX=0;
    motor_state[MOTOR_ID_A].target_encoder = noinit_data.motor_encoder[MOTOR_ID_A];
    motor_state[MOTOR_ID_A].current = 0;
  }

#if 0
    if (input_char = '\n'){
      Serial.print(InBuffer + ":");
      if (InBuffer=="+"){
        int Position = mm6_get_position(Command_Motor)+10;
        mm6_set_position(Command_Motor, Position);
      } else if (InBuffer=="++"){
        int Position = mm6_get_position(Command_Motor)+100;
        mm6_set_position(Command_Motor, Position);
      } else if (InBuffer=="-"){
        int Position = mm6_get_position(Command_Motor)-10;
        mm6_set_position(Command_Motor, Position);
      } else if (InBuffer=="--"){
        int Position = mm6_get_position(Command_Motor)-100;
        mm6_set_position(Command_Motor, Position);
      } else if ((InBuffer=="G") || (InBuffer=="g")){
        mm6_pid_enable(true);        
      } else if ((InBuffer=="H") || (InBuffer=="h")){
        SetPositionToHome();
      } else if ((InBuffer=="I") || (InBuffer=="i")){
        InterrogateLimitSwitches();
      } else if ((InBuffer=="K") || (InBuffer=="k")){
        RunWayPointSeq();
      } else if ((InBuffer=="O") || (InBuffer=="o")){
        OpenGripper();
      } else if ((InBuffer=="M") || (InBuffer=="m")){
        TestMotors();
      } else if ((InBuffer=="P") || (InBuffer=="p")){
        ShowPositions();
      } else if ((InBuffer=="Q") || (InBuffer=="q")){
        TestSeq1();
      } else if ((InBuffer=="R") || (InBuffer=="r")){
        if (InBuffer=="R"){
          config_set_robot_id(ROBOT_ID_RHINO_XR_4);  
        } else {
          config_set_robot_id(ROBOT_ID_RHINO_XR_3);  
        }        
      } else if ((InBuffer=="S") || (InBuffer=="s")){
        mm6_pid_enable(false);
      } else if ((InBuffer=="T") || (InBuffer=="t")){
        DisplayStatus();
      } else if ((InBuffer=="U") || (InBuffer=="u")){
        StopTracking();
      } else if ((InBuffer=="V") || (InBuffer=="v")){
        CloseGripper();
      } else if ((InBuffer=="W") || (InBuffer=="w")){
        StartTracking();
      } else if ((InBuffer=="X") || (InBuffer=="w")){
        SetHomeToCenterOfSwitches();
      } else if ((InBuffer=="Z") || (InBuffer=="z")){
        ZeroPositions();
      } else if ((InBuffer=="N") || (InBuffer=="n")){
        char version_str[15] = {};
        dtostrf(software_version, 3, 2, version_str);
        log_writeln(F("Version: %s (%s)."), version_str, psoftware_date);  
      } else if (InBuffer=="~"){
        SetZeroAngles();
      } else if (InBuffer=="?"){
        ShowHelp();
      } else if (InBuffer=="~!@#$+"){
        RunMotorsForAsmTest();
      } else {
        char m = InBuffer.charAt(0);
        if ((m>=65) && (m<=70)) {
          // A-F
          Command_Motor = m - 65;
          String WhatToDo = InBuffer.substring(1);          
          if (WhatToDo.length()>0) 
          {
            if (WhatToDo=="X") {
              Exercise(Command_Motor);
            } else if (WhatToDo=="R") {
              Reverse(Command_Motor);
            } else if (WhatToDo=="-") {
              int Position = mm6_get_position(Command_Motor)-10;
              mm6_set_position(Command_Motor, Position);              
            } else if (WhatToDo=="--") {
              int Position = mm6_get_position(Command_Motor)-100;
              mm6_set_position(Command_Motor, Position);              
            } else if (WhatToDo=="+") {
              int Position = mm6_get_position(Command_Motor)+10;
              mm6_set_position(Command_Motor, Position);              
            } else if (WhatToDo=="++") {
              int Position = mm6_get_position(Command_Motor)+100;
              mm6_set_position(Command_Motor, Position);              
            } else {              
              int Position = WhatToDo.toInt();
              mm6_set_position(Command_Motor, Position);
            }      
          } else {
            Serial.print("Command Motor Set to: ");
            Serial.print(char(Command_Motor+65));    
            Serial.println(".");
          }
        } else if ((m>=97) && (m<=102)) {
          // a-f
          Command_Motor = m - 97;
          if (InBuffer.length()>1) 
          {
            InBuffer.setCharAt(0,32);
            float Angle = InBuffer.toFloat();
            mm6_set_angle(Command_Motor, Angle);
            //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
          }
        } else if (m==120) {
          // x
          SetWayPointAngle();
        } else if (m==114) {
          // r
          GetWayPointAngle();
        } else if (m==33) {
          // !
          MoveToAWayPointAngle();
        }
      }
      InBuffer = "";
    } else {
      InBuffer = InBuffer + input_char;
    }
  }
#endif
}

//*****************************************
// Track the motors by sending the current 
//  position to a connected computer 
//*****************************************
void StartTracking() {
  if (InBuffer=="W"){
    Tracking=1; // Track Position 
    Serial.println("Tracking Positions");
  } else {
    Tracking=2; // Track Angles
    Serial.println("Tracking Angles");
  }
  // Set the last tracked position to some big number just  
  // to make the routine report the current position.
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    Tracked[iMotor]=32000;
  }
}

void StopTracking(){
  Tracking=0; // Track none
  Serial.println("Tracking Off");
}
 
void TrackReport(motor_id_t motor_id){
  if (Tracking>0){
    int Position = mm6_get_position(motor_id);
    if (Tracked[motor_id]!=Position) {
      Serial.print("@");
      if (Tracking==1) {
        Serial.print(char(motor_id+65));
        Serial.print(mm6_get_position(motor_id));    
      } else if (Tracking==2) {
        Serial.print(char(motor_id+97));
        Serial.print(mm6_get_angle(motor_id));    
      }    
      Serial.print(":HS");
      Serial.print(motor_state[motor_id].switch_previously_triggered);
      Serial.println(":");
      Tracked[motor_id]=Position;
    }
  }
}

//*************************************
// A test to see if the motor's current positions
//  are equal to the Target Positions.
// Which means that the motors are all at rest.
//*************************************
int AllAtTarget(){
  // Calculate and return a value that indicates that 
  // all motors have reached the target position.
  return ((motor_state[MOTOR_ID_A].progress == 0) && 
          (motor_state[MOTOR_ID_B].progress == 0) && 
          (motor_state[MOTOR_ID_C].progress == 0) && 
          (motor_state[MOTOR_ID_D].progress == 0) && 
          (motor_state[MOTOR_ID_E].progress == 0) && 
          (motor_state[MOTOR_ID_F].progress == 0));
}

void ZeroPositions() {
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    noinit_data.motor_encoder[iMotor] = 0;
    motor_state[iMotor].target_encoder = 0;
  }
  Serial.println("Current Positions set to Zero.");
}

void SetHomeToCenterOfSwitches() {
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    SetNewHome(iMotor);
  }
  Serial.println("Home Positions set to Center Of Switches.");
}

void ShowPositions() {
  Serial.print("Current Positions: ");
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    Serial.print(char(iMotor+65));
    Serial.print("=");
    Serial.print(mm6_get_position(iMotor));
    if (iMotor<5) Serial.print(",");
  }
  Serial.println(".");  
}

void SetPositionToHome() {
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
    motor_state[iMotor].target_encoder = 0;
  }  
  Serial.println("Setting Targets to Home Position.");
}

/*
void ShowHelp() {
  Serial.println("Command List:");
  Serial.println("  +  : Target Position +10");
  Serial.println("  ++ : Target Position +100");
  Serial.println("  -  : Target Position -10");
  Serial.println("  -- : Target Position -100");
  Serial.println("  G  : Go (Turn On PID)");        
  Serial.println("  H  : SetTarget Positions To Home");
  Serial.println("  I  : Interrogate Limit Switches");
  Serial.println("  K  : Run Way Point Seq");
  Serial.println("  M  : Test Motors");
  Serial.println("  N  : Show firmware Ver No.");
  Serial.println("  O  : Open Gripper");
  Serial.println("  P  : Show Current Positions");
  Serial.println("  Q  : Run Test Seq 50 times");
  Serial.println("  R  : Set Rhino Ver");
  Serial.println("  S  : Stop (Turn Off PID");
  Serial.println("  T  : Show Status");
  Serial.println("  U  : Stop Tracking");
  Serial.println("  V  : CloseGripper");
  Serial.println("  W  : StartTracking");
  Serial.println("  X  : Set Home to Center of Switches");
  Serial.println("  Z  : Set Current Positions to 0");
  Serial.println("  ~  : Set 0-Angles to Current Positions");  
  Serial.println("  ?  : Show this list");  
  Serial.println("---------------------------------");
}
*/

/*
  Serial.println("Command List:");
  Serial.println("+  : Target Position +10");
  Serial.println("++ : Target Position +100");
  Serial.println("-  : Target Position -10");
  Serial.println("-- : Target Position -100");
  Serial.println("GO  : Open Gripper");
  Serial.println("GC  : Close Gripper");
  Serial.println("G?  : Show Gripper Status");
  Serial.println("HM  : Set Target Positions To Home");
  Serial.println("HF  : Find Home (Center of Limit Switches");
  Serial.println("HZ  : Set Home to Center of Switches");
  Serial.println("MT  : Test Motors");
  Serial.println("M0  : Turn Motors Off (PID");
  Serial.println("M1  : Turn Motors On (PID)");
  Serial.println("M?  : Show Motor Status");
  Serial.println("PZ  : Set Current Positions to 0");  
  Serial.println("PA  : Set 0-Angles to Current Positions");  
  Serial.println("P?  : Show Current Positions and Angles");
  Serial.println("T0  : Stop Tracking");
  Serial.println("T1  : Start Tracking");
  Serial.println("T?  : Show Tracking");
  Serial.println("V1 - V4  : Set Rhino Ver");
  Serial.println("V?  : Show Firmware and Rhino Ver No.");
  Serial.println("?  : Show this list");  

  Serial.println("Q  : Run Test Seq 50 times");
  Serial.println("K  : Run Way Point Seq");
  Serial.println("---------------------------------");
 */

void OpenGripper(){
  Gripper_StallX=0;
  motor_state[MOTOR_ID_A].target_encoder = Gripper_OpenLoc;
  Serial.println("Opening Gripper.");
}

void CloseGripper(){
  Gripper_StallX=0;
  motor_state[MOTOR_ID_A].target_encoder = Gripper_CloseLoc;
  Serial.println("Closing Gripper.");
}

//************************************************************
// Determine Encoder Steps Per Degree
//************************************************************
float EncoderStepsPerDegree(int Motor) {
  switch (Motor) {
    case MOTOR_ID_F: 
      if (robot_id == ROBOT_ID_RHINO_XR_4) {
        return 17.5; //(4.4)(66.1/1) XR4        
      } else {
        return 29.5; //(4.4)(66.1/1) XR3
      }
      break;
    case MOTOR_ID_E:
    case MOTOR_ID_D:
    case MOTOR_ID_C:
      if (robot_id == ROBOT_ID_RHINO_XR_4) {
        return 35; //(8.8)(66.1/1) XR4
      } else {
        return 36; //(8.8)(66.1/1) XR3
      }
      break;
    case MOTOR_ID_B:
      return 12.5; //(5.51)(165.4/1) XR4
      break;
    case MOTOR_ID_A:
      return 1;
  }  
}

//************************************************************
// Cacluation to convert Rhino Robot angles to motor position
//************************************************************
int AngleToPosition(int i, float Angle) {
  return (Angle * EncoderStepsPerDegree(i)) + config.motor[i].angle_offset;
}

//**********************************************
// Move a single Motor to a specified position. 
//**********************************************
void MoveMotorToE(int zMotor, int Position) {
  motor_state[zMotor].target_encoder = Position;
  motor_state[zMotor].progress = MOTOR_PROGRESS_ON_WAY_TO_TARGET; //Set the flag that indicates that the motor has been put in motion.
  Serial.print(" ->Move Motor ");
  Serial.print(char(zMotor+65));
  Serial.print(" to ");
  Serial.println(Position);  
}

//***********************************************************
// Move Motors in synchronous mode by specifying the angles. 
// (Except the Gripper)
//***********************************************************
void SyncMoveAngle(float AngleB, float AngleC, float AngleD, float AngleE, float AngleF) {
  int PositionB = AngleToPosition(MOTOR_ID_B, AngleB);
  int PositionC = AngleToPosition(MOTOR_ID_C, AngleC);
  int PositionD = AngleToPosition(MOTOR_ID_D, AngleD);
  int PositionE = AngleToPosition(MOTOR_ID_E, AngleE);
  int PositionF = AngleToPosition(MOTOR_ID_F, AngleF);
  SyncMove(PositionB, PositionC, PositionD, PositionE, PositionF);
}

//**************************************************************
// Move Motors in synchronous mode by specifying the positions. 
// (Except the Gripper)
//**************************************************************
void SyncMove(int PositionB, int PositionC, int PositionD, int PositionE, int PositionF) {
  motor_sync_move_enabled = false;  
  SyncMove_Status = MOTOR_PROGRESS_ON_WAY_TO_TARGET;
  // Store the target positions.
  End[MOTOR_ID_B]=PositionB;
  End[MOTOR_ID_C]=PositionC;
  End[MOTOR_ID_D]=PositionD;
  End[MOTOR_ID_E]=PositionE;
  End[MOTOR_ID_F]=PositionF;
  float MaxDistance = 0;
  // Caculate the travel distance for each motor.
  for (int iMotor = MOTOR_ID_B; iMotor <= MOTOR_ID_F; iMotor++){
    Start[iMotor] = motor_state[iMotor].target_encoder;
    Distance[iMotor]=End[iMotor]-Start[iMotor];
    // Keep track of the furthest travel distance.
    MaxDistance = max(MaxDistance,abs(Distance[iMotor]));
  }
  // Using the motor with the furthest travel distance,
  // caculate the ratios of travel distance between all motors.
  for (int iMotor = MOTOR_ID_B; iMotor <= MOTOR_ID_F; iMotor++){
    Ratio[iMotor] = Distance[iMotor]/MaxDistance;
    if (abs(Distance[iMotor])==MaxDistance) {
      LeadMotor = iMotor;
    }
    Serial.print(char(iMotor+65));
    Serial.print(": From:");
    Serial.print(Start[iMotor]);
    Serial.print(" To:");
    Serial.print(End[iMotor]);
    Serial.print(" Distance:");
    Serial.print(Distance[iMotor]);
    Serial.print(" Speed Ratio:");  
    Serial.println(Ratio[iMotor] * 100);
  }  
  motor_state[LeadMotor].target_encoder = End[LeadMotor];  
  SyncMove_Status = MOTOR_PROGRESS_ON_WAY_TO_TARGET;
  motor_sync_move_enabled = true;
  Serial.println("Start Sync Move");
}

void InterrogateLimitSwitches(){
  Serial.println("Interrogate Limit Switches");  
  Serial.print("  ");
  int CurrentMotorState = mm6_pid_enabled ? 1 : 0;  
  mm6_pid_enable(true);    
  InterrogateLimitSwitches2();
  Serial.println("  Done Interrogating Limit Switches");  
  if (CurrentMotorState == 0) {
    // mm6_pid_enable(false);
  }  
}

void Reverse(motor_id_t motor_id) {
  noinit_data.motor_encoder[motor_id] *= -1;
  motor_state[motor_id].target_encoder *= -1;
  motor_state[motor_id].logic *= -1;
  config_set_motor_logic(motor_id, motor_state[motor_id].logic);
  Serial.print("Motor ");
  Serial.print(char(Command_Motor+65));    
  Serial.println(" reversed.");  
}

void Exercise(int m) {
  Serial.print("Exercise Motor ");    
  Serial.println(m);    
  int Rep = 0;
  mm6_pid_enable(true);  
  do {
      Rep++;
      Serial.print("Rep: ");    
      Serial.println(Rep);        
      switch (m)
      {
        case MOTOR_ID_F:; //A
          InterrogateLimitSwitch(MOTOR_ID_F, 450, -450);
          break;
        case MOTOR_ID_E:; //A
          InterrogateLimitSwitch(MOTOR_ID_E, 280, -280);
          break;
        case MOTOR_ID_D:; //A
          InterrogateLimitSwitch(MOTOR_ID_D, 320, -320);
          break;
        case MOTOR_ID_C:; //A
          InterrogateLimitSwitch(MOTOR_ID_C, 300, -300);
          break;
        case MOTOR_ID_B:; //A
          InterrogateLimitSwitch(MOTOR_ID_B, 380, -380);
          break;
        case MOTOR_ID_A:; //A
          Serial.println("Swing Gripper out.");
          motor_state[MOTOR_ID_A].target_encoder = 30;
          do {TrackReport(m);} while (noinit_data.motor_encoder[m] < 25);
          Serial.println("Swing Gripper in.");
          motor_state[MOTOR_ID_A].target_encoder = -280;
          do {TrackReport(m);} while (noinit_data.motor_encoder[m] > -275);  
          break;
      }   
      delay(500);
    } while (1);
}

void InterrogateLimitSwitches2(){
  // InterrogateLimitSwitch(MOTOR_ID_F, 450, -450);
  InterrogateLimitSwitch(MOTOR_ID_E, 280, -280);
  InterrogateLimitSwitch(MOTOR_ID_D, 320, -320);
  InterrogateLimitSwitch(MOTOR_ID_C, 300, -300);
  InterrogateLimitSwitch(MOTOR_ID_B, 380, -380);
  InterrogateLimitSwitchA();
}

void InterrogateLimitSwitchA() {  
  MoveMotorToE(MOTOR_ID_A, 9999);
  delay(2000);
  int CurF = analogRead(motor_pinout[MOTOR_ID_A].in_current_draw);
  int EncF = noinit_data.motor_encoder[MOTOR_ID_A];
  int SwcF = digitalRead(motor_pinout[MOTOR_ID_A].in_switch);
  MoveMotorToE(MOTOR_ID_A, -9999);
  delay(1500);
  int CurR = analogRead(motor_pinout[MOTOR_ID_A].in_current_draw);
  int EncR = noinit_data.motor_encoder[MOTOR_ID_A];
  int SwcR = digitalRead(motor_pinout[MOTOR_ID_A].in_switch);  
  MoveMotorToE(MOTOR_ID_A, 9999);
  delay(1500);
  MoveMotorToE(MOTOR_ID_A, -9999);
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
  if (SwcF==0) {
    // Encoder goes Positive towards switch.
    int OverSwitch = ((motor_state[MOTOR_ID_A].switch_forward_on + EncF ) / 2);      
    MoveMotorToE(MOTOR_ID_A, OverSwitch);
    do {TrackReport(MOTOR_ID_A);} while (noinit_data.motor_encoder[MOTOR_ID_A] != OverSwitch);
    noinit_data.motor_encoder[MOTOR_ID_A] = 0;  
    motor_state[MOTOR_ID_A].target_encoder  = 0;
    motor_state[MOTOR_ID_A].logic = -1;
    config_set_motor_logic(MOTOR_ID_A, motor_state[MOTOR_ID_A].logic);
    Gripper_OpenLoc = -140;
    Gripper_CloseLoc = -310;    
    config_set_gripper_open_location(Gripper_OpenLoc);
    config_set_gripper_close_location(Gripper_CloseLoc);
    Serial.println("Done");
  } else {
    // Encoder goes Negative towards switch.
    int OverSwitch = ((motor_state[MOTOR_ID_A].switch_reverse_on + EncR ) / 2);      
    MoveMotorToE(MOTOR_ID_A, OverSwitch);
    do {TrackReport(MOTOR_ID_A);} while (noinit_data.motor_encoder[MOTOR_ID_A] != OverSwitch);
    noinit_data.motor_encoder[MOTOR_ID_A] = 0;  
    motor_state[MOTOR_ID_A].target_encoder  = 0;
    motor_state[MOTOR_ID_A].logic = 1;
    config_set_motor_logic(MOTOR_ID_A, motor_state[MOTOR_ID_A].logic);
    Gripper_OpenLoc = 140;
    Gripper_CloseLoc = 310;    
    config_set_gripper_open_location(Gripper_OpenLoc);
    config_set_gripper_close_location(Gripper_CloseLoc);
    Serial.println("Done");
  }
}

void InterrogateLimitSwitch(int m, int f, int r) {
  if (motor_state[m].switch_previously_triggered == 0){
    //Serial.print(" Centering Motor ");
    //Serial.println(char(m+65));
    //Serial.print("   Moving to: ");
    noinit_data.motor_encoder[m] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
    motor_state[m].target_encoder = 0;
    
    // Move to one side of switch and wait for the switch to be unpressed.
    Serial.print("  ");
    MoveMotorToE(m,r-130);
    do {TrackReport(m);} while (motor_state[m].switch_previously_triggered == 0);

    // Move to the other side of switch and wait for the switch to be pressed and then unpressed.
    Serial.print("  ");
    MoveMotorToE(m,f+130);
    do {TrackReport(m);} while (motor_state[m].switch_previously_triggered != 0);
    do {TrackReport(m);} while (motor_state[m].switch_previously_triggered == 0);
    //do {TrackReport(m);} while (mm6_get_position(m) < f);

    // Move back to first side of switch and wait for the switch to be pressed and then unpressed.        
    Serial.print("  ");
    MoveMotorToE(m,r-130);
    do {TrackReport(m);} while (motor_state[m].switch_previously_triggered != 0);
    do {TrackReport(m);} while (motor_state[m].switch_previously_triggered == 0);
    //do {TrackReport(m);} while (mm6_get_position(m) > r);

    // Calculate center of switches and then move to that place.
    int CenterEncoder = ((motor_state[m].switch_forward_off + motor_state[m].switch_reverse_on + motor_state[m].switch_forward_on + motor_state[m].switch_reverse_off) / 4);
    Serial.print("   Switch Positions: ");
    Serial.print(motor_state[m].switch_reverse_off);
    Serial.print(",");
    Serial.print(motor_state[m].switch_forward_on);
    Serial.print(",");
    Serial.print(motor_state[m].switch_reverse_on);
    Serial.print(",");
    Serial.print(motor_state[m].switch_forward_off);
    Serial.println(".");
    Serial.print("     Centering");    
    MoveMotorToE(m,CenterEncoder);
    do {TrackReport(m);} while (noinit_data.motor_encoder[m] != CenterEncoder);

    // Set Encoder and Target Values to 0.
    noinit_data.motor_encoder[m] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
    motor_state[m].target_encoder = 0;
    
  } else {
    Serial.print(" Motor ");
    Serial.print(char(m+65));
    Serial.println(" Home Switch Not Closed. - Skipping.");
  }
}

void SetNewHome(int m) {
    int NewHome = ((motor_state[m].switch_forward_off + motor_state[m].switch_reverse_on + motor_state[m].switch_forward_on + motor_state[m].switch_reverse_off) / 4);
    MoveMotorToE(m,NewHome);
    do {TrackReport(m);} while (noinit_data.motor_encoder[m] != NewHome);
    noinit_data.motor_encoder[m] = 0;
    motor_state[m].target_encoder = 0;
    Serial.print(" Motor ");
    Serial.print(char(m+65));
    Serial.println(" Centered");  
}

void TestMotors() {
  log_writeln(F("Test Motors"));    
  mm6_pid_enable(false);
  delay(250);

  ShowPositions();

  // Set all motor power lines to High-Z state by 
  // Setting speed to 0 and turning off brakes.  
  Serial.println("Setting Drive Power for all Motors to High-Z.");
  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++){    
    digitalWrite(motor_pinout[i].out_brake, LOW); // Turn the Brakes off.
    motor_state[i].speed = 0;
    mm6_set_pwm(i);
  }  
  
  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++){    
    TestMotor(i);
    if (((P2[i] - P1[i]) > 0 ) && ((P3[i] - P2[i]) < 0)) {
      log_writeln(F("Reversing Direction Logic on Motor %c."), 'A' + i);
      Forward_Logic[i] = !Forward_Logic[i];
      Reverse_Logic[i] = !Reverse_Logic[i];
      config_set_direction_logic(i, Reverse_Logic[i]);
      i = i - 1; // Test motor[i] again.
    }    
  }

  log_write(F("Forward encoder count:"));    
  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++){
    log_write(F(" %c="), 'A' + i);
    Serial_Print_Pos(P3[i] - P2[i]);
  } 
  log_writeln(F(".")); 
  
  Serial.print("Reverse encoder count:");
  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++){
    log_write(F(" %c="), 'A' + i);
    Serial_Print_Pos(P2[i] - P1[i]);
  } 
  log_writeln(F(".")); 

  for (int i = MOTOR_ID_FIRST; i <= MOTOR_ID_LAST; i++) {
    log_write(F("Motor %c configuration "), 'A' + i);
    if (P2[i] != P1[i]) {
      config_set_motor_configured(i, true);
      log_writeln(F("passed."));
    } else {
      config_set_motor_configured(i, false);
      log_writeln(F("FAILED."));
    }
  }    

  /* Auto Correct Reversed Motors.
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; m++){
    if (((P2[iMotor]-P1[iMotor]) >0 ) && ((P3[iMotor]-P2[iMotor])<0)) {
      Serial.print("Reversing Direction Logic on Motor ");
      Serial.println(char(m+65));
      Forward_Logic[iMotor] = !Forward_Logic[iMotor];
      Reverse_Logic[iMotor] = !Reverse_Logic[iMotor];
    }    
  } 
  */
  
  ShowPositions();
  log_writeln(F("Done Testing Motors."));
  mm6_pid_enable(true);
}

void TestMotor(int m) 
{
  const int TestSpeed = 255 - motor_min_speed;
  const int SpeedDelay = 50;
  
  log_writeln(F("Moving Motor %c."), 'A' + m);
  log_write(F("  Backward"));   // TODO: F()
  P1[m] = noinit_data.motor_encoder[m];  // Get Current Position.
  
  motor_state[m].speed = -TestSpeed; // Set Motor Speed.
  mm6_set_pwm(m);            // Set Motor PWM.
  log_write(F(" on"));
  delay(SpeedDelay);         // Short Delay to allow the motor to move.  
    
  motor_state[m].speed = 0;          // Turn off motor.
  mm6_set_pwm(m);            // Set Motor PWM.
  log_writeln(F(" off."));
  delay(SpeedDelay);         // Short Delay to allow the motor to stop.  

  P2[m] = noinit_data.motor_encoder[m];  // Get Current Position.

  log_write(F("  Forward "));
  //Serial.print("Moving Motor ");
  //Serial.print(char(m+65));
  //Serial.print(" forward ");

  motor_state[m].speed = TestSpeed;  // Set Motor Speed.
  mm6_set_pwm(m);            // Set Motor PWM.
  log_write(F(" on"));
  delay(SpeedDelay);         // Short Delay to allow the motor to move.  
    
  motor_state[m].speed = 0;          // Turn off motor.
  mm6_set_pwm(m);            // Set Motor PWM.
  log_writeln(F(" off."));
  delay(SpeedDelay);         // Short Delay to allow the motor to stop.  

  P3[m] = noinit_data.motor_encoder[m]; // Get Current Position.
}

void Serial_Print_Pos(int i){
  if (i > 0){
    Serial.print("+");
  } 
  Serial.print(i);
}

void Serial_print(int a, int l) {
  String formatS ="%+0zd";
  String D = String(l+1);
  formatS.replace("z",D);
  char formatC[10];
  formatS.toCharArray(formatC,10);
  sprintf (padbuffer, formatC,a);
  Serial.print(padbuffer);
}

// ****************************************************
// ****************************************************
//
//                  Move Sequences
//
// ****************************************************
// ****************************************************

void TestSeq1() {
  for (int m=0;m<=50;m++){
    Serial.println(m);
    InterrogateLimitSwitches();
  }
}

void TestSeq1b() {
Serial.println("start");
  mm6_pid_enable(false);
  OpenGripper();
  mm6_set_angle(MOTOR_ID_B, 90);
  mm6_set_angle(MOTOR_ID_E, -130);
  mm6_set_angle(MOTOR_ID_F, 45);
  mm6_pid_enable(true);
  do {delay(50);} while (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_APPROACHING_TARGET); 
  mm6_set_angle(MOTOR_ID_D,17.1);
  do {delay(50);} while (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_APPROACHING_TARGET); 
  CloseGripper();
  delay(1000);
  mm6_set_angle(MOTOR_ID_D, 0);
  mm6_set_angle(MOTOR_ID_E, -80);
  do {delay(50);} while (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_APPROACHING_TARGET); 
  mm6_set_angle(MOTOR_ID_F,-45);
  do {delay(50);} while (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_APPROACHING_TARGET); 
  OpenGripper();
  delay(1000);
  SetPositionToHome();
  do {delay(50);} while (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_BESIDE_TARGET); 
  mm6_pid_enable(false); 
  Serial.println("stop");
}

void RunWayPointSeq() {
  Serial.println("Start WayPoints");
  EEPROM.get(0, WayPoint);
  int NumberOf = WayPoint.A;
  if (NumberOf < 100) {
    mm6_pid_enable(true);
    for (int Step = 1; Step <= NumberOf; Step++) 
    {
      int Pin = 4;
      int val = 0;
      int Stp = 0;
      Serial.print("Step: ");
      Serial.print(Step);
      Serial.print(" - ");
      int eeAddress = Step*40;   //Location we want the data to be put.
      EEPROM.get(eeAddress, WayPoint);
      switch (WayPoint.Command)
      {
        case 65:; //A
        case 66:; //B
        case 67:; //C
        case 68:; //D
          WayPointMove(Step);
          break;
        
        case 71: //G
          Stp = WayPoint.A;
          Step = Stp - 1;
          Serial.print("Goto Step ");
          Serial.println(Stp);        
          break;
        
        case 73: //I
          InterrogateLimitSwitches2();
          break;
  
        case 74: //J
          val = 0;
          Pin = WayPoint.B;
          Stp = WayPoint.A;
          Serial.print("Goto Step ");
          Serial.print(Stp);        
          Serial.print(" If I/O[");
          Serial.print(Pin);
          Serial.println("]");
          pinMode(expansion_io_pinout[Pin-1], INPUT);
          val = digitalRead(expansion_io_pinout[Pin-1]);
          if (val == 0) Step = Stp - 1;
          break;
          
        case 87: //W
          val = WayPoint.C;        
          Serial.print("Wait ");
          Serial.print(val);
          Serial.print(" Miliseconds");
          delay(val);
          break;
          
      }  
    }
    mm6_pid_enable(false);  
    Serial.println("Done");
  } else {
    Serial.println("No Waypoints");  
  }
}

void WayPointMove(int i) {
  Serial.print("Goto ");
  MoveToWayPointAngle(i);

  switch (WayPoint.Command)
  {
    case 65 :do {delay(50);TrackUm();} while (Check_A());
      break;
    case 66 :do {delay(50);TrackUm();} while (Check_B());
      break;
    case 67 :do {delay(50);TrackUm();} while (Check_C());
      break;
    case 68 :do {delay(50);TrackUm();} while (Check_D());
      break;
  }  
}

void TrackUm() {
  if (Tracking>0) {
    for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
      TrackReport(iMotor);
    }
  }   
}

boolean Check_A() {
  return
  (motor_state[MOTOR_ID_C].progress > MOTOR_PROGRESS_AT_TARGET) ||   
  (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_AT_TARGET) ||   
  (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_AT_TARGET) ||   
  (motor_state[MOTOR_ID_F].progress > MOTOR_PROGRESS_AT_TARGET);       
}

boolean Check_B() {
  return
  (motor_state[MOTOR_ID_C].progress > MOTOR_PROGRESS_BESIDE_TARGET) ||   
  (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_BESIDE_TARGET) ||   
  (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_BESIDE_TARGET) ||   
  (motor_state[MOTOR_ID_F].progress > MOTOR_PROGRESS_BESIDE_TARGET);       
}

boolean Check_C() {
  return
  (motor_state[MOTOR_ID_C].progress > MOTOR_PROGRESS_NEAR_TARGET) ||   
  (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_NEAR_TARGET) ||   
  (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_NEAR_TARGET) ||   
  (motor_state[MOTOR_ID_F].progress > MOTOR_PROGRESS_NEAR_TARGET);       
}

boolean Check_D() {
  return
  (motor_state[MOTOR_ID_C].progress > MOTOR_PROGRESS_APPROACHING_TARGET) ||   
  (motor_state[MOTOR_ID_D].progress > MOTOR_PROGRESS_APPROACHING_TARGET) ||   
  (motor_state[MOTOR_ID_E].progress > MOTOR_PROGRESS_APPROACHING_TARGET) ||   
  (motor_state[MOTOR_ID_F].progress > MOTOR_PROGRESS_APPROACHING_TARGET);       
}

void SplitWayPoint(String waypoint)
{    
  for (int i = 0; i < 8; i++) 
  {
    StringSplits[i] = GetStringPartAtSpecificIndex(waypoint, '!', i); 
  }
}

String GetStringPartAtSpecificIndex(String StringToSplit, char SplitChar, int StringPartIndex)
{
  String originallyString = StringToSplit;
  String outString = "";
  for (int i1 = 0; i1 <= StringPartIndex; i1++)
  {
    outString = "";                   //if the for loop starts again reset the outString (in this case other part of the String is needed to take out)
    int SplitIndex = StringToSplit.indexOf(SplitChar);  //set the SplitIndex with the position of the SplitChar in StringToSplit

    if (SplitIndex == -1)               //is true, if no Char is found at the given Index
    {
      //outString += "Error in GetStringPartAtSpecificIndex: No SplitChar found at String '" + originallyString + "' since StringPart '" + (i1-1) + "'";    //just to find Errors
      return outString;
    }
    for (int i2 = 0; i2 < SplitIndex; i2++)
    {
      outString += StringToSplit.charAt(i2);      //write the char at Position 0 of StringToSplit to outString
    }
    StringToSplit = StringToSplit.substring(StringToSplit.indexOf(SplitChar) + 1);  //change the String to the Substring starting at the position+1 where last SplitChar found
  }
  return outString;
}

void SetZeroAngles() {
  int B = mm6_get_position(MOTOR_ID_B);
  int C = mm6_get_position(MOTOR_ID_C);
  int D = mm6_get_position(MOTOR_ID_D);
  int E = mm6_get_position(MOTOR_ID_E);
  int F = mm6_get_position(MOTOR_ID_F);
  config_set_angle_offsets(B, C, D, E, F);
}

void SetWayPointAngle() {  
  InBuffer.setCharAt(0,32);
  int Position = InBuffer.toInt();
  SplitWayPoint(InBuffer);  
  WayPoint.Number = Position;
  char Comm = StringSplits[1][0];
  WayPoint.Command = Comm;
  WayPoint.A = StringSplits[2].toFloat();
  WayPoint.B = StringSplits[3].toFloat();
  WayPoint.C = StringSplits[4].toFloat();
  WayPoint.D = StringSplits[5].toFloat();
  WayPoint.E = StringSplits[6].toFloat();
  WayPoint.F = StringSplits[7].toFloat();
  int eeAddress = Position*40;   //Location we want the data to be put.
  EEPROM.put(eeAddress, WayPoint);
  ReportWayPoint();
  
  Serial.println("Set");  
}

void GetWayPointAngle() {
  InBuffer.setCharAt(0,32);
  int Position = InBuffer.toInt();
  int eeAddress = Position*40;   //Location we want the data to be put.
  EEPROM.get(eeAddress, WayPoint);
  ReportWayPoint();
}

void ReportWayPoint() {
  Serial.print("WayPoint:");
  Serial.print(WayPoint.Number);
  Serial.print("!");
  Serial.print(WayPoint.Command);
  Serial.print("!");
  Serial.print(WayPoint.A);
  Serial.print("!");
  Serial.print(WayPoint.B);
  Serial.print("!");
  Serial.print(WayPoint.C);
  Serial.print("!");
  Serial.print(WayPoint.D);
  Serial.print("!");
  Serial.print(WayPoint.E);
  Serial.print("!");
  Serial.print(WayPoint.F);  
  Serial.println("!");
}

void MoveToAWayPointAngle() {
  InBuffer.setCharAt(0,32);
  int Position = InBuffer.toInt();
  Serial.print("Goto ");
  MoveToWayPointAngle(Position);
}

void MoveToWayPointAngle(int Position) {
  if (Position > 0) {
    int eeAddress = Position*40;   //Location we want the data to be put.
    EEPROM.get(eeAddress, WayPoint);
    ReportWayPoint();    
    for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){
      float Angle = 0;
      switch (iMotor) {
        case MOTOR_ID_F: Angle = WayPoint.F;
          break;
        case MOTOR_ID_E: Angle = WayPoint.E;
          break;
        case MOTOR_ID_D: Angle = WayPoint.D;
          break;
        case MOTOR_ID_C: Angle = WayPoint.C;
          break;
        case MOTOR_ID_B: Angle = WayPoint.B;
          break;
        case MOTOR_ID_A: Angle = WayPoint.A;
      }      
      mm6_set_angle(iMotor, Angle);  
    }  
  }  
}

void RunMotorsForAsmTest() {
  Serial.println("Run Motors For Asm Test");    
  mm6_pid_enable(false);
  delay(250);

  // Set all motor power lines to High-Z state by 
  // Setting speed to 0 and turning off brakes.  
  Serial.println("Setting Drive Power for all Motors to High-Z.");
  for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){    
    digitalWrite(motor_pinout[iMotor].out_brake, LOW); // Turn the Brakes off.
    motor_state[iMotor].speed = 0;
    mm6_set_pwm(iMotor);
  }  

  int MDir = 1;
  for (int z=0; z<10000; z++){
    for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){    
      MotorTestFullSpeed(iMotor, MDir);
    }
    delay(5000);               // Run for 5 sec.
    for (int iMotor = MOTOR_ID_FIRST; iMotor <= MOTOR_ID_LAST; iMotor++){    
      MotorTestStop(iMotor);
    }
    delay(100);                // Short Delay to allow the motor to stop.  
    MDir = MDir * -1;
    Serial.println("Reverse.");
  }
}

void MotorTestStop(int m) {
  motor_state[m].speed = 0;          // Turn off motor.
  mm6_set_pwm(m);            // Set Motor PWM.
}

void MotorTestFullSpeed(int m, int MDir) {
  int TestSpeed = 255 - motor_min_speed;
  motor_state[m].speed = TestSpeed*MDir; // Set Motor Speed.
  mm6_set_pwm(m);                // Set Motor PWM.
}

// Change log
// 1.02  3/19/2017 line 352: Excluded Motor A from the stall detect routine because it is the gripper and closing on an item is technically a stall.
// 1.03  3/31/2017 line 792: Added TestMotors routine to test motors that are in an electrically unknown state.
// 1.04  4/01/2017 line 792: Changed TestMotors routine so that all motor Drives are High-Z except the ones under test.
// 1.05  7/24/2017 Line 235: Changed the Letter case when Sending Position vs Angle.
// 1.05  7/24/2017 Line 760/763 - removed the 300 point adjustment when Interrogate Limit Switches.
// 1.06  7/25/2017 Added Waypoints.
// 1.07  7/26/2017 Added Version reporting.
// 1.08  7/31/2017 Added Status check on Waypoints so that Waypoints will run ONLY if Waypoints have actually been written to the MegaMotor6.
// 1.09  8/02/2017 Changed the boot up message.
// 1.10  8/25/2017 Added ++ and -- commands.
// 1.11  4/28/2018 Added Exercise command. Ax - Fx.
// 1.12 10/19/2018 Added ? Command to Print command list.
// 1.12 10/19/2018 Added X Command to Set Home To Center Of Switches.
// 1.13 11/09/2018 Added R/r Commands to set Rhino Ver.
// 1.13 11/09/2018 Added ~ Command to store the angle offsets.
// 1.14 11/12/2018 Added Motor Reverse settings. 
// 1.14 11/12/2018 Added switch settings to tracking info.
// 1.15 11/13/2018 Added Motor A to the InterrogateLimitSwitches routine.
// 1.16 11/17/2018 Added logic for a Home Button.
// 1.17 01/12/2020 Added logic to run motors for assembly test.  RunMotorsForAsmTest()
// 2.00 06/20/2021 Reworked by petermcd1010. 
