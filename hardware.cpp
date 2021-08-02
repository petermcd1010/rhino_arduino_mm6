/*
 * Implementation for non-MegaMotor6-specific hardware functionality.
 */

#define __ASSERT_USE_STDERR
#include <assert.h>
#include <stdlib.h>
#include <EEPROM.h>
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "mm6.h"

static const int OPRLED = 13;
static const int expansion_io_pinout[] = { A15, A14, A13, A12, 53, 49, 48, 41 };

void hardware_init() {
  pinMode(OPRLED, OUTPUT);
  pinMode(expansion_io_pinout[0], OUTPUT); // Tone

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
    // LOG_D(F("%d"), motor_state[i].logic);
  }  
}

void hardware_erase_eeprom()
{
  log_write(F("Erasing EEPROM... "));
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);    
  }
  log_writeln(F("Completed. Zeroed %d bytes."), EEPROM.length());
}

void hardware_reset()
{
  mm6_enable_all(false);
  hardware_erase_eeprom();
  hardware_reboot();
}

void (*hardware_really_reboot)(void) = 0;  // Call hardware_really_reboot() to reset the board.
void hardware_reboot() 
{
  mm6_enable_all(false);
  delay(1000);  // Wait 1s for log output to complete writing.
  hardware_really_reboot();
}

bool hardware_get_led()
{
  return digitalRead(OPRLED) != 0;
}

void hardware_set_led(bool enable)
{
  digitalWrite(OPRLED, enable);
}

