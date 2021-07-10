// ===============================
// MegaMotor6 Controller Program
// For Rhino Robot XR1 to XR4
// Written by Scott Savage
// Feb 2017-20 GNU General Public License (GPL)
// ===============================
#define Ver 1.17 // Change log at end of file (Command "N" will read this number.)
#define VerDate "(01/12/2020)"
// ===============================

#include <EEPROM.h>


//*************************************
// Define Motor Designators
// Note: These Designators are used to select an element 
//       from the arrays that hold motor information.
// -AND- nearly every array is used this way. 
//*************************************
#define MotorA 0
#define MotorB 1
#define MotorC 2
#define MotorD 3
#define MotorE 4
#define MotorF 5

//*************************************
// Define I/O lines
//*************************************
int Motor_IO_DIR[] = {11 ,A9 ,3  ,A1 ,17 ,51 }; // Motor Direction
int Motor_IO_PWM[] = {10 ,7  ,5  ,8  ,2  ,9  }; // Motor Speed (PWM)
int Motor_IO_BRK[] = {12 ,39 ,4  ,A2 ,16 ,50 }; // Motor Brake
int Motor_IO_CUR[] = {A5 ,A0 ,A6 ,A4 ,A7 ,A10}; // Motor Current Draw
int Motor_IO_THR[] = {14 ,A11,6  ,A3 ,15 ,52 }; // Motor Thermal Overload
int Motor_IO_LIM[] = {A8 ,26 ,28 ,30 ,40 ,38 }; // Motor Limit/Home Switch
int Motor_IO_QEA[] = {47 ,32 ,45 ,34 ,43 ,36 }; // Quadrature Encoder Channel A
int Motor_IO_QEB[] = {46 ,33 ,44 ,35 ,42 ,37 }; // Quadrature Encoder Channel B
int Expansion_IO[] = {A15,A14,A13,A12,53,49,48,41}; // Expansion Lines 
int Home_IO = 51;
#define ESERT 18 // Serial Transmit
#define ESERR 19 // Serial Receive
#define EI2CD 20 // I2C Data
#define EI2CC 21 // I2C Clock
#define OPRLED 13 // Operate LED

//*************************************
// Motor Status / Motion Control Vars.
//*************************************
int DoPid = 0;
int Motion_Status[] = {0,0,0,0,0,0}; // Motion Status: 
int SyncMove_Status = 0;
#define AtTarget 0
#define BesideTarget 1 // Within 1 click.
#define CloseToTarget 2 // between 2 and 30 clicks.
#define OnApproachToTarget 3 // between 30 and 200 clicks.
#define OnWayToTarget 4 // More than 200 clicks away.
int Motor_Last_Dir[] = {0,0,0,0,0,0}; // Last Move Direction
int Motor_Encoder[6] __attribute__ ((section (".noinit"))); // Current Motor Encoder Positions.
int Target_Encoder[] = {0,0,0,0,0,0}; // Target Motor Positions
int Motor_PID[] = {0,0,0,0,0,0}; // PID on or off.
int Motor_Speed[] = {0,0,0,0,0,0}; // Current Speed 
int Target_Speed[] = {0,0,0,0,0,0}; // Target Speed
int Motor_Current[] = {0,0,0,0,0,0}; // Motor Current Consumption.
int Current_PWM[] = {0,0,0,0,0,0}; // Current PWM
int PID_PError[] = {0,0,0,0,0,0}; // Proportional Error (Difference between Current and Target)
int PID_DValue[] = {0,0,0,0,0,0}; // 
int Limit_Prev[] = {0,0,0,0,0,0}; // Limit/Home switch Previous Value
int Limit_Bounce[] = {0,0,0,0,0,0}; // Limit/Home switch Previous Value (for debounce)
int Limit_For_On[] = {0,0,0,0,0,0}; // Limit/Home switch High Value
int Limit_For_Off[] = {0,0,0,0,0,0}; // Limit/Home switch High Value
int Limit_Rev_On[] = {0,0,0,0,0,0}; // Limit/Home switch Low Value
int Limit_Rev_Off[] = {0,0,0,0,0,0}; // Limit/Home switch Low Value
int QE_Prev[6] __attribute__ ((section (".noinit"))); ; // Quadrature Encoder Previous Read. Note that his is NOT reset to 0 when the CPU is reset.
int QE_Inc_States[] = {1, 3, 0, 2};
int QE_Dec_States[] = {2, 0, 3, 1};
int Forward_Logic[] = {0,0,0,0,0,0}; // Forward Logic - The value for the Direction IO Line when the motor needs to move forward to sync with encoders.
int Reverse_Logic[] = {1,1,1,1,1,1}; // Reverse Logic - The value for the Direction IO Line when the motor needs to move Reverse to sync with encoders.
int Motor_Logic[] = {0,0,0,0,0,0}; // Reverse Logic - The value for inverting the Position when the motor needs to run in Reverse.
int P1[] = {0,0,0,0,0,0}; // Testing Positions
int P2[] = {0,0,0,0,0,0}; // Testing Positions
int P3[] = {0,0,0,0,0,0}; // Testing Positions
int Start[] = {0,0,0,0,0,0};
int End[] = {0,0,0,0,0,0};
int Distance[] = {0,0,0,0,0,0};
float Ratio[] = {0,0,0,0,0,0};
#define MinSpeed 55
#define MaxError (255-MinSpeed)
#define MinError -(255-MinSpeed)
int Tracking = 0;
int Tracked[] = {0,0,0,0,0,0}; // Last Value send while tracking.
int DoSyncMove = 0;
int LeadMotor = 0;
int TravelSoFar = 0;
int TravelSoFarPrev = 0;
int tb = 0;
int tc = 0;
int td = 0;
int te = 0;
int tf = 0;
int RhinoType = 0;
int AngleOffset[] = {0,0,0,0,0,0}; // Used to adjust the physical positioning of arm's angles
#define Eloc_RobotType 4000
int AngleOffsetELoc[] = {4004,4008,4012,4016,4020,4024}; // EEPROM locations of AngleOffsets
int MotorLogicELoc[] = {4028,4032,4036,4040,4044,4048}; // EEPROM locations of MotorLogic
int DirLogicELoc[] = {4052,4056,4060,4064,4068,4072}; // EEPROM locations of MotorLogic
#define Gripper_OpenEloc 4076
#define Gripper_CloseEloc 4080
int Gripper_OpenLoc = -130;
int Gripper_CloseLoc = -310;
int Gripper_StallE = 0;
int Gripper_StallC = 0;
int Gripper_StallX = 0;
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

String InBuffer ="";
char padbuffer [50];
int Command_Motor = MotorE;
int savedFlag __attribute__ ((section (".noinit"))); 

//*************************************
// Initial Setup.
//*************************************
void setup(){
  
  pinMode(OPRLED, OUTPUT);
  pinMode(Expansion_IO[0], OUTPUT); // Tone
  pinMode(Home_IO, INPUT_PULLUP);
  InitMotorIO();
  
  //******************************************
  // TIMER SETUP - allows preceise timed 
  //  measurements of the Quadrature Encoder
  //******************************************
  cli();//stop interrupts
  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0;
  // set timer count for 2khz increments
  OCR1A = 1000;// = (16*10^6) / (2000*8) - 1
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);  
  sei();//allow interrupts

  // Zero out saved variables on power cycle.
  // Note: On reset, these values are NOT erased.
  if(savedFlag != 1234){  // 1234 is the magic number used to indicate valid noinit data
    for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
      Motor_Encoder[iMotor]=0;
      QE_Prev[iMotor]=0;
    }
    savedFlag = 1234; // Set savedFlag to the magic number.
  }

  //********************************************************
  // Get the Angle Offsets and Forward_Logic for ALL motors
  //********************************************************
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){    
    int Logic = 0;
    
    // The Angle Offset is used in the Position-to-Angle and Angle-to-Position calculations
    //   Each Rhino Robot may have mechanical differences in the positions of the Home Siwthes, 
    //   So the encoder count when the arm is strigt up is stored as an "AngleOffset" so that the
    //   MegaMotor6 Angle Values will work the actual physical position of the robot
    //   while the Position Values work with positions relative to the home switches.
    //     The values for the AngleOffets come from the "~" Command.
    EEPROM.get(AngleOffsetELoc[iMotor], AngleOffset[iMotor]);

    // The Forward and Reverse Locic is used to turn the motors in the right direction to sync with the encoders.
    //   Each Rhino Robot may have the wires to the motors reversed.
    //   So the Forward and Reverse Locic is used to correct that.    
    //     The values for the Direction Logic come from the "m" command.
    //       Since the Forward and Reverse Locic are used for an I/O line the values are 0 or 1'
    EEPROM.get(DirLogicELoc[iMotor], Logic);
    if (Logic!=0) Logic=!0;  
    Reverse_Logic[iMotor] = Logic; // Reverse Logic - The value for the Direction IO Line when the motor needs to move Reverse. Defaults to 1.
    Forward_Logic[iMotor] = !Logic; // Forward Logic - The value for the Direction IO Line when the motor needs to move forward. Defaults to 0.
    
    // The Motor Locic is used to contol which way the motors turn in responce to the Positions.
    //   Each Rhino Robot may have the motor assembled on either side - which winds up reversing the motors direction mechanically.
    //   So the Moto Locic is used to correct that.    
    //     The values for the Motor Logic are set by the setup.
    //       Since the Forward and Reverse Locic are used to invert the position the values are 1 or -1'
    EEPROM.get(MotorLogicELoc[iMotor], Logic);
    if (Logic!=1) Logic=-1;
    Motor_Logic[iMotor] = Logic; // Motor Logic - The value for inverting the Position when the motor needs to move Reverse. 
  }
  //******************************************
  // Get the Type of Rhino Robot being used.
  // XR1, XR2, XR3, or XR4
  //******************************************
  EEPROM.get(Eloc_RobotType, RhinoType);
  //******************************************
  // Get the gripper open and close  
  //******************************************
  EEPROM.get(Gripper_OpenEloc, Gripper_OpenLoc);
  EEPROM.get(Gripper_CloseEloc, Gripper_CloseLoc);
  //******************************************
  // Show startup message
  //******************************************
  Serial.begin(38400);
  Serial.println("MegaMotor6 Controller Program");
  Serial.println("For Rhino Robot XR1 to XR4");
  ShowVerNo();
  if ((RhinoType>4) || (RhinoType<1)) {
    // If the EEPROM reported something other than 1 -4,
    //  then the EEPROM data wasn't set yet.
    // Set EEPROM RhinoType to XR4.
    // AND set all EEPROM Angle Offsets to 0.
    EEPROMSetRhinoType(4);
    EEPROMSetAngleOffsets(0,0,0,0,0);
  } else {
    Serial.print("Configured for Rhino XR");
    Serial.println(RhinoType);
  }
  ShowPositions();
  ShowHelp();
  Serial.println("Ready");  

}

//*************************************
// Main 
//*************************************
int lMotor = 0;
void loop(){
  lMotor++; if (lMotor==6) lMotor=0;// Move to the next motor  
  Motor_Current[lMotor] = analogRead(Motor_IO_CUR[lMotor]);

  if (Tracking>0) {
    TrackReport(lMotor);
  } 

/*  int HomeButton = digitalRead(Home_IO);
  if (HomeButton == 0) {
    for (int hMotor=MotorA; hMotor<=MotorF; hMotor++){
      //Target_Encoder[hMotor]=0;
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
    Target_Encoder[MotorA] = Motor_Encoder[MotorA];
    Motor_Current[MotorA]=0;
  }

  if (Serial.available()){
    int a= Serial.read();// read the incoming data as string    
    if (a==13){
      Serial.print(InBuffer + ":");
      if (InBuffer=="+"){
        int Position = Motor_Position(Command_Motor)+10;
        MoveMotorToP(Command_Motor, Position);
      } else if (InBuffer=="++"){
        int Position = Motor_Position(Command_Motor)+100;
        MoveMotorToP(Command_Motor, Position);
      } else if (InBuffer=="-"){
        int Position = Motor_Position(Command_Motor)-10;
        MoveMotorToP(Command_Motor, Position);
      } else if (InBuffer=="--"){
        int Position = Motor_Position(Command_Motor)-100;
        MoveMotorToP(Command_Motor, Position);
      } else if ((InBuffer=="G") || (InBuffer=="g")){
        TurnOnPID();        
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
          EEPROMSetRhinoType(4);  
        } else {
          EEPROMSetRhinoType(3);  
        }        
      } else if ((InBuffer=="S") || (InBuffer=="s")){
        TurnOffPID();
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
        ShowVerNo();
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
              int Position = Motor_Position(Command_Motor)-10;
              MoveMotorToP(Command_Motor, Position);              
            } else if (WhatToDo=="--") {
              int Position = Motor_Position(Command_Motor)-100;
              MoveMotorToP(Command_Motor, Position);              
            } else if (WhatToDo=="+") {
              int Position = Motor_Position(Command_Motor)+10;
              MoveMotorToP(Command_Motor, Position);              
            } else if (WhatToDo=="++") {
              int Position = Motor_Position(Command_Motor)+100;
              MoveMotorToP(Command_Motor, Position);              
            } else {              
              int Position = WhatToDo.toInt();
              MoveMotorToP(Command_Motor, Position);
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
            MoveMotorToAngle(Command_Motor, Angle);
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
      InBuffer = InBuffer + char(a);
    }
  }
}

//*****************************************
// Print Version.
//*****************************************
void ShowVerNo() {
  Serial.print("Version: ");
  Serial.print(Ver);  
  Serial.print(" ");
  Serial.println(VerDate);  
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
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    Tracked[iMotor]=32000;
  }
}

void StopTracking(){
  Tracking=0; // Track none
  Serial.println("Tracking Off");
}

int Motor_Position(int tMotor){
  return Motor_Encoder[tMotor] * Motor_Logic[tMotor];
}
 
void TrackReport(int tMotor){
  if (Tracking>0){
    int Position = Motor_Position(tMotor);
    if (Tracked[tMotor]!=Position) {
      Serial.print("@");
      if (Tracking==1) {
        Serial.print(char(tMotor+65));
        Serial.print(Motor_Position(tMotor));    
      } else if (Tracking==2) {
        Serial.print(char(tMotor+97));
        Serial.print(Motor_Angle(tMotor));    
      }    
      Serial.print(":HS");
      Serial.print(Limit_Prev[tMotor]);
      Serial.println(":");
      Tracked[tMotor]=Position;
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
  return ((Motion_Status[MotorA] == 0) && 
          (Motion_Status[MotorB] == 0) && 
          (Motion_Status[MotorC] == 0) && 
          (Motion_Status[MotorD] == 0) && 
          (Motion_Status[MotorE] == 0) && 
          (Motion_Status[MotorF] == 0));
}

//*******************************************************
// Set the I/O lines used by the LMD18200t Motor Driver
//*******************************************************
void InitMotorIO(){
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    pinMode(Motor_IO_BRK[iMotor], OUTPUT);
    pinMode(Motor_IO_DIR[iMotor], OUTPUT);
    pinMode(Motor_IO_PWM[iMotor], OUTPUT);
    pinMode(Motor_IO_LIM[iMotor], INPUT_PULLUP);
    pinMode(Motor_IO_THR[iMotor], INPUT_PULLUP);
    pinMode(Motor_IO_QEA[iMotor], INPUT_PULLUP);
    pinMode(Motor_IO_QEB[iMotor], INPUT_PULLUP);
    digitalWrite(Motor_IO_BRK[iMotor], HIGH); // Turn the Brakes on.
    digitalWrite(Motor_IO_DIR[iMotor], HIGH); // Set the direction to Forward.
    digitalWrite(Motor_IO_PWM[iMotor], LOW); // Set Speed to Zero.
  }  
}

//*******************************************************
// interrupt routine.
// Interrupts the main program at freq of 2kHz 
//*******************************************************
int intMotor = 0;
int LEDCounter = 0;
ISR(TIMER1_COMPA_vect) {
  //==========================================
  // Quadrature Encoders - read at rate of 2kHz 
  //==========================================
  for (int qMotor=MotorA; qMotor<=MotorF; qMotor++){
    int Limit_Value = digitalRead(Motor_IO_LIM[qMotor]);
    int QE_Value_A = digitalRead(Motor_IO_QEA[qMotor]);
    int QE_Value_B = digitalRead(Motor_IO_QEB[qMotor]) * 2;
    int QE_State = QE_Value_A + QE_Value_B;
    if (QE_State == QE_Inc_States[QE_Prev[qMotor]]) {
      Motor_Last_Dir[qMotor] = 1;
      Motor_Encoder[qMotor]++;
      PID_DValue[qMotor]=0;
    } else if (QE_State == QE_Dec_States[QE_Prev[qMotor]]) {
      Motor_Last_Dir[qMotor] = -1;
      Motor_Encoder[qMotor]--;      
      PID_DValue[qMotor]=0;
    } else {
      PID_DValue[qMotor]++;
      if (PID_DValue[qMotor]>10000) {
        PID_DValue[qMotor]=10000;
      }
    }    
    QE_Prev[qMotor] = QE_State;

    //==========================================================
    // See if Limit/Home switch has changed from on to off or off to on.
    // There are 4 different motor positions stored for the Home Switches.
    // -The On and Off locations when the motor is moving forward.
    // -The On and Off locations when the motor is moving reverse.
    // The average value of these 4 positions is used as the center of "Home".
    //==========================================================
    if (Limit_Bounce[qMotor] == Limit_Value) {
      if (Limit_Value != Limit_Prev[qMotor]) {
        Limit_Prev[qMotor] = Limit_Value;
        if (Motor_Last_Dir[qMotor] == 1) {
          if (Limit_Value==0) {
            Limit_For_On[qMotor] = Motor_Encoder[qMotor]; 
          } else {
            Limit_For_Off[qMotor] = Motor_Encoder[qMotor]; 
          }        
        }      
        if (Motor_Last_Dir[qMotor] == -1) {
          if (Limit_Value==0) {
            Limit_Rev_On[qMotor] = Motor_Encoder[qMotor];
          } else {
            Limit_Rev_Off[qMotor] = Motor_Encoder[qMotor];
          }                
        }      
      }    
    }
    Limit_Bounce[qMotor] = Limit_Value;
  }

  //==========================================================
  // Blink the Operate LED
  //==========================================================
  LEDCounter++; // LEDCounter
  if (DoPid){
    // if the PID is on, then blink faster.
    if (!digitalRead(OPRLED)){
      LEDCounter+=4; // Count the Interrupts         
    }
    LEDCounter++; // Count the Interrupts   
  }
  if (LEDCounter>1000){
    LEDCounter-=1000;
    bool LEDState = !digitalRead(OPRLED);
    digitalWrite(OPRLED, LEDState);
    // Optional - Output the LED onto Expansion_IO 1.
    // This can be used to drive a speaker so 
    // that one can hear the PID status.
    digitalWrite(Expansion_IO[0], LEDState);
  }  

  //==========================================================
  // Calculate Motor status values.
  //==========================================================
  // Calculate only one motor per interupt - rate of 333Hz
  // This is done by stepping intMotor once per interupt. 
  // intMotor is then used to specifiy which motor to do 
  // calculations on.
  //==========================================================
  intMotor++; if (intMotor==6) intMotor=0;// Move to the next motor

  //==========================================================
  // See if the Motor PID needs to be turned on.
  if (Motor_PID[intMotor]==1230) {
    // The IntMotor PID is NOT running.
    // ( =1230 instead of =0 is just to kill the routine for now because it isn't doing what it needs to do.)
    PID_DValue[intMotor]=0; //Clear the PID DValue for this motor
    int intMDiff = abs(Target_Encoder[intMotor] - Motor_Encoder[intMotor]);
    if (intMDiff>3) {
      Motor_PID[intMotor]=1; // Turn the PID On for this Motor.
    }
  } else {
    // Brakes are not on - so the IntMotor is running.
    //==========================================================
    // Check for stall.
    // Note that the Gripper is excluded because it needs to 
    // apply tension on whatever it is gripping.
    //==========================================================
    if (intMotor>MotorA){    
      // For Motors other than the gripper, High Current means 
      // that the motor is in a stall situation.  To unstall,
      // the target position is set back a bit from the
      // current position.      
      
      if (Motor_Current[intMotor] > 200) {
        if (Motor_Last_Dir[intMotor]==1) {
          Target_Encoder[intMotor] = Motor_Encoder[intMotor] - 50;
          Motor_Current[intMotor]=0;
        } else if (Motor_Last_Dir[intMotor]==-1) {
          Target_Encoder[intMotor] = Motor_Encoder[intMotor] + 50;
          Motor_Current[intMotor]=0;
        }
      }
           
    } else {
      if (Motor_Current[intMotor] > 150) {
        // Motor A is a special case where High Current 
        // means that the Gripper is gripping someting.
        // Set gripper tension on MotorA by setting
        // the Target Position to the Currernt Position.
        // This will cause the PWM to drop to off.
        //   AND if the relaxed gripper opens a little,
        //     it will turn back on but at a much lower
        //       PWM duty cycle.
        Gripper_StallC = Motor_Current[intMotor];
        Gripper_StallE = Motor_Encoder[intMotor];
        Gripper_StallX++;
      }
    }
  
    //==========================================================
    // Calculate PID Proportional Error
    //==========================================================
    int PIDPError = (Target_Encoder[intMotor]-Motor_Encoder[intMotor]);
    PID_PError[intMotor] = PIDPError; // Save
      
    //==========================================================
    // Calc the Target Speed from the Proportional Error
    // The target speed is just the differnce between the 
    //  Current Position and the Target Position (with limits).
    // Results in a speed of +/- 255.
    //==========================================================
    if (PIDPError > MaxError) {
      Target_Speed[intMotor] = MaxError;
        // Set the Status that indicates that the Motor is more than 200 clicks from target.
        Motion_Status[intMotor] = OnWayToTarget;      
      
    } else if (PIDPError < MinError) {
      Target_Speed[intMotor] = MinError;
        // Set the Status that indicates that the Motor is more than 200 clicks from target.
        Motion_Status[intMotor] = OnWayToTarget;      
      
    } else if (PIDPError > 0) {
      Target_Speed[intMotor] = PID_PError[intMotor]+(PID_DValue[intMotor]/6);
      if (PIDPError < 2) {
        // Set the Status that indicates that the Motor is 1 click from target
        Motion_Status[intMotor] = BesideTarget;  
      } else if (PIDPError < 30) {
        // Set the Status that indicates that the Motor is 2-29 clicks from target
        Motion_Status[intMotor] = CloseToTarget;
      } else {
        // Set the Status that indicates that the Motor is 30-200 clicks from target
        Motion_Status[intMotor] = OnApproachToTarget;
      }
      
    } else if (PIDPError < 0) {
      Target_Speed[intMotor] =PID_PError[intMotor]-(PID_DValue[intMotor]/6),-255;
      if (PIDPError > -2) {
        // Set the Status that indicates that the Motor is 1 click from target
        Motion_Status[intMotor] = BesideTarget;  
      } else if (PIDPError > -30) {
        // Set the Status that indicates that the Motor is 2-29 clicks from target
        Motion_Status[intMotor] = CloseToTarget;
      } else {
        // Set the Status that indicates that the Motor is 30-200 clicks from target
        Motion_Status[intMotor] = OnApproachToTarget;
      }
      
    } else {
      Target_Speed[intMotor] = 0; 
      Motion_Status[intMotor] = AtTarget;  // Clear the flag that indicates that the Motor is in motion.
      //Motor_PID[intMotor]=0; // Turn intMotor's PID off.      
    }
  
    //==========================================================
    // PID (Currenty Just the P)
    //==========================================================
    if (DoPid){
  
      //============================================
      // Ramp Up/Down Current Speed to Target Speed.
      // Prevents the motors from jumping from dead 
      //  stop to full speed and vice-versa
      //============================================
      int CurrentSpeedChanged = 0;
      if (Target_Speed[intMotor]>Motor_Speed[intMotor]) {
        if (Motor_Speed[intMotor]<(255-MinSpeed)){
          Motor_Speed[intMotor]++; // if the target is higher, then inc up to the target.
          if (Target_Speed[intMotor]>Motor_Speed[intMotor]) {
            if (Motor_Speed[intMotor]<(255-MinSpeed)){
              Motor_Speed[intMotor]++; // if the target is higher, then inc up to the target a second time.
            }
          }
          CurrentSpeedChanged = 1;
        }
      } else if (Target_Speed[intMotor]<Motor_Speed[intMotor]) {
        if (Motor_Speed[intMotor]>-(255-MinSpeed)){
          Motor_Speed[intMotor]--; // if the target is lower, then inc down to the target.
          if (Target_Speed[intMotor]<Motor_Speed[intMotor]) {
            if (Motor_Speed[intMotor]>-(255-MinSpeed)){
              Motor_Speed[intMotor]--; // if the target is lower, then inc down to the target a second time.
            }
          }
          CurrentSpeedChanged = 1;
        }
      }
  
      if (CurrentSpeedChanged == 1){
        SetMotorPWM(intMotor);    
      }        
    }
  
    //==========================================================
    // Sync Move.
    // as it stands, the synchronized move does not work well
    // with the current PID.  
    // The current PID only regulates encoder position.
    // For the Sync Move to work well, it needs to regulate speed.
    //==========================================================
    if (DoSyncMove>0) {    
      TravelSoFar =abs(Motor_Encoder[LeadMotor]-Start[LeadMotor]+1);
      if (TravelSoFar != TravelSoFarPrev) {
        TravelSoFarPrev = TravelSoFar;
        float TravelSoFarFloat = TravelSoFar;
        for (int intMotor=MotorB; intMotor<=MotorF; intMotor++){
          if (intMotor!=LeadMotor){
            float RP = TravelSoFarFloat*Ratio[intMotor];
            int RI = int(RP);
            int TG = Start[intMotor]+RI;
            Target_Encoder[intMotor]=Start[intMotor]+RI;
          }
        } 
        tb = abs(End[MotorB]-Motor_Encoder[MotorB]);
        tc = abs(End[MotorC]-Motor_Encoder[MotorC]);
        td = abs(End[MotorD]-Motor_Encoder[MotorD]);
        te = abs(End[MotorE]-Motor_Encoder[MotorE]);
        tf = abs(End[MotorF]-Motor_Encoder[MotorF]);            
        if ((tb==0) && (tc==0) && (td==0) && (te==0) && (tf==0)) {
          // Set the Status that indicates that the Motor is 1 click from target
          SyncMove_Status = AtTarget;
          DoSyncMove = 0;
        } else if ((tb<2) && (tc<2) && (td<2) && (te<2) && (tf<2)) {
          SyncMove_Status = BesideTarget;
        } else if ((tb<30) && (tc<30) && (td<30) && (te<30) && (tf<30)) {
          // Set the Status that indicates that the Motor is 2-29 clicks from target
          SyncMove_Status = CloseToTarget;
        } else if ((tb<200) && (tc<200) && (td<200) && (te<200) && (tf<200)) {
          // Set the Status that indicates that the Motor is 2-29 clicks from target
          SyncMove_Status = OnApproachToTarget;
        } else {
          // Set the Status that indicates that the Motor is 30-200 clicks from target
          SyncMove_Status = OnWayToTarget;
        }        
      }
    }
  }
}

void SetMotorPWM(int m){
  //==========================================================
  // Calculate the PWM and Direction for the Speed
  // Converts the speed's +/- 255 value to PWM and Direction.
  //==========================================================  
  if (Motor_Speed[m] > 0) {      
    Current_PWM[m] = Motor_Speed[m]+MinSpeed;
    analogWrite(Motor_IO_PWM[m],Current_PWM[m]);  
    digitalWrite(Motor_IO_DIR[m],Forward_Logic[m]);          
  } else if (Motor_Speed[m] < 0) {      
    Current_PWM[m] = abs(Motor_Speed[m])+MinSpeed;
    analogWrite(Motor_IO_PWM[m],Current_PWM[m]);  
    digitalWrite(Motor_IO_DIR[m],Reverse_Logic[m]);        
  } else {
    Current_PWM[m] = 0;
    analogWrite(Motor_IO_PWM[m],0);          
  }  
}

void ZeroPositions() {
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    Motor_Encoder[iMotor] = 0;
    Target_Encoder[iMotor] = 0;
  }
  Serial.println("Current Positions set to Zero.");
}

void SetHomeToCenterOfSwitches() {
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    SetNewHome(iMotor);
  }
  Serial.println("Home Positions set to Center Of Switches.");
}

void ShowPositions() {
  Serial.print("Current Positions: ");
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    Serial.print(char(iMotor+65));
    Serial.print("=");
    Serial.print(Motor_Position(iMotor));
    if (iMotor<5) Serial.print(",");
  }
  Serial.println(".");  
}

void SetPositionToHome() {
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    Target_Encoder[iMotor] = 0;
  }  
  Serial.println("Setting Targets to Home Position.");
}

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

void TurnOffPID(){
  DoPid = 0;
  DoSyncMove = 0;
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    analogWrite(Motor_IO_PWM[iMotor],0); // Set the speed to 0.
    digitalWrite(Motor_IO_BRK[iMotor], LOW); // Turn the Brakes on.
  }
  Serial.println("Motors are now off.");
}

void TurnOnPID(){
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    digitalWrite(Motor_IO_BRK[iMotor], LOW); // Turn the Brakes off.
  }
  DoPid = 1;
  Serial.println("Motors are now on.");
}

void OpenGripper(){
  Gripper_StallX=0;
  Target_Encoder[MotorA] = Gripper_OpenLoc;
  Serial.println("Opening Gripper.");
}

void CloseGripper(){
  Gripper_StallX=0;
  Target_Encoder[MotorA] = Gripper_CloseLoc;
  Serial.println("Closing Gripper.");
}

void DisplayStatus(){  
  Serial.println("Motor Status Report");
  Serial.print("  PID: ");
  if (DoPid==1) {
    Serial.println("On");  
  } else {
    Serial.println("Off");  
  }
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
  Serial.print("  ");
  Serial.print(char(iMotor+65));
  Serial.print(": Home=");  
  Serial.print(Limit_Prev[iMotor]);
  Serial.print(" Sta=");
  Serial.print(Motion_Status[iMotor]); // Report whether or not the Motor has reached the target location.
  //Serial.print(" Pos=");
  //Serial_print(Motor_Position(iMotor),4);
  Serial.print(" Enc=");  
  Serial_print(Motor_Encoder[iMotor] * Motor_Logic[iMotor],4);
  Serial.print(" Tar=");  
  Serial_print(Target_Encoder[iMotor] * Motor_Logic[iMotor],4);
  Serial.print(" Err=");  
  Serial_print(PID_PError[iMotor] * Motor_Logic[iMotor],4);
  Serial.print(" Spd=");
  Serial_print(Motor_Speed[iMotor] * Motor_Logic[iMotor],3);
  //Serial.print(" TSpd=");
  //Serial.print(Target_Speed[m]);
  Serial.print(" PWM=");
  Serial_print(Current_PWM[iMotor],3);
  Serial.print(" Cur=");  
  Serial_print(Motor_Current[iMotor],3);
  Serial.print(" HS=");
  Serial_print(Limit_Rev_Off[iMotor],3);
  Serial.print(",");
  Serial_print(Limit_For_On[iMotor],3);
  Serial.print(",");
  Serial_print(Limit_Rev_On[iMotor],3);
  Serial.print(",");
  Serial_print(Limit_For_Off[iMotor],3);
  Serial.print(",");
  Serial.print((Limit_For_Off[iMotor] + Limit_Rev_On[iMotor] + Limit_For_On[iMotor] + Limit_Rev_Off[iMotor]) / 4);
  Serial.print(" Angle=");  
  Serial.print(Motor_Angle(iMotor));
  Serial.println(" ");   
  } 
}

//************************************************************
// Determine Encoder Steps Per Degree
//************************************************************
float EncoderStepsPerDegree(int Motor) {
  switch (Motor) {
    case MotorF: 
      if (RhinoType==4)
      {
        return 17.5; //(4.4)(66.1/1) XR4        
      } else {
        return 29.5; //(4.4)(66.1/1) XR3
      }
      break;
    case MotorE:
    case MotorD:
    case MotorC:
      if (RhinoType==4)
      {
        return 35; //(8.8)(66.1/1) XR4
      } else {
        return 36; //(8.8)(66.1/1) XR3
      }
      break;
    case MotorB:
      return 12.5; //(5.51)(165.4/1) XR4
      break;
    case MotorA:
      return 1;
  }  
}

//************************************************************
// Cacluation to convert Rhino Robot angles to motor position
//************************************************************
int AngleToPosition(int Motor, float Angle) {
  return (Angle * EncoderStepsPerDegree(Motor)) + AngleOffset[Motor];
}

//************************************************************
// Cacluation to convert Rhino Robot motor positions to angles
//************************************************************
float Motor_Angle(int zMotor) {  
  return (Motor_Position(zMotor) - AngleOffset[zMotor]) / EncoderStepsPerDegree(zMotor);
}

//**********************************************
// Move a single Motor to a specified position. 
//**********************************************
void MoveMotorToE(int zMotor, int Position) {
  Target_Encoder[zMotor] = Position;
  Motion_Status[zMotor] = OnWayToTarget; //Set the flag that indicates that the motor has been put in motion.
  Serial.print(" ->Move Motor ");
  Serial.print(char(zMotor+65));
  Serial.print(" to ");
  Serial.println(Position);  
}

//**********************************************
// Move a single Motor to a specified position. 
//**********************************************
void MoveMotorToP(int zMotor, int Position) {  
  Target_Encoder[zMotor] = Position * Motor_Logic[zMotor];
  Motion_Status[zMotor] = OnWayToTarget; //Set the flag that indicates that the motor has been put in motion.
  Serial.print(" ->Move Motor ");
  Serial.print(char(zMotor+65));
  Serial.print(" to ");
  Serial.println(Position);  
}

//**********************************************
// Move a single Motor to a specified angle. 
//**********************************************
void MoveMotorToAngle(int Motor, float Angle) {
  int Position = AngleToPosition(Motor, Angle);
  MoveMotorToP(Motor, Position);
}

//***********************************************************
// Move Motors in synchronous mode by specifying the angles. 
// (Except the Gripper)
//***********************************************************
void SyncMoveAngle(float AngleB, float AngleC, float AngleD, float AngleE, float AngleF) {
  int PositionB = AngleToPosition(MotorB, AngleB);
  int PositionC = AngleToPosition(MotorC, AngleC);
  int PositionD = AngleToPosition(MotorD, AngleD);
  int PositionE = AngleToPosition(MotorE, AngleE);
  int PositionF = AngleToPosition(MotorF, AngleF);
  SyncMove(PositionB, PositionC, PositionD, PositionE, PositionF);
}

//**************************************************************
// Move Motors in synchronous mode by specifying the positions. 
// (Except the Gripper)
//**************************************************************
void SyncMove(int PositionB, int PositionC, int PositionD, int PositionE, int PositionF) {
  DoSyncMove = 0;  
  SyncMove_Status = OnWayToTarget;
  // Store the target positions.
  End[MotorB]=PositionB;
  End[MotorC]=PositionC;
  End[MotorD]=PositionD;
  End[MotorE]=PositionE;
  End[MotorF]=PositionF;
  float MaxDistance = 0;
  // Caculate the travel distance for each motor.
  for (int iMotor=MotorB; iMotor<=MotorF; iMotor++){
    Start[iMotor]=Target_Encoder[iMotor];
    Distance[iMotor]=End[iMotor]-Start[iMotor];
    // Keep track of the furthest travel distance.
    MaxDistance = max(MaxDistance,abs(Distance[iMotor]));
  }
  // Using the motor with the furthest travel distance,
  // caculate the ratios of travel distance between all motors.
  for (int iMotor=MotorB; iMotor<=MotorF; iMotor++){
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
  Target_Encoder[LeadMotor] = End[LeadMotor];  
  SyncMove_Status = OnWayToTarget;
  DoSyncMove = 1;
  Serial.println("Start Sync Move");
}

void InterrogateLimitSwitches(){
  Serial.println("Interrogate Limit Switches");  
  Serial.print("  ");
  int CurrentMotorState = DoPid;  
  TurnOnPID();    
  InterrogateLimitSwitches2();
  Serial.println("  Done Interrogating Limit Switches");  
  if (CurrentMotorState == 0) {
    //TurnOffPID();
  }  
}

void Reverse(int m) {
  Motor_Encoder[m] = (Motor_Encoder[m]*-1);
  Target_Encoder[m] = (Target_Encoder[m]*-1);
  Motor_Logic[m]= (Motor_Logic[m]*-1);
  EEPROM.put(MotorLogicELoc[m], Motor_Logic[m]);
  Serial.print("Motor ");
  Serial.print(char(Command_Motor+65));    
  Serial.println(" reversed.");  
}

void Exercise(int m) {
  Serial.print("Exercise Motor ");    
  Serial.println(m);    
  int Rep = 0;
  TurnOnPID();  
  do {
      Rep++;
      Serial.print("Rep: ");    
      Serial.println(Rep);        
      switch (m)
      {
        case MotorF:; //A
          InterrogateLimitSwitch(MotorF, 450, -450);
          break;
        case MotorE:; //A
          InterrogateLimitSwitch(MotorE, 280, -280);
          break;
        case MotorD:; //A
          InterrogateLimitSwitch(MotorD, 320, -320);
          break;
        case MotorC:; //A
          InterrogateLimitSwitch(MotorC, 300, -300);
          break;
        case MotorB:; //A
          InterrogateLimitSwitch(MotorB, 380, -380);
          break;
        case MotorA:; //A
          Serial.println("Swing Gripper out.");
          Target_Encoder[MotorA] = 30;
          do {TrackReport(m);} while (Motor_Encoder[m] < 25);
          Serial.println("Swing Gripper in.");
          Target_Encoder[MotorA] = -280;
          do {TrackReport(m);} while (Motor_Encoder[m] > -275);  
          break;
      }   
      delay(500);
    } while (1);
}

void InterrogateLimitSwitches2(){
  InterrogateLimitSwitch(MotorF, 450, -450);
  InterrogateLimitSwitch(MotorE, 280, -280);
  InterrogateLimitSwitch(MotorD, 320, -320);
  InterrogateLimitSwitch(MotorC, 300, -300);
  InterrogateLimitSwitch(MotorB, 380, -380);
  InterrogateLimitSwitchA();
}

void InterrogateLimitSwitchA() {  
  MoveMotorToE(MotorA,9999);
  delay(2000);
  int CurF = analogRead(Motor_IO_CUR[MotorA]);
  int EncF = Motor_Encoder[MotorA];
  int SwcF = digitalRead(Motor_IO_LIM[MotorA]);
  MoveMotorToE(MotorA,-9999);
  delay(1500);
  int CurR = analogRead(Motor_IO_CUR[MotorA]);
  int EncR = Motor_Encoder[MotorA];
  int SwcR = digitalRead(Motor_IO_LIM[MotorA]);  
  MoveMotorToE(MotorA,9999);
  delay(1500);
  MoveMotorToE(MotorA,-9999);
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
    //Serial.print("  Limit_For_On=");
    //Serial.println(Limit_For_On[MotorA]);        
  if (SwcF==0) {
    // Encoder goes Positive towards switch.
    int OverSwitch = ((Limit_For_On[MotorA] + EncF ) / 2);      
    MoveMotorToE(MotorA,OverSwitch);
    do {TrackReport(MotorA);} while (Motor_Encoder[MotorA] != OverSwitch);
    Motor_Encoder[MotorA] = 0;  
    Target_Encoder[MotorA] = 0;
    Motor_Logic[MotorA] = -1;
    EEPROM.put(MotorLogicELoc[MotorA], Motor_Logic[MotorA]);
    Gripper_OpenLoc = -140;
    Gripper_CloseLoc = -310;    
    EEPROM.put(Gripper_OpenEloc, Gripper_OpenLoc);
    EEPROM.put(Gripper_CloseEloc, Gripper_CloseLoc);
    Serial.println("Done");
  } else {
    // Encoder goes Negative towards switch.
    int OverSwitch = ((Limit_Rev_On[MotorA] + EncR ) / 2);      
    MoveMotorToE(MotorA,OverSwitch);
    do {TrackReport(MotorA);} while (Motor_Encoder[MotorA] != OverSwitch);
    Motor_Encoder[MotorA] = 0;  
    Target_Encoder[MotorA] = 0;
    Motor_Logic[MotorA] = 1;
    EEPROM.put(MotorLogicELoc[MotorA], Motor_Logic[MotorA]);
    Gripper_OpenLoc = 140;
    Gripper_CloseLoc = 310;    
    EEPROM.put(Gripper_OpenEloc, Gripper_OpenLoc);
    EEPROM.put(Gripper_CloseEloc, Gripper_CloseLoc);
    Serial.println("Done");
  }
}

void InterrogateLimitSwitch(int m, int f, int r) {
  if (Limit_Prev[m]==0){
    //Serial.print(" Centering Motor ");
    //Serial.println(char(m+65));
    //Serial.print("   Moving to: ");
    Motor_Encoder[m] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
    Target_Encoder[m] = 0;
    
    // Move to one side of switch and wait for the switch to be unpressed.
    Serial.print("  ");
    MoveMotorToE(m,r-130);
    do {TrackReport(m);} while (Limit_Prev[m]==0);

    // Move to the other side of switch and wait for the switch to be pressed and then unpressed.
    Serial.print("  ");
    MoveMotorToE(m,f+130);
    do {TrackReport(m);} while (Limit_Prev[m]!=0);
    do {TrackReport(m);} while (Limit_Prev[m]==0);
    //do {TrackReport(m);} while (Motor_Position(m) < f);

    // Move back to first side of switch and wait for the switch to be pressed and then unpressed.        
    Serial.print("  ");
    MoveMotorToE(m,r-130);
    do {TrackReport(m);} while (Limit_Prev[m]!=0);
    do {TrackReport(m);} while (Limit_Prev[m]==0);
    //do {TrackReport(m);} while (Motor_Position(m) > r);

    // Calculate center of switches and then move to that place.
    int CenterEncoder = ((Limit_For_Off[m] + Limit_Rev_On[m] + Limit_For_On[m] + Limit_Rev_Off[m]) / 4);
    Serial.print("   Switch Positions: ");
    Serial.print(Limit_Rev_Off[m]);
    Serial.print(",");
    Serial.print(Limit_For_On[m]);
    Serial.print(",");
    Serial.print(Limit_Rev_On[m]);
    Serial.print(",");
    Serial.print(Limit_For_Off[m]);
    Serial.println(".");
    Serial.print("     Centering");    
    MoveMotorToE(m,CenterEncoder);
    do {TrackReport(m);} while (Motor_Encoder[m] != CenterEncoder);

    // Set Encoder and Target Values to 0.
    Motor_Encoder[m] = 0;  // The Home Switch was pressed, so assume the encoder is at 0.
    Target_Encoder[m] = 0;
    
  } else {
    Serial.print(" Motor ");
    Serial.print(char(m+65));
    Serial.println(" Home Switch Not Closed. - Skipping.");
  }
}

void SetNewHome(int m) {
    int NewHome = ((Limit_For_Off[m] + Limit_Rev_On[m] + Limit_For_On[m] + Limit_Rev_Off[m]) / 4);
    MoveMotorToE(m,NewHome);
    do {TrackReport(m);} while (Motor_Encoder[m] != NewHome);
    Motor_Encoder[m] = 0;
    Target_Encoder[m] = 0;
    Serial.print(" Motor ");
    Serial.print(char(m+65));
    Serial.println(" Centered");  
}

void TestMotors() {
  Serial.println("Test Motors");    
  TurnOffPID();
  delay(250);

  ShowPositions();

  // Set all motor power lines to High-Z state by 
  // Setting speed to 0 and turning off brakes.  
  Serial.println("Setting Drive Power for all Motors to High-Z.");
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){    
    digitalWrite(Motor_IO_BRK[iMotor], LOW); // Turn the Brakes off.
    Motor_Speed[iMotor] = 0;
    SetMotorPWM(iMotor);
  }  
  
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){    
    TestMotor(iMotor);
    if (((P2[iMotor]-P1[iMotor]) >0 ) && ((P3[iMotor]-P2[iMotor])<0)) {
      Serial.print("Reversing Direction Logic on Motor ");
      Serial.println(char(iMotor+65));
      Forward_Logic[iMotor] = !Forward_Logic[iMotor];
      Reverse_Logic[iMotor] = !Reverse_Logic[iMotor];
      EEPROM.put(DirLogicELoc[iMotor], Reverse_Logic[iMotor]);
      iMotor=iMotor-1; // Test motor[iMotor] again.
    }    
  }

  Serial.print("Forward encoder count");    
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    Serial.print(" : ");
    Serial.print(char(iMotor+65));
    Serial.print("=");
    Serial_Print_Pos(P3[iMotor]-P2[iMotor]);
  } 
  Serial.println("."); 
  
  Serial.print("Reverse encoder count");
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
    Serial.print(" : ");
    Serial.print(char(iMotor+65));
    Serial.print("=");
    Serial_Print_Pos(P2[iMotor]-P1[iMotor]);
  } 
  Serial.println("."); 

  /* Auto Correct Reversed Motors.
  for (int iMotor=MotorA; iMotor<=MotorF; m++){
    if (((P2[iMotor]-P1[iMotor]) >0 ) && ((P3[iMotor]-P2[iMotor])<0)) {
      Serial.print("Reversing Direction Logic on Motor ");
      Serial.println(char(m+65));
      Forward_Logic[iMotor] = !Forward_Logic[iMotor];
      Reverse_Logic[iMotor] = !Reverse_Logic[iMotor];
    }    
  } 
  */
  
  ShowPositions();
  Serial.println("Done Testing Motors");  
}

void TestMotor(int m) {
  int TestSpeed = 255-MinSpeed;
  int SpeedDelay = 50;
  Serial.print("Moving Motor ");
  Serial.println(char(m+65));
  Serial.print("   backward");
  
  P1[m] = Motor_Encoder[m]; // Get Current Position.
  
  Motor_Speed[m]=-TestSpeed; // Set Motor Speed.
  SetMotorPWM(m);            // Set Motor PWM.
  Serial.print(" On");
  delay(SpeedDelay);         // Short Delay to allow the motor to move.  
    
  Motor_Speed[m]=0;          // Turn off motor.
  SetMotorPWM(m);            // Set Motor PWM.
  Serial.println(" Off");
  delay(SpeedDelay);         // Short Delay to allow the motor to stop.  

  P2[m] = Motor_Encoder[m]; // Get Current Position.

  Serial.print("   forward ");
  //Serial.print("Moving Motor ");
  //Serial.print(char(m+65));
  //Serial.print(" forward ");

  Motor_Speed[m]=TestSpeed;  // Set Motor Speed.
  SetMotorPWM(m);            // Set Motor PWM.
  Serial.print(" On");
  delay(SpeedDelay);         // Short Delay to allow the motor to move.  
    
  Motor_Speed[m]=0;          // Turn off motor.
  SetMotorPWM(m);            // Set Motor PWM.
  Serial.println(" Off");
  delay(SpeedDelay);         // Short Delay to allow the motor to stop.  

  P3[m] = Motor_Encoder[m]; // Get Current Position.

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
  TurnOffPID();
  OpenGripper();
  MoveMotorToAngle(MotorB,90);
  MoveMotorToAngle(MotorE,-130);
  MoveMotorToAngle(MotorF,45);
  TurnOnPID();
  do {delay(50);} while (Motion_Status[MotorE] > OnApproachToTarget); 
  MoveMotorToAngle(MotorD,17.1);
  do {delay(50);} while (Motion_Status[MotorD] > OnApproachToTarget); 
  CloseGripper();
  delay(1000);
  MoveMotorToAngle(MotorD,0);
  MoveMotorToAngle(MotorE,-80);
  do {delay(50);} while (Motion_Status[MotorD] > OnApproachToTarget); 
  MoveMotorToAngle(MotorF,-45);
  do {delay(50);} while (Motion_Status[MotorE] > OnApproachToTarget); 
  OpenGripper();
  delay(1000);
  SetPositionToHome();
  do {delay(50);} while (Motion_Status[MotorE] > BesideTarget); 
  TurnOffPID(); 
  Serial.println("stop");
}

void RunWayPointSeq() {
  Serial.println("Start WayPoints");
  EEPROM.get(0, WayPoint);
  int NumberOf = WayPoint.A;
  if (NumberOf < 100) {
    TurnOnPID();
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
          pinMode(Expansion_IO[Pin-1], INPUT);
          val = digitalRead(Expansion_IO[Pin-1]);
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
    TurnOffPID();  
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
    for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
      TrackReport(iMotor);
    }
  }   
}

boolean Check_A() {
  return
  (Motion_Status[MotorC] > AtTarget) ||   
  (Motion_Status[MotorD] > AtTarget) ||   
  (Motion_Status[MotorE] > AtTarget) ||   
  (Motion_Status[MotorF] > AtTarget);       
}

boolean Check_B() {
  return
  (Motion_Status[MotorC] > BesideTarget) ||   
  (Motion_Status[MotorD] > BesideTarget) ||   
  (Motion_Status[MotorE] > BesideTarget) ||   
  (Motion_Status[MotorF] > BesideTarget);       
}

boolean Check_C() {
  return
  (Motion_Status[MotorC] > CloseToTarget) ||   
  (Motion_Status[MotorD] > CloseToTarget) ||   
  (Motion_Status[MotorE] > CloseToTarget) ||   
  (Motion_Status[MotorF] > CloseToTarget);       
}

boolean Check_D() {
  return
  (Motion_Status[MotorC] > OnApproachToTarget) ||   
  (Motion_Status[MotorD] > OnApproachToTarget) ||   
  (Motion_Status[MotorE] > OnApproachToTarget) ||   
  (Motion_Status[MotorF] > OnApproachToTarget);       
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

void EEPROMSetRhinoType(int xRhinoType) {
  RhinoType = xRhinoType;
  EEPROM.put(Eloc_RobotType, RhinoType);
  Serial.print("Now configured for Rhino XR");
  Serial.println(RhinoType);  
}

void EEPROMSetAngleOffsets(int B, int C, int D, int E, int F) {
  AngleOffset[MotorB] = B;
  AngleOffset[MotorC] = C;
  AngleOffset[MotorD] = D;
  AngleOffset[MotorE] = E;
  AngleOffset[MotorF] = F;
  EEPROM.put(AngleOffsetELoc[MotorB], AngleOffset[MotorB]);
  EEPROM.put(AngleOffsetELoc[MotorC], AngleOffset[MotorC]);
  EEPROM.put(AngleOffsetELoc[MotorD], AngleOffset[MotorD]);
  EEPROM.put(AngleOffsetELoc[MotorE], AngleOffset[MotorE]);
  EEPROM.put(AngleOffsetELoc[MotorF], AngleOffset[MotorF]);
  Serial.print("Angle Offsets Set to: ");
  Serial.print(AngleOffset[MotorB]);
  Serial.print(", ");
  Serial.print(AngleOffset[MotorC]);
  Serial.print(", ");
  Serial.print(AngleOffset[MotorD]);
  Serial.print(", ");
  Serial.print(AngleOffset[MotorE]);
  Serial.print(", ");
  Serial.print(AngleOffset[MotorF]);
  Serial.println(".");
}

void SetZeroAngles() {
  int B = Motor_Position(MotorB);
  int C = Motor_Position(MotorC);
  int D = Motor_Position(MotorD);
  int E = Motor_Position(MotorE);
  int F = Motor_Position(MotorF);
  EEPROMSetAngleOffsets(B, C, D, E, F);
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
    for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){
      float Angle = 0;
      switch (iMotor) {
        case MotorF: Angle = WayPoint.F;
          break;
        case MotorE: Angle = WayPoint.E;
          break;
        case MotorD: Angle = WayPoint.D;
          break;
        case MotorC: Angle = WayPoint.C;
          break;
        case MotorB: Angle = WayPoint.B;
          break;
        case MotorA: Angle = WayPoint.A;
      }      
      MoveMotorToAngle(iMotor, Angle);  
    }  
  }  
}

void RunMotorsForAsmTest() {
  Serial.println("Run Motors For Asm Test");    
  TurnOffPID();
  delay(250);

  // Set all motor power lines to High-Z state by 
  // Setting speed to 0 and turning off brakes.  
  Serial.println("Setting Drive Power for all Motors to High-Z.");
  for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){    
    digitalWrite(Motor_IO_BRK[iMotor], LOW); // Turn the Brakes off.
    Motor_Speed[iMotor] = 0;
    SetMotorPWM(iMotor);
  }  

  int MDir = 1;
  for (int z=0; z<10000; z++){
    for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){    
      MotorTestFullSpeed(iMotor,MDir);
    }
    delay(5000);               // Run for 5 sec.
    for (int iMotor=MotorA; iMotor<=MotorF; iMotor++){    
      MotorTestStop(iMotor);
    }
    delay(100);                // Short Delay to allow the motor to stop.  
    MDir = MDir * -1;
    Serial.println("Reverse.");
  }
}

void MotorTestStop(int m) {
  Motor_Speed[m]=0;          // Turn off motor.
  SetMotorPWM(m);            // Set Motor PWM.
}

void MotorTestFullSpeed(int m, int MDir) {
  int TestSpeed = 255-MinSpeed;
  Motor_Speed[m]=TestSpeed*MDir; // Set Motor Speed.
  SetMotorPWM(m);                // Set Motor PWM.
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
