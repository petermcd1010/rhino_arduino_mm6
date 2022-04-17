/*
 * Implementation for waypoints.
 * See the LICENSE file in the root directory of this project for copyright and licensing details.
 */

#include <Arduino.h>
#include <EEPROM.h>

// k/K -> Run WayPointSeq()
// x -> SetWayPointAngle()
// r -> GetWayPointAngle()
// ! -> MoveToAWayPointAngle()

struct sWayPoint {
    uint8_t Number;
    char    Command;
    float   A;
    float   B;
    float   C;
    float   D;
    float   E;
    float   F;
};

static sWayPoint WayPoint = {
    0, 0,
    0, 0,0,  0, 0, 0
};

#define MotorA 0
#define MotorB 1
#define MotorC 2
#define MotorD 3
#define MotorE 4
#define MotorF 5

#define AtTarget 0
#define BesideTarget 1  // Within 1 click.
#define CloseToTarget 2  // between 2 and 30 clicks.
#define OnApproachToTarget 3  // between 30 and 200 clicks.
#define OnWayToTarget 4  // More than 200 clicks away.

static String InBuffer = "";
static String StringSplits[8];  //Used to store waypoints
static int Expansion_IO[] = { A15, A14, A13, A12, 53, 49, 48, 41 };  // Expansion Lines
static int Motion_Status[] = { 0, 0, 0, 0, 0, 0 };  // Motion Status:
static int Tracking = 0;
static int Tracked[] = { 0, 0, 0, 0, 0, 0 };  // Last Value send while tracking.
static int Limit_Prev[] = { 0, 0, 0, 0, 0, 0 };  // Limit/Home switch Previous Value

static void WayPointMove(int i);
static void TrackUm();
static void MoveToAWayPointAngle();
static void MoveToWayPointAngle(int);
static bool Check_A();
static bool Check_B();
static bool Check_C();
static bool Check_D();
static void TrackReport(int tMotor);
static void ReportWayPoint();
static String GetStringPartAtSpecificIndex(String StringToSplit, char SplitChar, int StringPartIndex);

static void TurnOnPID()
{
}

static void TurnOffPID()
{
}

static void InterrogateLimitSwitches2()
{
}

static void MoveMotorToAngle(int Motor, float Angle)
{
}

static int Motor_Position(int tMotor)
{
    // return Motor_Encoder[tMotor] * Motor_Logic[tMotor];
    return 0;
}

static float Motor_Angle(int zMotor)
{
    // return (Motor_Position(zMotor) - AngleOffset[zMotor]) / EncoderStepsPerDegree(zMotor);
    return 0.0f;
}


static void RunWayPointSeq()
{
    Serial.println("Start WayPoints");
    EEPROM.get(0, WayPoint);
    int NumberOf = WayPoint.A;

    if (NumberOf < 100) {
        TurnOnPID();
        for (int Step = 1; Step <= NumberOf; Step++) {
            int Pin = 4;
            int val = 0;
            int Stp = 0;
            Serial.print("Step: ");
            Serial.print(Step);
            Serial.print(" - ");
            int eeAddress = Step * 40;  //Location we want the data to be put.
            EEPROM.get(eeAddress, WayPoint);
            switch (WayPoint.Command) {
            case 65:;  //A
            case 66:;  //B
            case 67:;  //C
            case 68:;  //D
                WayPointMove(Step);
                break;

            case 71:  //G
                Stp = WayPoint.A;
                Step = Stp - 1;
                Serial.print("Goto Step ");
                Serial.println(Stp);
                break;

            case 73:  //I
                InterrogateLimitSwitches2();
                break;

            case 74:  //J
                val = 0;
                Pin = WayPoint.B;
                Stp = WayPoint.A;
                Serial.print("Goto Step ");
                Serial.print(Stp);
                Serial.print(" If I/O[");
                Serial.print(Pin);
                Serial.println("]");
                pinMode(Expansion_IO[Pin - 1], INPUT);
                val = digitalRead(Expansion_IO[Pin - 1]);
                if (val == 0) Step = Stp - 1;
                break;

            case 87:  //W
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

static void WayPointMove(int i)
{
    Serial.print("Goto ");
    MoveToWayPointAngle(i);

    switch (WayPoint.Command) {
    case 65: do {
            delay(50); TrackUm();
    } while (Check_A());
        break;
    case 66: do {
            delay(50); TrackUm();
    } while (Check_B());
        break;
    case 67: do {
            delay(50); TrackUm();
    } while (Check_C());
        break;
    case 68: do {
            delay(50); TrackUm();
    } while (Check_D());
        break;
    }
}

static void TrackUm()
{
    if (Tracking > 0) {
        for (int iMotor = MotorA; iMotor <= MotorF; iMotor++) {
            TrackReport(iMotor);
        }
    }
}

static boolean Check_A()
{
    return
        (Motion_Status[MotorC] > AtTarget) ||
        (Motion_Status[MotorD] > AtTarget) ||
        (Motion_Status[MotorE] > AtTarget) ||
        (Motion_Status[MotorF] > AtTarget);
}

static boolean Check_B()
{
    return
        (Motion_Status[MotorC] > BesideTarget) ||
        (Motion_Status[MotorD] > BesideTarget) ||
        (Motion_Status[MotorE] > BesideTarget) ||
        (Motion_Status[MotorF] > BesideTarget);
}

static boolean Check_C()
{
    return
        (Motion_Status[MotorC] > CloseToTarget) ||
        (Motion_Status[MotorD] > CloseToTarget) ||
        (Motion_Status[MotorE] > CloseToTarget) ||
        (Motion_Status[MotorF] > CloseToTarget);
}

static boolean Check_D()
{
    return
        (Motion_Status[MotorC] > OnApproachToTarget) ||
        (Motion_Status[MotorD] > OnApproachToTarget) ||
        (Motion_Status[MotorE] > OnApproachToTarget) ||
        (Motion_Status[MotorF] > OnApproachToTarget);
}

static void SplitWayPoint(String waypoint)
{
    for (int i = 0; i < 8; i++) {
        StringSplits[i] = GetStringPartAtSpecificIndex(waypoint, '!', i);
    }
}

static String GetStringPartAtSpecificIndex(String StringToSplit, char SplitChar, int StringPartIndex)
{
    String originallyString = StringToSplit;
    String outString = "";

    for (int i1 = 0; i1 <= StringPartIndex; i1++) {
        outString = "";                //if the for loop starts again reset the outString (in this case other part of the String is needed to take out)
        int SplitIndex = StringToSplit.indexOf(SplitChar);  //set the SplitIndex with the position of the SplitChar in StringToSplit

        if (SplitIndex == -1)          //is true, if no Char is found at the given Index

            //outString += "Error in GetStringPartAtSpecificIndex: No SplitChar found at String '" + originallyString + "' since StringPart '" + (i1-1) + "'";    //just to find Errors
            return outString;
        for (int i2 = 0; i2 < SplitIndex; i2++) {
            outString += StringToSplit.charAt(i2);  //write the char at Position 0 of StringToSplit to outString
        }
        StringToSplit = StringToSplit.substring(StringToSplit.indexOf(SplitChar) + 1);  //change the String to the Substring starting at the position+1 where last SplitChar found
    }
    return outString;
}

static void SetWayPointAngle()
{
    InBuffer.setCharAt(0, 32);
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
    int eeAddress = Position * 40;  //Location we want the data to be put.

    EEPROM.put(eeAddress, WayPoint);
    ReportWayPoint();

    Serial.println("Set");
}

static void GetWayPointAngle()
{
    InBuffer.setCharAt(0, 32);
    int Position = InBuffer.toInt();
    int eeAddress = Position * 40;  //Location we want the data to be put.

    EEPROM.get(eeAddress, WayPoint);
    ReportWayPoint();
}

static void ReportWayPoint()
{
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

static void MoveToAWayPointAngle()
{
    InBuffer.setCharAt(0, 32);
    int Position = InBuffer.toInt();

    Serial.print("Goto ");
    MoveToWayPointAngle(Position);
}

static void MoveToWayPointAngle(int Position)
{
    if (Position > 0) {
        int eeAddress = Position * 40;  //Location we want the data to be put.
        EEPROM.get(eeAddress, WayPoint);
        ReportWayPoint();
        for (int iMotor = MotorA; iMotor <= MotorF; iMotor++) {
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

static void TrackReport(int tMotor)
{
    if (Tracking > 0) {
        int Position = Motor_Position(tMotor);
        if (Tracked[tMotor] != Position) {
            Serial.print("@");
            if (Tracking == 1) {
                Serial.print(char(tMotor + 65));
                Serial.print(Motor_Position(tMotor));
            } else if (Tracking == 2) {
                Serial.print(char(tMotor + 97));
                Serial.print(Motor_Angle(tMotor));
            }
            Serial.print(":HS");
            Serial.print(Limit_Prev[tMotor]);
            Serial.println(":");
            Tracked[tMotor] = Position;
        }
    }
}
