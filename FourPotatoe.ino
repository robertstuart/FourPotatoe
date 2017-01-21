#include "Common.h"
#include "pwm01.h"
#include <DueTimer.h>
#include <Wire.h>
#include <LSM6.h>


/******************** Constants to be adjusted **************************/
const float T_FACT = 6.0;
const float U_FACT = 7.4;
const float V_FACT = 4.0;
const float W_FACT = 0.18;  // 0.18
const float X_FACT = 0.05;
const float Y_FACT = 130.0;
const float Z_FACT = 0.0;
const float COS_TC = 0.2;
const float SPEED_MULTIPLIER = 5.0; // Multiplier for controllerX to get speed
const float PITCH_DRIFT = -30.0;
const float ROLL_DRIFT = -8.0;
const float YAW_DRIFT = 15.0;
const float GYRO_SENS = 0.0696;     // Multiplier to get degree. subtract 1.8662% * 8 for 2000d/sec
const float GYRO_WEIGHT = 0.98;    // Weight for gyro compared to accelerometer
const float MOTOR_GAIN = 10.0;

/*
 * 1/293 = .00341
 */
const double ENC_FACTOR = 3458.0f;  // Change pulse width to fps speed
const long ENC_FACTOR_M = 3458000;  // Change pulse width to milli-fps speed
const float TICKS_PER_FOOT = 293.0; // 

static const int ZERO_PW = 32766;  // Center of pulse width range. FPS = 0;
static const int DEAD_ZONE = 1200; // Zone around ZERO_PW that gives zero FPS
static const int PW_VS_FPS = 2800.0; // change in PW gives change of 1.0 FPS

#define BLUE_SER Serial3

const int BATTERY   = 0;

const int PWM_RW   = 6;
const int DIR_RW   = 7;
const int PWM_DW   = 8;
const int DIR_DW   = 9;

const int ENC_A_RW = 22;
const int ENC_B_RW = 24;
const int ENC_A_DW = 23;
const int ENC_B_DW = 25;

const int BU_LED     = 28; 
const int RE_LED     = 29;
const int RE_SWITCH  = 30;
const int BU_SWITCH  = 31;
const int X_SWITCH   = 32;
const int Y_SWITCH   = 33;
const int Z_SWITCH   = 34;
const int SPEAKER    = 35;
const int ACCEL_INT1  = 37;
const int GYRO_INT2  = 36;

const int CH1_RADIO = 40;
const int CH2_RADIO = 41;
const int CH3_RADIO = 42;
const int CH4_RADIO = 43;
const int CH5_RADIO = 44;
const int CH6_RADIO = 45;

const int PWM_FREQ = 20000;



// Due has 96 kbytes sram
// Arrays to save data to be dumped in blocks.
#define DATA_ARRAY_SIZE 2500
long  aArray[ DATA_ARRAY_SIZE];
short bArray[ DATA_ARRAY_SIZE];
short cArray[ DATA_ARRAY_SIZE];
short dArray[ DATA_ARRAY_SIZE];
short eArray[ DATA_ARRAY_SIZE];
short fArray[ DATA_ARRAY_SIZE];
short gArray[ DATA_ARRAY_SIZE];
unsigned int dataArrayPtr = 0;


// Flash sequences
const byte END_MARKER = 42;
byte BLINK_OFF[] = {0,END_MARKER};               // Off
byte BLINK_SF[] = {1,0,0,0,0,0,0,0,END_MARKER};  // Slow flash
byte BLINK_FF[] = {1,0,END_MARKER};              // Fast flash
byte BLINK_SB[] = {1,1,1,1,0,0,0,0,END_MARKER};  // Slow blink
byte BLINK_ON[] = {1,END_MARKER};                // On

// Beep sequences
int BEEP_UP [] = {1200, 100, 1500, 100, 0};
int BEEP_WARBLE[] = {2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 
                  2400, 300, 2600, 300, 0};
int BEEP_DOWN[] = {1000, 200, 666, 200, 0};

 
int mode = MODE_FP;
int debugInt = 0;
float debugFloat = 0.0;

/******************** Algorithm.ino ************************************/
// xBalance()
float fpRotation = 0.0;
float fpCos = 0.0;
float fpLpfCos = 0.0;
float fpLpfCosAccel = 0.0;
float fpLpfCosOld = 0.0;
float fpControllerSpeed = 0.0; 
float fpSpeedError = 0.0;
float fpTargetAngle = 0.0;
float fpAngleError = 0.0;
float fpCorrection = 0.0;
float fpLpfCorrection = 0.0;
float fpLpfCorrectionOld = 0.0;
float fpFps = 0.0;
// logXX()
char message[200];

/************************ Imu.ino *************************************/
LSM6 lsm6;
// setGyroData()
float gyroPitchRaw = 0.0;
float gyroPitchRate = 0.0;
float gyroPitchDelta = 0.0;
float gPitch = 0.0;
float gaPitch = 0.0;
float gaRealPitch = 0.0;
float gyroRollRaw = 0.0;
float gyroRollRate = 0.0;
float gyroRollDelta = 0.0;
float gRoll = 0.0;
float gaRoll = 0.0;
float gyroYawRaw = 0.0;
float gyroYawRate = 0.0;
float gyroYawDelta = 0.0;
float gYaw = 0.0;
float gyroCumHeading = 0.0;
float gyroHeading = 0.0;
// setAccelData()
float aPitch = 0.0;
float aRoll = 0.0;

/************************ Motor.ino ************************************/
int mFpsDw = 0;
int mFpsRw = 0;
float fpsDw = 0.0;
float fpsRw = 0.0;
int interruptErrorsDw = 0;
int interruptErrorsRw = 0;
unsigned int tickTimeDw = 0;
unsigned int tickTimeRw = 0;
int tickPeriodDw = 0;
int tickPeriodRw = 0;
int tickPositionDw = 0;
int tickPositionRw = 0;
float targetSpeedDw = 0.0;
float targetMFpsDw = 0;
float targetMFpsRw = 0;
int targetPwDw = 0;
int targetPwRw = 0;
unsigned int pwDw = 0;
unsigned int pwRw = 0;

/************************ Msg.ino **************************************/
// readPc()
unsigned int tPc = 0;
// doMsg()
float hcX = 0.0;
float pcX = 0.0;
float hcY = 0.0;
float pcY = 0.0;
float controllerX = 0.0;
float controllerY = 0.0;
float tVal = T_FACT;
float uVal = U_FACT;
float vVal = V_FACT;
float wVal = W_FACT;
float xVal = X_FACT;
float yVal = Y_FACT;
float zVal = Z_FACT;

/************************ Tasks.ino ************************************/
//controllerConnected()
boolean isHcActive = false;
boolean isPcActive = false;
boolean isController = false;
//battery()
float battVolt = 0.0;

boolean isRunReady = false;
boolean isRunning = false;
boolean isJigger = false;
boolean isUpright = true;
boolean isDumpingData = false;
boolean isBattAlarmDisabled = false;

unsigned int timeMilliseconds = 0;
unsigned int timeMicroseconds = 0;
int tickPosition = 0;

void setup() {

  pinMode(BU_LED, OUTPUT);
  pinMode(RE_LED, OUTPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(RE_SWITCH, INPUT_PULLUP);
  pinMode(BU_SWITCH, INPUT_PULLUP);
  pinMode(X_SWITCH, INPUT_PULLUP);
  pinMode(Y_SWITCH, INPUT_PULLUP);
  pinMode(Z_SWITCH, INPUT_PULLUP);
  
  pinMode(ENC_A_DW, INPUT);
  pinMode(ENC_B_DW, INPUT);
  pinMode(ENC_A_RW, INPUT);
  pinMode(ENC_B_RW, INPUT);
  
  pinMode(CH1_RADIO, INPUT);
  pinMode(CH2_RADIO, INPUT);
  pinMode(CH3_RADIO, INPUT);
  pinMode(CH4_RADIO, INPUT);
  pinMode(CH5_RADIO, INPUT);
  pinMode(CH6_RADIO, INPUT);
//  attachInterrupt(CH1_RADIO, ch1Isr, CHANGE);
//  attachInterrupt(CH2_RADIO, ch2Isr, CHANGE);
//  attachInterrupt(CH3_RADIO, ch3Isr, CHANGE);
//  attachInterrupt(CH4_RADIO, ch4Isr, CHANGE);
//  attachInterrupt(CH5_RADIO, ch5Isr, CHANGE);
//  attachInterrupt(CH6_RADIO, ch6Isr, CHANGE);

  BLUE_SER.begin(115200);   // Bluetooth 
  Serial.begin(115200);     // for debugging output

  imuInit();
  motorInit();
  beep(BEEP_UP);
}

void loop() {
  switch (mode) {
  case MODE_PWM_SPEED:
    aPwmSpeed();
    break;
  case MODE_T_SPEED:
    aTickSpeed();
    break;
  case MODE_FP:
    fpRun();
    break;
  default:
    commonTasks; 
    break;
  } // end switch(mode)
}



/**************************************************************************.
 * aPwmSpeed()
 **************************************************************************/
void aPwmSpeed() {
  unsigned long pwmTrigger = 0;
  unsigned long tt;
  isRunning = true;
  setBlink(RE_LED, BLINK_OFF);
  setBlink(BU_LED, BLINK_FF);
  
  while (mode == MODE_PWM_SPEED) {
    commonTasks();
    if (isNewGyro()) log200PerSec();
    if (timeMicroseconds > pwmTrigger) {
      pwmTrigger = timeMicroseconds + 100000; // 00/sec
      
      setMotor(true, ((unsigned int) (tVal)));
//      setMotor(false, ((unsigned int) (uVal)));
      readSpeed(true);  
      log10PerSec();         
      sendStatusBluePc();
//      if (isDumpingTicks) dumpTicks();
    } // end timed loop 
  } // while
} // aPwmSpeed()



/**************************************************************************.
 * aTickSpeed()
 **************************************************************************/
void aTickSpeed() {
  static unsigned int loop = 0;
  setBlink(RE_LED, BLINK_OFF);
  setBlink(BU_LED, BLINK_FF);
  
  while (mode == MODE_T_SPEED) {
    commonTasks();
    if (isNewGyro()) {
      fpControllerSpeed = ((float) tVal) / 1000.0;
      setTargetSpeed(true, fpControllerSpeed);
      checkMotor(true);
      readSpeed(true);  
//      sendLog();

      if ((++loop % 200) == 0) {
        sprintf(message, "fpControllerSpeed: %5.2f \t fpsDw: %5.2f \t late: %d", fpControllerSpeed, fpsDw, targetPwDw);
        sendBMsg(SEND_MESSAGE, message);
      }
    }
  } // while
} // aTickSpeed()

