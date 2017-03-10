#include "Common.h"
#include "pwm01.h"
#include <DueTimer.h>
#include <Wire.h>
#include <LSM6.h>

/******************** Constants to be adjusted **************************/

const float T_RW_VAL = 0.0;  // angle for, rwAngle
const float T_VAL = 60.0;  // rollP for rollBalance()
const float U_VAL = 250.0;    // rollD for rollBalance()
const float V_VAL = 60.0;    // vectorP for rollBalance()
const float W_VAL = 40.0;    // vectorD for rollBalance()
const float X_VAL = 190.0;   // yawD for yawAction()
const float Y_VAL = 60.0;   // yawP for yawAction()
const float Z_VAL = 0.0;

const float T_FP_VAL = 6.0;
const float U_FP_VAL = 10.0;
const float V_FP_VAL = 4.0;
const float W_FP_VAL = 0.40;  // 0.18
const float X_FP_VAL = 0.05;

const boolean IS_ENABLE_DW = false;
const boolean IS_ENABLE_RW = false;
const float TICKS_PER_FOOT = 292.2;
const float FEET_PER_TICK = 1 / TICKS_PER_FOOT;
const int PWM_FREQ = 20000;
const float COS_TC = 0.2;
const float SPEED_MULTIPLIER = 5.0; // Multiplier for controllerX to get speed
const float PITCH_DRIFT = -36.0;
const float ROLL_DRIFT = -8.0;
const float YAW_DRIFT = 13.66;
const float GYRO_SENS = 0.0696;     // Multiplier to get degree. subtract 1.8662% * 8 for 2000d/sec
const float GYRO_WEIGHT = 0.98;     // Weight for gyro compared to accelerometer
const float MOTOR_GAIN = 10.0;
//const float JIG_MAX_FPS = 1.0;
//const float JIG_FPS_INC = 0.01;   // Speed increase each 1/200 sec during jig
const int JIG_TICK_RANGE = 55;

/*
 * 1/293 = .00341
 */
const double ENC_FACTOR = 3458.0f;  // Change pulse width to fps speed
const long ENC_FACTOR_M = ENC_FACTOR * 1000.0;  // Change pulse width to milli-fps speed

static const int ZERO_PW = 32766;  // Center of pulse width range. FPS = 0;
static const int DEAD_ZONE = 1200; // Zone around ZERO_PW that gives zero FPS
static const int PW_VS_FPS = 2800.0; // change in PW gives change of 1.0 FPS

// State of the reaction wheel
static const int RW_STOP = 0;
static const int RW_SPIN_UP = 1;
static const int RW_RUNNING = 2;
static const int RW_SPIN_DOWN = 3;

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

const int BD_LED     = 13; // led on arduino board
const int RE_LED     = 28; 
const int BU_LED     = 29;
const int RE_SWITCH  = 30;
const int BU_SWITCH  = 31;
const int X_SWITCH   = 32;
const int Y_SWITCH   = 33;
const int Z_SWITCH   = 34;
const int SPEAKER    = 35;
const int ACCEL_INT1  = 37;
const int GYRO_INT2  = 36;

const int CH2_RADIO = 49;
const int CH3_RADIO = 51;
const int CH4_RADIO = 53;




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
//int mode = MODE_RW_ANGLE;
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
float targetHeading = 0.0;

// logXX()
char message[200];

float roll_correction = 3.2;
float yaw_p = 10.0;
float yaw_d = 10.0;

float targetRwFps = 0.0;
float targetRwAngle = 0.0;

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
volatile int mFpsDw = 0;
volatile int mFpsRw = 0;
volatile int interruptErrorsDw = 0;
volatile int interruptErrorsRw = 0;
volatile unsigned int tickTimeDw = 0;
volatile unsigned int tickTimeRw = 0;
volatile int tickPeriodDw = 0;
volatile int tickPeriodRw = 0;
volatile int tickPositionDw = 0;
volatile int tickPositionRw = 0;
float fpsDw = 0.0;

float fpsRw = 0.0;
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
float tVal = T_FP_VAL;
float uVal = 3.2; //U_FP_VAL;
float vVal = V_FP_VAL;
float wVal = W_FP_VAL;
float xVal = X_FP_VAL;
float yVal = Y_VAL;  // roll zero
float zVal = Z_VAL;  // pitch zero

/************************ Tasks.ino ************************************/
//controllerConnected()
boolean isHcActive = false;
boolean isPcActive = false;
boolean isController = false;
//battery()
float battVolt = 0.0;

/*********************** Global variables ************************************/
boolean isRunReady = false;
boolean isRunning = false;
boolean isJigger = false;
boolean isUpright = true;
boolean isDumpingData = false;
boolean isBattAlarmDisabled = false;
boolean isJig = false;

int rwState = RW_STOP;

int ch2pw = 0;
int ch3pw = 0;
int ch4pw = 0;
volatile unsigned int ch2riseTime = 0;
volatile unsigned int ch3riseTime = 0;
volatile unsigned int ch4riseTime = 0;
boolean ch3sw = false;

unsigned int timeMilliseconds = 0;
unsigned int timeMicroseconds = 0;

void setup() {

  pinMode(BU_LED, OUTPUT);
  pinMode(RE_LED, OUTPUT);
  pinMode(BD_LED, OUTPUT);
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
  
  BLUE_SER.begin(115200);   // Bluetooth 
  Serial.begin(115200);     // for debugging output

  imuInit();
  motorInit();
  rcRadioInit();
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
  case MODE_RW_ANGLE:
    aRwAngle();
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
  
  tVal = ZERO_PW;
  uVal = ZERO_PW;
  isRunning = true;
  setBlink(RE_LED, BLINK_FF);
  setBlink(BD_LED, BLINK_FF);
  while (mode == MODE_PWM_SPEED) {
    commonTasks();
    if (isNewGyro()) log200PerSec();
    if (timeMicroseconds > pwmTrigger) {
      pwmTrigger = timeMicroseconds + 100000; // 10/sec
      
      setMotorPw(true, ((unsigned int) (tVal)));
      setMotorPw(false, ((unsigned int) (uVal)));
      readSpeed(true);  
      readSpeed(false);  
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
  tVal = 0.0;
  uVal = 0.0;
  setBlink(RE_LED, BLINK_FF);
  setBlink(BD_LED, BLINK_FF);
  
  while (mode == MODE_T_SPEED) {
    commonTasks();
    if (isNewGyro()) {
      setGyroData();
      fpControllerSpeed = ((float) tVal) / 1000.0;
      setTargetSpeed(true, fpControllerSpeed);
      targetRwFps = ((float) uVal) / 1000.0;
      setTargetSpeed(false, targetRwFps);
      checkMotor(true);
      checkMotor(false);
      readSpeed(true);  
      readSpeed(false);  
      sendLog();
    }
  } // while
} // aTickSpeed()



/**************************************************************************.
 * aRwAngle()
 **************************************************************************/
void aRwAngle() {
  tVal = T_RW_VAL;  // Heading adjustment
  vVal = X_VAL;  // P value for yawAction() yawP
  wVal = Y_VAL;  // D value for yawAction() yawD
  static float sum = 0.0;
  static int loop = 0;
  float headAngle;
  
  delay(100);  // For switches()
  
  while (mode == MODE_RW_ANGLE) {
    commonTasks();
    if (isNewGyro()) {
      setGyroData();
      readSpeed(false); 
      headAngle = tVal * 1.0;
      yawAction(headAngle - gyroHeading);
      checkMotor(false);
      sendLog();
      if (!isRunning) {
        zeroYaw();
//        targetRwFps = 0.0;
      }

//      if ((++loop % 200) == 0) {
//        sprintf(message, "fpControllerSpeed: %5.2f \t fpsDw: %5.2f \t late: %d", fpControllerSpeed, fpsDw, targetPwDw);
//        sendBMsg(SEND_MESSAGE, message);
//      }
    }
  } // while
} // aRwAngle()

