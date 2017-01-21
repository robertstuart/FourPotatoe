int lastMWsFpsLeft = 0;
int rejectCountRight = 0;
int rejectCountLeft = 0;



/***********************************************************************.
 *  motorInitTp() 
 ***********************************************************************/
void motorInit() {
  pinMode(PWM_DW, OUTPUT);
  pinMode(DIR_DW, OUTPUT);
  pinMode(PWM_RW, OUTPUT);
  pinMode(DIR_RW, OUTPUT);
  digitalWrite(PWM_DW, LOW);
  digitalWrite(PWM_RW, LOW);

  // Initialize the pwm frequency
  pwm_set_resolution(16);
  pwm_setup(DIR_DW, 20000, 1);  // on clock A
  pwm_setup(DIR_RW, 20000, 1);  // on clock A
  
  setMotor(true, ZERO_PW);
  setMotor(false, ZERO_PW);

  attachInterrupt(ENC_A_DW, encoderIsrDw, CHANGE);
  attachInterrupt(ENC_A_RW, encoderIsrRw, CHANGE);
}



/*********************************************************
 * encoderIsrDw() Drive wheel interrupt
 *********************************************************/
void encoderIsrDw() {
  static int lastTickPeriodDw = 0;
  static boolean encAStat;
  int pw = 0;
  boolean encA = (!!(g_APinDescription[ENC_A_DW].pPort -> PIO_PDSR & g_APinDescription[ENC_A_DW].ulPin)) ? true : false;
  if (encA == encAStat) {
    interruptErrorsDw++;
    return;  // Ignore if bogus interrupt!
  }
  encAStat = encA;
  unsigned long lastTickTime = tickTimeDw;
  tickTimeDw = micros();  
  boolean encB = (!!(g_APinDescription[ENC_B_DW].pPort -> PIO_PDSR & g_APinDescription[ENC_B_DW].ulPin)) ? true : false;
    
  if (encA != encB) {
    tickPeriodDw = (long) tickTimeDw - (long) lastTickTime;
    tickPositionDw++;
  } 
  else {
    tickPeriodDw = (long) lastTickTime - (long) tickTimeDw;
    tickPositionDw--;
  }
  mFpsDw = (ENC_FACTOR_M / tickPeriodDw); // speed in milli-fps

// is this needed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (isRunning && ((mode == MODE_FP) ||  (mode == MODE_T_SPEED))) {
    int speedError = mFpsDw - targetMFpsDw;
    int pwCorrection = MOTOR_GAIN * ((float) speedError);
    int newPw =  targetPwDw - pwCorrection;
    setMotor(true, newPw);
  }
//  addLog(
//        (long) (tickPeriodDw),
//        (short) (fpsDw * 100.0),
//        (short) (0),
//        (short) (0),
//        (short) (0),
//        (short) (0),
//        (short) (0)
//   );

} // encoderIsrDw()


/************************************************************************
 *  encoderIsrRw() Reaction wheel interrupt
 ************************************************************************/
void encoderIsrRw() {
  static int lastTickPeriodRw = 0;
  static boolean encAStat;
  int pw = 0;
  boolean encA = (!!(g_APinDescription[ENC_A_RW].pPort -> PIO_PDSR & g_APinDescription[ENC_A_RW].ulPin)) ? true : false;
  if (encA == encAStat) {
    interruptErrorsRw++;
    return;  // Ignore if bogus interrupt!
  }
  encAStat = encA;
  unsigned long lastTickTime = tickTimeRw;
  tickTimeRw = micros();  
  boolean encB = (!!(g_APinDescription[ENC_B_RW].pPort -> PIO_PDSR & g_APinDescription[ENC_B_RW].ulPin)) ? true : false;
    
  if (encA != encB) {
    tickPeriodRw = (long) tickTimeRw - (long) lastTickTime;
    tickPositionRw++;
  } 
  else {
    tickPeriodRw = (long) lastTickTime - (long) tickTimeRw;
    tickPositionRw--;
  }
 mFpsRw = (ENC_FACTOR_M / tickPeriodRw); // speed in milli-fps

  if (isRunning && ((mode == MODE_FP) ||  (mode == MODE_T_SPEED))) {
    int speedError = mFpsRw - targetMFpsRw;
    int pwCorrection = MOTOR_GAIN * ((float) speedError);
    int newPw =  targetPwRw - pwCorrection;
    setMotor(false, newPw);
  }
} // end encoderIsrMon();  // !!!!!!!!!!!!!! reactionwheel !!!!!!!!!!!!!!



/************************************************************************
 * checkMotorXXX() Give appropriate PWM if motor is inactive.
 ************************************************************************/
void checkMotor(boolean isDw) {
  int speedError, pwCorrection, newPw;
  if (isDw) {
    speedError = mFpsDw - targetMFpsDw;
    pwCorrection = MOTOR_GAIN * ((float) speedError);
    newPw =  targetPwDw - pwCorrection;
  } else {
    speedError = mFpsRw - targetMFpsRw;
    pwCorrection = MOTOR_GAIN * ((float) speedError);
    newPw =  targetPwRw - pwCorrection;
  }
  if (isRunning && ((mode == MODE_FP) ||  (mode == MODE_T_SPEED))) {
    setMotor(isDw, newPw);
  }
}



/***********************************************************************.
 *
 * setTargetSpeed()   Set the pulse width that will produce the given
 *                    speed with the motor not under load.
 *
 ***********************************************************************/
void setTargetSpeed(boolean isDw, float targetFps) {
  int targetPw;
  int dead = (targetFps < 0) ? - DEAD_ZONE : DEAD_ZONE;
  int target = (targetFps * PW_VS_FPS) + dead;
  targetPw = ZERO_PW + target;
  if (isDw) {
    targetPwDw = targetPw;
    targetMFpsDw = (int) (targetFps * 1000.0); 
  }
  else {
    targetPwRw = targetPw;
    targetMFpsRw = (int) (targetFps * 1000.0); 
  }
}


 
/*********************************************************
 * setMotor()
 *********************************************************/
void setMotor(boolean isDw, int pw) {
  if (pw > 65535) pw = 65535;
  else if (pw < 0) pw = 65535;
  
  if (isDw) {
    pwDw = pw;
    pwm_write_duty(DIR_DW, pw);
  } else {
    pwRw = pw;
    pwm_write_duty(DIR_RW, pw);
  }
}



/***********************************************************************.
 *
 *  readSpeed() Compute the speed calculating a new (lower) speed if 
 *              the tick hasn't appeared ealier than the last period.
 *
 ***********************************************************************/
void readSpeed(boolean isDw) {
  int waitMFps;
  if (isDw) {
    waitMFps = ENC_FACTOR_M /(micros() - tickTimeDw);
    if (mFpsDw > 0) {
      if (waitMFps < mFpsDw) mFpsDw = waitMFps;
    } else {
      if (-waitMFps > mFpsDw) mFpsDw = -waitMFps;
    }
    fpsDw = ((float) mFpsDw) / 1000.0;
  } else {
    waitMFps = ENC_FACTOR_M /(micros() - tickTimeRw);
    if (mFpsRw > 0) {
      if (waitMFps < mFpsRw) mFpsRw = waitMFps;
    } else {
      if (-waitMFps > mFpsRw) mFpsRw = -waitMFps;
    }
    fpsRw = ((float) mFpsRw) / 1000.0;
  }
}



///***********************************************************************.
// *  fpsToPw() Return the Pulse Width (PW) value that will give the 
// *            specified speed in Feet Per Second (FPS)
// ***********************************************************************/
//int fpsToPw(float fps) {
//  int x = ((int) (abs(fps) * PW_VS_FPS)) + DEAD_ZONE;
//  if (x > ZERO_PW) x = ZERO_PW;
//  if (fps > 0.0) return(ZERO_PW + x);
//  else return(ZERO_PW + x);
//}


