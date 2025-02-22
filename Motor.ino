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
  
  setMotorPw(true, ZERO_PW);
  setMotorPw(false, ZERO_PW);

  attachInterrupt(ENC_A_DW, encoderIsrDw, CHANGE);
  attachInterrupt(ENC_A_RW, encoderIsrRw, CHANGE);
}



/*********************************************************
 * encoderIsrDw() Drive wheel interrupt
 *********************************************************/
void encoderIsrDw() {
  static boolean encAStat;
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
    if (tickPeriodDw < 0) {  // reversed?
      tickPeriodDw = 1000000;  
    } else {
      tickPeriodDw = (long) tickTimeDw - (long) lastTickTime;
    }
    tickPositionDw++;
  } 
  else {
    if (tickPeriodDw >= 0) {  // reversed?
      tickPeriodDw = -1000000;
    } else {
      tickPeriodDw = (long) lastTickTime - (long) tickTimeDw;
    }
    tickPositionDw--;
  }
  mFpsDw = (ENC_FACTOR_M / tickPeriodDw); // speed in milli-fps
  int speedError = mFpsDw - targetMFpsDw;
  int pwCorrection = MOTOR_GAIN * ((float) speedError);
  int newPw =  targetPwDw - pwCorrection;
  if (mode != MODE_PWM_SPEED) {
    setMotorPw(true, newPw); //------------------------------------------------------------------------------
  }
//  addLog(
//    (long) (0),
//    (short) (mFpsDw),
//    (short) (targetMFpsDw),
//    (short) (speedError),
//    (short) (pwCorrection),
//    (short) (0),
//    (short) (newPw)
//   );
} // encoderIsrDw()


/************************************************************************
 *  encoderIsrRw() Reaction wheel interrupt
 ************************************************************************/
void encoderIsrRw() {
//return;
  static boolean encAStat;
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
    if (tickPeriodRw < 0) {
      tickPeriodRw = 1000000;
    } else {
      tickPeriodRw = (long) tickTimeRw - (long) lastTickTime;
    }
    tickPositionRw++;
  } else {
    if (tickPeriodRw >= 0) {
      tickPeriodRw = -1000000;
    } else {
      tickPeriodRw = (long) lastTickTime - (long) tickTimeRw;
    }
    tickPositionRw--;
  }
  mFpsRw = (ENC_FACTOR_M / tickPeriodRw); // speed in milli-fps
  int speedError = mFpsRw - targetMFpsRw;
  int pwCorrection = MOTOR_GAIN * ((float) speedError);
  int newPw =  targetPwRw - pwCorrection;
  if (mode != MODE_PWM_SPEED) {
    setMotorPw(false, newPw);
  }
//  addLog(
//    (long) (tickTimeRw),
//    (short) (tickPeriodRw),
//    (short) (tickPositionRw),
//    (short) (mFpsRw),
//    (short) (gyroYawRate * 100.0),
//    (short) (gyroHeading * 100.0),
//    (short) (encB)
//   );
} // end encoderIsrRw();  // !!!!!!!!!!!!!! reactionwheel !!!!!!!!!!!!!!



/************************************************************************
 *   checkMotorXXX() Give appropriate PWM if motor is inactive.
 ************************************************************************/
void checkMotor(boolean isDw) {
  int speedError, pwCorrection, newPw;
  if (isDw) {
    speedError = mFpsDw - targetMFpsDw;
    pwCorrection = MOTOR_GAIN * ((float) speedError);
    newPw =  targetPwDw - pwCorrection;
    if (isRunning && ((mode == MODE_FP) ||  (mode == MODE_T_SPEED) || (mode == MODE_PWM_SPEED))) {
      setMotorPw(true, newPw);
    }
//    addLog(
//      (long) (5),
//      (short) (mFpsDw),
//      (short) (targetMFpsDw),
//      (short) (speedError),
//      (short) (pwCorrection),
//      (short) (0),
//      (short) (newPw)
//     );

  } else {
    speedError = mFpsRw - targetMFpsRw;
    pwCorrection = MOTOR_GAIN * ((float) speedError);
    newPw =  targetPwRw - pwCorrection;
    setMotorPw(false, newPw);
  }
 }



/***********************************************************************.
 *
 *   setTargetSpeed()   Set the pulse width that will produce the given
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
  } else {
    targetPwRw = targetPw;
    targetMFpsRw = (int) (targetFps * 1000.0); 
  }
}


 
/*********************************************************
 *   setMotorPw()
 *********************************************************/
void setMotorPw(boolean isDw, int pw) {
  if (!isDw) pw = 65535 - pw;  // Reverse direction for reaction wheel
  if (pw > 65535) pw = 65535;
  else if (pw < 0) pw = 0;
  
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
  int tt, mFps;
  int waitMFps;
  if (isDw) {
    noInterrupts();
    tt = tickTimeDw;
    mFps = mFpsDw;
    interrupts();
     waitMFps = ENC_FACTOR_M /(micros() - tt);
    if (mFpsDw > 0) {
      if (waitMFps < mFps) mFps = waitMFps;
    } else {
      if (-waitMFps > mFps) mFps = -waitMFps;
    }
    fpsDw = ((float) mFps) / 1000.0;
  } else {
    noInterrupts();
    tt = tickTimeRw;
    mFps = mFpsRw;
    interrupts();
    waitMFps = ENC_FACTOR_M /(micros() - tt);
    if (mFpsRw > 0) {
      if (waitMFps < mFps) mFps = waitMFps;
    } else {
      if (-waitMFps > mFps) mFps = -waitMFps;
    }
    fpsRw = ((float) mFps) / 1000.0;
//  addLog(
//    (long) (waitMFps),
//    (short) (mFpsRw),
//    (short) (tt),
//    (short) (mFps),
//    (short) (0),
//    (short) (0),
//    (short) (0)
//   );
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


