void fpRun() {
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickPosition = 0L;
  tVal = T_FACT;
  uVal = U_FACT;
  vVal = V_FACT;
  wVal = W_FACT;
  xVal = X_FACT;
  yVal = Y_FACT;
  zVal = Z_FACT;
  setBlink(RE_LED, BLINK_SB);
  delay(200);
  while (mode == MODE_FP) { // main loop
    commonTasks();
    static int intCount = 0;
    static unsigned int loopTrigger = 0;
    // Do the timed loop
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if (isNewGyro()) {
      setGyroData();
      readRcRadio();
      pitchBalance();
      yawAction(0.0);
      sendLog();
      //      safeAngle();
      checkMotor(true);
      checkMotor(false);
    }
    if (isNewAccel()) {
      setAccelData();
    }
  }
}



/***********************************************************************.
    pitchBalance()
        Balance on the pitch axis.  Go forward or backward to counteract
        falling forward or backward.
        Set the motor speed (fpFps).
 ***********************************************************************/
void pitchBalance() {
  int path = 0;
  static float jigFps = 0.0;
  static float targetJigFps = 0.0;
  static boolean isJigFwd = true;
  readSpeed(true);
  // compute the Center of Oscillation Speed (COS)
  fpRotation = U_FACT * (-gyroPitchDelta); // 7.4?
  fpCos = fpsDw + fpRotation; // subtract rotation
  fpLpfCos = (fpLpfCosOld * (1.0 - COS_TC))  + (fpCos  * COS_TC); // smooth it out a little (0.2)
  fpLpfCosAccel = fpLpfCos - fpLpfCosOld;
  fpLpfCosOld = fpLpfCos;

  fpControllerSpeed = controllerY * SPEED_MULTIPLIER;

  // Jig back and fort then fpControllerSpeed is low.
  if (abs(fpControllerSpeed) < 0.2) {
    if (!isStopped) {  // just entered stopped condition?
      isStopped = true;
      homeTickPosition = tickPositionDw;
    }
    if (isJigFwd) {
      if (tickPositionDw > (homeTickPosition + (TICKS_PER_FOOT / 2.0))) {
        isJigFwd = false;
      }
      path = 1;
      fpControllerSpeed = jigFps = goToFps(jigFps, 1.0);
    } else {
      if (tickPositionDw < (homeTickPosition - (TICKS_PER_FOOT / 2.0))) {
        isJigFwd = true;
      }
      path = 2;
      fpControllerSpeed = jigFps = goToFps(jigFps, -1.0);
    }
  } else {
    isStopped = false;
  }

  addLog(
    (long) (isStopped),
    (short) (isJigFwd),
    (short) (tickPositionDw),
    (short) (homeTickPosition),
    (short) (fpControllerSpeed * 100.0),
    (short) (fpsDw * 100.0),
    (short) (path)
  );


  // find the speed error
  double fpSpeedError = fpControllerSpeed - fpLpfCos;

  // compute a weighted angle to eventually correct the speed error
  fpTargetAngle = -(fpSpeedError * V_FACT); //************ Speed error to angle *******************

  // Compute maximum angles for the current wheel speed and enforce limits.
  float fwdA = fpsDw - 13.0;
  float bkwdA = fpsDw + 13.0;
  if (fpTargetAngle < fwdA)  fpTargetAngle = fwdA;
  if (fpTargetAngle > bkwdA) fpTargetAngle = bkwdA;

  // Compute angle error and weight factor
  fpAngleError = fpTargetAngle - gaPitch;  //
  fpCorrection = fpAngleError * wVal; //******************* Angle error to speed *******************
  fpLpfCorrection = (fpLpfCorrectionOld * (1.0f - X_FACT))  + (fpCorrection * X_FACT);
  fpLpfCorrectionOld = fpLpfCorrection;

  // Add the angle error to the base speed to get the target speed.
  fpFps = fpLpfCorrection + fpCos;

  setTargetSpeed(true, fpFps);
} // end fpAlgorithm()



/***********************************************************************.
    yBalance()
        Balance on the Y axis.  Go forward or backward  sideways to
        counteract falling left or right.
        Adjust the motor speed (fpFps) previously set by xBalance().
 ***********************************************************************/
void yBalance() {

}



/**************************************************************************.
    yawAction()    Called 200/sec.  Adjusts the Reaction Wheel Speed so that
                   FourPotatoe orients toward targetFpBearing
 **************************************************************************/
void yawAction(float target) {
  int path;
  static boolean isOldRwAccel = true;
  boolean isRwAccel = true;
  boolean isChange;
  float aDiff = rangeAngle(target - gyroHeading);
  float decelThresh = aDiff * DECEL_FACTOR;

  readSpeed(false);
  if (!isRunning) {
    zeroYaw();
    targetRwSpeed = 0.0;
  }

  if (abs(fpsRw) < 0.4) return;

  if (aDiff >= 0.0) {
    // Head clockwise
    if (gyroYawRate > decelThresh) {           // < speed threshold, accel
      isRwAccel = true;
      isChange = (isOldRwAccel != isRwAccel);
      if (isChange) {
        path = 1;
        targetRwSpeed = fpsRw + ACCEL_RATE;           //1
      }  else {
        path = 2;
        targetRwSpeed += ACCEL_RATE;                  //2
      }
    } else {                                  // > speed threshold decel
      isRwAccel = false;
      isChange = (isOldRwAccel != isRwAccel);
      if (isChange) {
        path = 3;
        targetRwSpeed = fpsRw - ACCEL_RATE;           //3
      } else {
        path = 4;
        targetRwSpeed -= ACCEL_RATE;                  //4
      }
    }
  } else {
    // Head counterclockwise
    if (gyroYawRate < decelThresh) {           // < speed threshold, accel
      isRwAccel = false;
      isChange = (isOldRwAccel != isRwAccel);
      if (isChange) {
        path = 5;
        targetRwSpeed = fpsRw - ACCEL_RATE;           //5
      } else {
        path = 6;
        targetRwSpeed -= ACCEL_RATE;                  //6
      }
    } else {                                  // > speed threshold decel
      isRwAccel = true;
      isChange = (isOldRwAccel != isRwAccel);
      if (isChange) {
        path = 7;
        targetRwSpeed = fpsRw + ACCEL_RATE;          //7
      } else {
       path = 8;
       targetRwSpeed += ACCEL_RATE;                  //8
      }
    }
  }
  setTargetSpeed(false, targetRwSpeed);
  isOldRwAccel = isRwAccel;
  
//  static unsigned int loop = 0;
//  if ((++loop % 100) == 0) {
//    sprintf(message, "aDiff: %5.2f \t thresh %9.2f \t yawRate: %5.2f \t tSpeed: %5.2f", aDiff, decelThresh, gyroYawRate, targetRwSpeed);
//    sendBMsg(SEND_MESSAGE, message);
//  }

//  addLog(
//    (long) (decelThresh * 100.0),
//    (short) (gyroHeading * 100.0),
//    (short) (target * 100.0),
//    (short) (gyroYawRate * 100.0),
//    (short) (targetRwSpeed * 100.0),
//    (short) (fpsDw * 100.0),
//    (short) (path)
//  );
}



/***********************************************************************.
    sendLog() Called 208 times/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;
  logLoop++;

  if (isDumpingData) {
    if ((logLoop % 4) == 0)  dumpData();
  }

    if ((logLoop % 104) == 5) log2PerSec();
  //  if ((logLoop % 21) == 5) log10PerSec();  // 10/sec
  //  if ((logLoop % 10) == 7) routeLog(); // 20/sec
  //  log200PerSec();
}


void log2PerSec() {
//  Serial.print(battVolt); Serial.print("\t");
//  Serial.print(fpLpfCos); Serial.print("\t");
//  Serial.print(gPitch); Serial.print("\t");
//  Serial.print(gaPitch); Serial.print("\t");
//  Serial.println();
  sprintf(message, "gyroYawRaw: %6.2f \t fpLpfCos: %5.2f \t fpTargetAngle: %5.2f", gyroYawRaw, fpLpfCos, fpTargetAngle);
  sendBMsg(SEND_MESSAGE, message);
}


void log10PerSec() {
  Serial.print(battVolt); Serial.print("\t");
  Serial.print(fpsDw); Serial.print("\t");
  Serial.print(fpsRw); Serial.print("\t");
  Serial.println();
}


void log200PerSec() {
  static int loop = 0;
  static float sum = 0.0;
  sum += fpsDw;
  if (loop++ >= 208) {
    Serial.print(battVolt); Serial.print("\t");
    Serial.print(sum / 208.0); Serial.print("\t");
    Serial.println();
    sprintf(message, "fpsDw %4.2f", (sum / 208.0));
    sendBMsg(SEND_MESSAGE, message);
    loop = 0;
    sum = 0.0;
  }
}



/***********************************************************************.
 *  goToFps() change the fpControllerSpeed toward the target
 ***********************************************************************/
float goToFps(float currentFps, float targetFps) {
  float resultFps;
  if (targetFps >= currentFps) {
    resultFps = currentFps + GO_TO_INCREMENT;
    if (resultFps > targetFps) resultFps = targetFps;
  } else {
    resultFps = currentFps - GO_TO_INCREMENT;
    if (resultFps < targetFps) resultFps = targetFps;
  }
  return resultFps;
}



