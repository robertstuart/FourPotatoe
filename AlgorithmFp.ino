void fpRun() {
//  timeMicroseconds = micros();
//  timeMilliseconds = timeMicroseconds / 1000;
  tickPosition = 0L;
  tVal = T_VAL;
  uVal = U_VAL;
  vVal = V_VAL;
  wVal = W_VAL;
  xVal = X_VAL;
  yVal = Y_VAL;
  zVal = Z_VAL;
//  setBlink(RE_LED, BLINK_SB);
  delay(100);  // For switches()
  while (mode == MODE_FP) { // main loop
    commonTasks();
//    static int intCount = 0;
//    static unsigned int loopTrigger = 0;
    // Do the timed loop
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if (isNewGyro()) {
      setGyroData();
      readRcRadio();
      pitchBalance();
      rollBalance();
//      yawAction(0.0);
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
  readSpeed(true);
  // compute the Center of Oscillation Speed (COS)
  fpRotation = U_VAL * (-gyroPitchDelta); // 10.0 for 4potatoe
  fpCos = fpsDw + fpRotation; // subtract rotation
  fpLpfCos = (fpLpfCosOld * (1.0 - COS_TC))  + (fpCos  * COS_TC); // smooth it out a little (0.2)
  fpLpfCosAccel = fpLpfCos - fpLpfCosOld;
  fpLpfCosOld = fpLpfCos;

  fpControllerSpeed = controllerY * SPEED_MULTIPLIER;

  fpControllerSpeed = jigZero(fpControllerSpeed); 

  // find the speed error
  fpSpeedError = fpControllerSpeed - fpLpfCos;

  // compute a weighted angle to eventually correct the speed error
  fpTargetAngle = -(fpSpeedError * V_VAL); //************ Speed error to angle *******************

  // Compute maximum angles for the current wheel speed and enforce limits.
  float fwdA = fpsDw - 16.0;
  float bkwdA = fpsDw + 16.0;
  if (fpTargetAngle < fwdA)  fpTargetAngle = fwdA;
  if (fpTargetAngle > bkwdA) fpTargetAngle = bkwdA;

  // Compute angle error and weight factor
  fpAngleError = fpTargetAngle - gaPitch;  //
  fpCorrection = fpAngleError * wVal; //******************* Angle error to speed *******************
  fpLpfCorrection = (fpLpfCorrectionOld * (1.0f - X_VAL))  + (fpCorrection * X_VAL);
  fpLpfCorrectionOld = fpLpfCorrection;

  // Add the angle error to the base speed to get the target speed.
  fpFps = fpLpfCorrection + fpCos;

  setTargetSpeed(true, fpFps);
} // end fpAlgorithm()



/***********************************************************************.
    jigZero()  Jig back and fort then fpControllerSpeed is low.
 ***********************************************************************/
const float JIG_SPEED = 4.0;     // Target speed during jig
const float JIG_DISTANCE = 20.0;  // distance from center (JigTickPosition
const float GO_TO_INCREMENT = 0.03;

float jigZero(float controllerSpeed) {
  static int jigHomeTickPosition = 0;
  static float jigFps = 0.0;
  static boolean isJigFwd = true;
  int path = 0;
  float newSpeed = 0;
  if (abs(controllerSpeed) < 0.2) {
    if (!isJig) {  // just entered stopped condition?
      isJig = true;
      isJigFwd = true;
      jigHomeTickPosition = tickPositionDw + (TICKS_PER_FOOT * 3.0);  // Jig in front of start
    }
    if (isJigFwd) {
      if (tickPositionDw > (jigHomeTickPosition + (TICKS_PER_FOOT * JIG_DISTANCE))) {
        isJigFwd = false;
      }
      path = 1;
      newSpeed = jigFps = goToFps(jigFps, JIG_SPEED, GO_TO_INCREMENT);
    } else {
      if (tickPositionDw < (jigHomeTickPosition - (TICKS_PER_FOOT * JIG_DISTANCE))) {
        isJigFwd = true;
      }
      path = 2;
      newSpeed = jigFps = goToFps(jigFps, -JIG_SPEED, GO_TO_INCREMENT);
    }
  } else {
    isJig = false;
    newSpeed = controllerSpeed;
  }

//    sprintf(message, "jigTickPosition: %6d \t tickPositionDw: %6d \t", jigTickPosition, tickPositionDw);
//  sendBMsg(SEND_MESSAGE, message);

//
//  addLog(
//    (long) (isStopped),
//    (short) (isJigFwd),
//    (short) (tickPositionDw),
//    (short) (jigTickPosition),
//    (short) (fpControllerSpeed * 100.0),
//    (short) (fpsDw * 100.0),
//    (short) (path)
//  );

  return newSpeed;
}



/***********************************************************************.
    rollBalance()
 ***********************************************************************/
const float ROLL_CORRECTION = 3.2; // start w 2.0
float targetBearing = 0.0;  // temp, will add rc steering control

void rollBalance() {
  float correction = 0.0;
  float raw = gaRoll * ROLL_CORRECTION;
//  raw -= (targetBearing - gyroHeading) * BEARING_CORRECTION;
  if (abs(fpsDw) > 0.5) {
    correction = raw / fpLpfCos;
  } else {
    correction = 0.0;
  }
  yawAction(gyroHeading + correction);
  
//  static unsigned int loop = 0;
//  if ((++loop % 104) == 0) {
//    sprintf(message, "gyroYawRate: %6.2f \t gyroHeading: %6.2f", gyroYawRate, gyroHeading);
//    sendBMsg(SEND_MESSAGE, message);
//  }
}



/**************************************************************************.
 *  yawAction()    Called 200/sec.  Adjusts the Reaction Wheel Speed so that
 *                 FourPotatoe orients toward target
 **************************************************************************/
//const float DECEL_FACTOR = 30.0;     // Point to change rw accel/decel
const float DECEL_FACTOR = 60.0;     // Point to change rw accel/decel
const float DEAD_ANGLE_RANGE = 1.0;  //+- range where no action is taken
//const float ACCEL_RATE = 2.00;    // Change in rw speed to reach target angle
const float ACCEL_RATE = 1.50;    // Change in rw speed to reach target angle
const float COAST_RATE = 0.1;
const int FP_DEAD = 0;
const int FP_CW = 1;
const int FP_CCW = 2;

void yawAction(float targetHeading) {
  static int motion = FP_DEAD;  
  static float pastAccel[] = {0.0, 0.0, 0.0};
  static int pastAccelPtr = 0;
  static float sumPastAccel = 0.0;
  float estHeading = gyroHeading + (sumPastAccel * 0.1);
//  float aDiff = rangeAngle(targetHeading - estHeading);
  float aDiff = rangeAngle(targetHeading - gyroHeading);
  float decelThreshold = aDiff * DECEL_FACTOR;
  float rwAccel;
  int path;

  readSpeed(false);
  
  // Reaction wheel motion.
  if (motion == FP_DEAD) {
    if (aDiff > DEAD_ANGLE_RANGE) {
      motion = FP_CW;
    } else if (aDiff < -DEAD_ANGLE_RANGE) {
      motion = FP_CCW;
    } else {
      if (fpsRw > 0) rwAccel = - COAST_RATE;
      else rwAccel = COAST_RATE;
      if (abs(fpsRw) < COAST_RATE) rwAccel = - fpsRw;
      path = 0;
    }
  } 
  if (motion == FP_CW) {
    if (aDiff < 0.0) {
      rwAccel = + COAST_RATE;
      motion = FP_DEAD;
    } else { 
      if (gyroYawRate > decelThreshold) {
        rwAccel = ACCEL_RATE; 
        path = 1;
      } else {
        rwAccel = -ACCEL_RATE;
        path = 2;
      }
    }
  }
  if (motion == FP_CCW) {
    if (aDiff > 0.0) {
      rwAccel = - COAST_RATE;
      motion = FP_DEAD;
    } else {
      if (gyroYawRate < decelThreshold) {
        rwAccel = -ACCEL_RATE;
        path = 3;
      } else {
        rwAccel = ACCEL_RATE;
        path = 4;
      }
    }
  }

  // Log past motion it improve gyroHeading estimate
  float oldAccel = pastAccel[pastAccelPtr];
  pastAccel[pastAccelPtr] = rwAccel;
  sumPastAccel = sumPastAccel + rwAccel - oldAccel;
  pastAccelPtr = ++pastAccelPtr % (sizeof(pastAccel)/sizeof(float));

  setTargetSpeed(false, fpsRw + rwAccel);

  addLog(
    (long) (path + (isRunning * 100)),
    (short) (gyroYawRate * 100.0),
    (short) (gaRoll * 100.0),
    (short) (targetHeading * 100.0),
    (short) (gyroHeading * 100.0),
    (short) (fpsRw * 100.0),
    (short) (decelThreshold * 100.0)
  );
}

 
//const float DECEL_FACTOR = 15.0;     // Point to change rw accel/decel
//const float DEAD_ANGLE_RANGE = 2.0;  //+- range where no action is taken
//const float DEAD_GYRO_RATE = 20.0;  //+- range where no action is taken
//const float ACCEL_RATE = 2.00;    // Change in rw speed to reach target angle
//const float COAST_RATE = 0.05;
//
//void yawAction(float target) {
//  static boolean isOrienting = false;
//  int path;
//  boolean isChange;
//  float accelRate;
//  float aDiff = rangeAngle(target - gyroHeading);
//  float decelThresh = aDiff * DECEL_FACTOR;
//
//  readSpeed(false);
//  if (!isRunning) {
//    zeroYaw();
//    targetRwSpeed = 0.0;
//  }
//
//  // Adjust accelRate to zero as aDiff approaches 0;
////  static const float RANGE = 0.0;  // degrees where rate is decreased
//  float absADiff = abs(aDiff);
//  float absYawRate = abs(gyroYawRate);
////  if (absADiff < RANGE) {
////    accelRate = (absADiff / RANGE) * ACCEL_RATE;
////  } else {
//    accelRate = ACCEL_RATE;
////  }
//
////  if (abs(fpsDw) < 0.2) return;
//
//  if (aDiff > DEAD_ANGLE_RANGE) {
//    // Head clockwise
//    if (gyroYawRate < decelThresh) {           // < speed threshold? accel, path 1
//        path = 1;                              
//        targetRwSpeed = fpsRw - accelRate;          
//    } else {                                   // > speed threshold? decel, path 2
//        path = 2;                              
//        targetRwSpeed = fpsRw + accelRate;           
//    }
//  } else if (aDiff < -DEAD_ANGLE_RANGE) {
//    // Head counterclockwise
//    if (gyroYawRate > decelThresh) {            // > speed threshold? accel, path 3
//        path = 3; 
//        targetRwSpeed = fpsRw + accelRate;           
//    } else {                                    // < speed threshold? decel, path 4
//        path = 4;
//        targetRwSpeed = fpsRw - accelRate;          
//    }
//  } else if (gyroYawRate > DEAD_GYRO_RATE) { // moving l/r slow it down
//    path = 5;
//    targetRwSpeed = fpsRw + accelRate;
//  } else if (gyroYawRate < (-DEAD_GYRO_RATE)) {
//    path = 6;
//    targetRwSpeed = fpsRw - accelRate;
//  } else { // coast to zero fps
//    if (fpsRw > 0) targetRwSpeed = fpsRw - COAST_RATE;
//    else targetRwSpeed = fpsRw + COAST_RATE;
//    if (abs(targetRwSpeed) < COAST_RATE) targetRwSpeed = 0.0;
//    path = 0;
//    isOrienting = false;
//  }
//
//  setTargetSpeed(false, targetRwSpeed);
//
////  static unsigned int loop = 0;
////  if ((++loop % 100) == 0) {
////    sprintf(message, "aDiff: %5.2f \t thresh %9.2f \t yawRate: %5.2f \t tSpeed: %5.2f", aDiff, decelThresh, gyroYawRate, targetRwSpeed);
////    sendBMsg(SEND_MESSAGE, message);
////  }
//
////  if (isRunning)
//  addLog(
//    (long) (path + (isRunning * 100)),
//    (short) (gyroYawRate * 100.0),
//    (short) (decelThresh * 100.0),
//    (short) (target * 100.0),
//    (short) (gyroHeading * 100.0),
//    (short) (fpsRw * 100.0),
//    (short) (targetRwSpeed * 100.0)
//  );
//}
//


/***********************************************************************.
    sendLog() Called 208 times/sec.
 ***********************************************************************/
void sendLog() {
  static unsigned int logLoop = 0;
  logLoop++;

  if (isDumpingData) {
    if ((logLoop % 4) == 0)  dumpData();
  }
    //  if ((logLoop % 104) == 5) log2PerSec();
    //  if ((logLoop % 10) == 5) log20PerSec();  // 10/sec
    //  if ((logLoop % 10) == 7) routeLog(); // 20/sec
    //  log200PerSec();
}


void log2PerSec() {
//  Serial.print(battVolt); Serial.print("\t");
//  Serial.print(fpLpfCos); Serial.print("\t");
//  Serial.print(gPitch); Serial.print("\t");
//  Serial.print(gaPitch); Serial.print("\t");
//  Serial.println();
//  sprintf(message, "tickPositionDw: %6d \t", tickPositionDw);
//  sendBMsg(SEND_MESSAGE, message);
}


void log20PerSec() {
  addLog(
    (long) (0),
    (short) (fpsDw * 100.0),
    (short) (fpLpfCos * 100.0),
    (short) (gaPitch * 100.0),
    (short) (gyroHeading * 100.0),
    (short) (0),
    (short) (0)
  );
//  Serial.print(battVolt); Serial.print("\t");
//  Serial.print(fpsDw); Serial.print("\t");
//  Serial.print(fpsRw); Serial.print("\t");
//  Serial.println();
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
float goToFps(float currentFps, float targetFps, float goToIncrement) {
  float resultFps;
  if (targetFps >= currentFps) {
    resultFps = currentFps + goToIncrement;
    if (resultFps > targetFps) resultFps = targetFps;
  } else {
    resultFps = currentFps - goToIncrement;
    if (resultFps < targetFps) resultFps = targetFps;
  }
  return resultFps;
}



