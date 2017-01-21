void fpRun() {
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  tickPosition = 0L;
  setBlink(RE_LED, BLINK_SB);
  delay(200);
  while(mode == MODE_FP) { // main loop
    commonTasks();
    static int intCount = 0;
    static unsigned int loopTrigger = 0;
    // Do the timed loop
    timeMicroseconds = micros();
    timeMilliseconds = timeMicroseconds / 1000;
    if (isNewGyro()) {
      setGyroData();
      xBalance(); 
      yBalance();
      sendLog();
//      safeAngle();
      checkMotor(true);
//      checkMotorRw();
    }
    if (isNewAccel()) {
      setAccelData();
    }
  } 
}



/***********************************************************************.
 *  xBalance() 
 *      Balance on the X axis.  Go forward or backward to counteract
 *      falling forward or backward.
 *      Set the motor speed (fpFps).
 ***********************************************************************/
void xBalance() {
  readSpeed(true);
  // compute the Center of Oscillation Speed (COS)
  fpRotation = U_FACT * (-gyroPitchDelta); // 7.4?
  fpCos = fpsDw + fpRotation; // subtract rotation 
  fpLpfCos = (fpLpfCosOld * (1.0 - COS_TC))  + (fpCos  * COS_TC); // smooth it out a little (0.2)
  fpLpfCosAccel = fpLpfCos - fpLpfCosOld;
  fpLpfCosOld = fpLpfCos;

  fpControllerSpeed = controllerY * SPEED_MULTIPLIER; 

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
 *  yBalance() 
 *      Balance on the Y axis.  Go forward or backward  sideways to 
 *      counteract falling left or right.
 *      Adjust the motor speed (fpFps) previously set by xBalance().
 ***********************************************************************/
void yBalance() {
  
}




/***********************************************************************.
 *  sendLog() Called 208 times/sec.
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
  Serial.print(battVolt); Serial.print("\t");
  Serial.print(tickPositionDw); Serial.print("\t");
  Serial.print(fpsDw); Serial.print("\t");
  Serial.print(targetPwDw); Serial.print("\t");
  Serial.print(isUpright); Serial.print(isRunReady); Serial.print(isRunning); Serial.print("\t");
  Serial.println();
        sprintf(message, "fpFps: %5.2f \t targetPwDw: %5.2f \t late: %d", fpFps, fpsDw, targetPwDw);
        sendBMsg(SEND_MESSAGE, message);
}


void log10PerSec() {
  Serial.print(battVolt); Serial.print("\t");
  Serial.print(fpsDw); Serial.print("\t");
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
  



