/***********************************************************************.
 *  imuInit()
 ***********************************************************************/
void imuInit() {
  imuReset();
  Wire.begin();
  if (!lsm6.init()) Serial.println("IMU initialize failed!");
  else Serial.println("IMU Initialized!****************************");
  lsm6.enableDefault();
  lsm6.writeReg(LSM6::INT1_CTRL, 0X02); // Accel data on INT1
  lsm6.writeReg(LSM6::INT2_CTRL, 0X01); // Gyro data ready on INT2
//  lsm6.writeReg(LSM6::CTRL2_G, 0X6C);   // Gyro 2000fs, 416hz
  lsm6.writeReg(LSM6::CTRL2_G, 0X5C);   // Gyro 2000fs, 208hz
  lsm6.writeReg(LSM6::CTRL1_XL, 0X40);  // Accel 2g, 104hz
}


/**************************************************************************.
 * isNew???()  Return true if new data has been read.
 **************************************************************************/
boolean isNewGyro() {
  if (digitalRead(GYRO_INT2) == HIGH) {
    lsm6.readGyro();
    return true;
  }
  return false;
}
boolean isNewAccel() {
  if (digitalRead(ACCEL_INT1) == HIGH) {
    lsm6.readAcc();
    return true;
  }
  return false;
}


#define TG_PITCH_TC 0.90D

/***********************************************************************.
 * setGyroData()
 ***********************************************************************/
void setGyroData() {
  
  // Pitch
  gyroPitchRaw = ((float) lsm6.g.y) - PITCH_DRIFT;
  gyroPitchRate = (((float) gyroPitchRaw) * GYRO_SENS);  // Rate in degreesChange/sec
  gyroPitchDelta = -gyroPitchRate / 208.0; // degrees changed during period
  gPitch = gPitch + gyroPitchDelta;   // Used by tgPitch & debugging
  gaPitch = gyroPitchDelta + gaPitch;  // used in weighting final angle
  gaRealPitch = gyroPitchDelta + gaPitch;  // Used for "isUpright" 
  
  // Roll
  gyroRollRaw = ((float) lsm6.g.z)- ROLL_DRIFT;
  gyroRollRate = (((float) gyroRollRaw) * GYRO_SENS);
  gyroRollDelta = -gyroRollRate / 208.0;
  gRoll = gRoll - gyroRollDelta;
  gaRoll = gaRoll - gyroRollDelta;

  // Yaw
  gyroYawRaw = ((float) lsm6.g.x) - YAW_DRIFT; 
  gyroYawRate = ((float) gyroYawRaw) * GYRO_SENS;  // Rate in degreesChange/sec
  gyroYawDelta = gyroYawRate / 208.0; // degrees changed during period
  gYaw += gyroYawDelta;
  gyroCumHeading += gyroYawDelta;   //
  float tc = (gyroCumHeading > 0.0) ? 180.0 : -180.0;
  int rotations = (int) ((gyroCumHeading + tc) / 360.0);
  gyroHeading = gyroCumHeading - (((float) rotations) * 360.0);
}

void zeroYaw() {
  gYaw = 0.0;
  gyroCumHeading = 0.0;
  gyroHeading = 0.0;
}


/***********************************************************************.
 *  setAccelData()
 ***********************************************************************/
void setAccelData() {

  // Pitch
  float k8 = 45.5;  // for new MinImu
//  double accelZ = lsm6.a.z + (k8 * 1000.0 * tp6LpfCosAccel);
  float accelZ = lsm6.a.z;
  aPitch = ((atan2(accelZ, -lsm6.a.x)) * RAD_TO_DEG) + zVal;
  gaRealPitch = (gaRealPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
  if (          (lsm6.a.x < 11000)
             && ((accelZ > -11000) && (accelZ < 11000))
             && ((aPitch > -45.0) && (aPitch < 45.0))) {
    gaPitch = (gaPitch * GYRO_WEIGHT) + (aPitch * (1 - GYRO_WEIGHT));
  }

  // Roll
  aRoll =  (atan2(lsm6.a.y, -lsm6.a.x) * RAD_TO_DEG);
  gaRoll = (gaRoll * GYRO_WEIGHT) + (aRoll * (1 - GYRO_WEIGHT)); // Weigh factors
}



/**************************************************************************.
 * resetIMU()  From: https://forum.arduino.cc/index.php?topic=386269.0
 *             I2C clocks to make sure no slaves are hung in a read
 *             at startup
 **************************************************************************/
void imuReset() {
  // Issue 20 I2C clocks to make sure no slaves are hung in a read
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(70, OUTPUT);
  pinMode(71, OUTPUT);
  digitalWrite(20, LOW);
  digitalWrite(70, LOW);
  for (int i = 0; i < 1000; i++)
  {
    digitalWrite(21, LOW);
    digitalWrite(71, LOW);
    delayMicroseconds(10);
    digitalWrite(21, HIGH);
    digitalWrite(71, HIGH);
    delayMicroseconds(10);
  }
}

