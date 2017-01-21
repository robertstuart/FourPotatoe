
const int B_BUFFER_SIZE = 100;
char msgStrB[B_BUFFER_SIZE];
int msgStrPtrB = 0;
int msgCmdB = 0;
boolean isMessageInProgressB = false;
char msgStrX[B_BUFFER_SIZE];
int msgStrPtrX = 0;
boolean isMessageInProgressX = false;

void   readBluetooth() {
  while (BLUE_SER.available()) {
    byte b = BLUE_SER.read();
    if (b >= 128) {
      msgStrPtrB = 0;
      msgCmdB = b;
      isMessageInProgressB = true;
    }
    else {
      if (isMessageInProgressB) {
        if (msgStrPtrB >= B_BUFFER_SIZE) {
          isMessageInProgressB = false;
        } else if (b == 0) {
          msgStrX[msgStrPtrX] = 0;
          doMsg(msgCmdB, msgStrB, msgStrPtrB, false);
        } 
        else {
          msgStrB[msgStrPtrB++] = b;
          tPc = timeMilliseconds;
        }
      }
    }
  }
}




void doMsg(int cmd, char msgStr[], int count, boolean isHc) {
  int intVal;
  float floatVal;
  boolean booleanVal;
//  int x = 0;
  String ss;
  
  msgStr[count] = 0; // Just to be sure.
//  if ((cmd != RCV_JOYX) && (cmd != RCV_JOYY)) {
//    Serial.print(cmd); Serial.print("  "); Serial.println(msgStr);
//  }
 
  switch(cmd) {
    case RCV_JOYX:
      if (sscanf(msgStr, "%f", &floatVal) >0) {
        if (isHc) {
          hcX = floatVal;
          if (abs(hcX) > abs(pcX)) controllerX = floatVal;
        } else {
          pcX = floatVal;
          if (abs(pcX) > abs(hcX)) controllerX = floatVal;
          sendStatusBluePc();
        }
        if ((abs(hcX) < 0.02) && (abs(pcX) < 0.02)) controllerX = 0.0; 
      }
      break;
    case RCV_JOYY:
      if (sscanf(msgStr, "%f", &floatVal) > 0) {
        if (isHc) {
          hcY = floatVal;
          if (abs(hcY) > abs(pcY)) controllerY = floatVal;
        } else {
          pcY = floatVal;
          if (abs(pcY) > abs(hcY)) controllerY = floatVal;
        }
        if ((abs(hcY) < 0.02) && (abs(pcY) < 0.02)) controllerY = 0.0;
      }
      break;
    case RCV_RUN:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
       isRunReady = (intVal != 0);
      }
      break;
    case RCV_MODE:
      if (sscanf(msgStr, "%d", &intVal) > 0) {
        mode = intVal;
        Serial.print("Mode: "); Serial.println(mode);
      }
      break;
    case RCV_DUMP_START:
      sendDumpData();
      break;
    case RCV_T: // valset t
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        tVal = (int) floatVal;
      } 
      break;
    case RCV_U:  // valset u
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        uVal = (int) floatVal;
      } 
      break;
    case RCV_V:  // valset v
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        vVal = floatVal;
      } 
      break;
    case RCV_W:  // valset w
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        wVal = floatVal;
      } 
      break;
    case RCV_X:  // valset x
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        xVal = floatVal;
      } 
      break;
    case RCV_Y:  // valset y
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        yVal = floatVal;
      } 
      break;
    case RCV_Z:  // valset z
      if(sscanf(msgStr, "%f", &floatVal) > 0) {
        zVal = floatVal;
      } 
      break;
    default:
      Serial.print("Illegal message received: "); Serial.println(cmd);
      break;
  }
}



void setValSet(int newValSet) {
}



const int RF_DATA_SIZE = 72;
byte rfData[RF_DATA_SIZE];
int rfDataPtr = 0;
//byte txRequestDataFrame[100];
//byte packetBuffer[120] = {0x7E}; 

int dumpPtr, dumpEnd;
int tickDumpPtr, tickDumpEnd;

/*********************************************************
 * SendStatus??() 
 *********************************************************/
void sendStatusBluePc() {
  static int part = 0;
  part = ++part % 2;
  switch (part) {
    case 0:
      sendBMsg(SEND_FPS, 2, fpsDw);
      break;
    case 1:
      sendBMsg(SEND_BATT_A, 2, battVolt); 
      break;
  }
  sendBMsg(SEND_STATE, getState());
}



int getState() {
  int statusInt = 0;
//  if (isRunning)          statusInt += 1;
//  if (isRunReady)         statusInt += 2;
//  if (isUpright)          statusInt += 4;
//  if (isLifted)           statusInt += 8;
//  if (isHcActive)         statusInt += 16;
//  if (isPcActive)         statusInt += 32;
//  if (isRouteInProgress)  statusInt += 64;
//  if (isDumpingData)      statusInt += 128;
//  if (isHoldHeading)      statusInt += 256;
//  if (isSpin)             statusInt += 512;
//  if (false)              statusInt += 1024; // empty
//  if (isStand)            statusInt += 2048;
  return statusInt;
}


/*********************************************************
 * dumpData() Set up to start a data dump.
 *********************************************************/
void sendDumpData() {
  dumpEnd = dumpPtr =  dataArrayPtr;
  isDumpingData = true;
}

///*********************************************************
// * dumpTicks() Set up to start a tick dump.
// *********************************************************/
//void sendDumpTicks() {
//  tickDumpEnd = tickDumpPtr =  tickArrayPtr;
//  isDumpingTicks = true;
//}



/*********************************************************
 * dumpData()
 *********************************************************/
void dumpData() {
  BLUE_SER.write(SEND_DUMP_DATA);
  dumpPtr = (dumpPtr + 1) %  DATA_ARRAY_SIZE;
  if (dumpPtr != dumpEnd) {
    BLUE_SER.print(aArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(bArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(cArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(dArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(eArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(fArray[dumpPtr]); BLUE_SER.print(",");
    BLUE_SER.print(gArray[dumpPtr]);
  }
  else {
    BLUE_SER.print("12345678");
    isDumpingData = false;
  }
  BLUE_SER.write((byte) 0);
}



///*********************************************************
// * dumpTicks()
// *********************************************************/
//void dumpTicks() {
//  BLUE_SER.write(SEND_DUMP_TICKS);
//  tickDumpPtr = (tickDumpPtr + 1) %  TICK_ARRAY_SIZE;
//  if (tickDumpPtr != tickDumpEnd) {
//    BLUE_SER.print(tArray[tickDumpPtr]); BLUE_SER.print(",");
//    BLUE_SER.print(uArray[tickDumpPtr]);
//  }
//  else {
//    BLUE_SER.print("12345678");
//    isDumpingTicks = false;
//  }
//  BLUE_SER.write((byte) 0);
//}



/*********************************************************
 * send?Msg()
 *********************************************************/
void sendBMsg(int cmd, int precision, double val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val, precision);
  BLUE_SER.write((byte) 0);
}

void sendBMsg(int cmd, int val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val);
  BLUE_SER.write((byte) 0);
}

void sendBMsg(int cmd, String val) {
  BLUE_SER.write(cmd);
  BLUE_SER.print(val);
  BLUE_SER.write((byte) 0);
}

