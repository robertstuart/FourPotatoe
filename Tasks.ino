
int beepCycleCount = 0;
boolean beepStat = false;
int *beepSequence;
int beepPtr = 0;

const byte* patternBd = BLINK_OFF;
const byte* patternBlue = BLINK_OFF;
const byte* patternRed = BLINK_OFF;
int blinkPtrBd = 0;
int blinkPtrBlue = 0;
int blinkPtrRed = 0;

/**************************************************************************.
 *
 * commonTasks()
 *
 *    Execute all tasks common to every algorithm.  This includes:
 *      1. Reading the XBee for commands.
 *      2. Flushing the serial output buffer
 *      3. Setting the time variables.
 *      4. Dumping any data if we are in "dump data" mode.
 *      5. Turning the power off if the motors have been idle.
 *      6. Flash the led.
 *
 **************************************************************************/
void commonTasks() {
  timeMicroseconds = micros();
  timeMilliseconds = timeMicroseconds / 1000;
  readBluetooth();
  rwSpin();
  blinkLed();
  battery();
  controllerConnected();
  switches();
  safeAngle();
  setRunningState();
}



/**************************************************************************.
 *
 * setRunningState()
 *
 *     Set the STATE_RUNNING bit if the following are true:
 *         RUN_READY is true
 *         STATE_UPRIGHT is true
 *         
 *      Set x and y to zero if there is no connection to
 *      a controller.
 *
 *      Set blinking according to the above states.
 *
 **************************************************************************/
void setRunningState() {
  byte *bk;

  // Set isRunning bit.
  if (mode == MODE_FP) { 
    if (isRunReady && isUpright && (rwState == RW_RUNNING)) {
       isRunning = true;
       isJig = false;
    } else {
      isRunning = false;
    }
  } else if (mode == MODE_RW_ANGLE) {
    if (isRunReady && (rwState == RW_RUNNING)) isRunning = true;
    else isRunning = false;
  } else { // For all test modes, just set accoding to ready bit
    isRunning = isRunReady;
  }

  // set LEDs
  switch(mode) {
    case MODE_FP: // set once at start of main loop.
      break;
    case MODE_PWM_SPEED:
    case MODE_T_SPEED:
      setBlink(BU_LED, isRunning ? BLINK_ON : BLINK_OFF);
      setBlink(RE_LED, BLINK_FF);
      setBlink(BD_LED, BLINK_FF);
      break;
    case MODE_RW_ANGLE:
      setBlink(BU_LED, isRunReady ? BLINK_ON : BLINK_OFF);
      setBlink(RE_LED, BLINK_FF);
      setBlink(BD_LED, BLINK_FF);
     break;
  }
    
  if (!isController) controllerX = controllerY = 0.0f;

  // Turn motors on or off
  if (((mode == MODE_FP) || (mode == MODE_PWM_SPEED)) && IS_ENABLE_DW) {
    digitalWrite(PWM_DW, isRunning ? HIGH : LOW);
  }
  if (((mode == MODE_T_SPEED) || (mode == MODE_PWM_SPEED)) && IS_ENABLE_RW) {  
    digitalWrite(PWM_RW, isRunning ? HIGH : LOW);
  } else {
    digitalWrite(PWM_RW, (rwState == RW_STOP) ? LOW : HIGH);
  }
}


/**************************************************************************.
 * controllerConnected()
 *********************************************************/
void controllerConnected() {
//  isHcActive =  ((tHc + 1000) > timeMilliseconds) ? true : false;
  isPcActive =  ((tPc + 1000) > timeMilliseconds) ? true : false;
  isController = (isHcActive || isPcActive);
}



/**************************************************************************.
 * battery()
 ***************************************************************/
void battery() {
  static unsigned long batteryTrigger = 0L;
  static int lowCount = 0;
  if (timeMilliseconds > batteryTrigger) {
    batteryTrigger = timeMilliseconds + 1000;  // 1 per second
    battVolt = ((float) analogRead(BATTERY)) * 0.0174;
    if( battVolt < 10.0) lowCount++;
    else lowCount = 0;
    if ((lowCount > 10) && !isBattAlarmDisabled) {
      lowCount = 0;
      beep(BEEP_WARBLE);
    }
  }
}


 
/*********************************************************
 *
 * safeAngle()
 *
 *     Check to see if we have fallen sidways or forwards
 *     set the STATE_UPRIGHT bit accordingly.
 *
 *********************************************************/
void safeAngle() {
  static unsigned long tTime = 0UL; // time of last state change
  static boolean tState = false;  // Timed state. true = upright

  boolean cState = ((abs(gaRealPitch) < 30.0) && ((abs(gaRoll) < 30.0))); // Current real state
  if (cState != tState) {
    tTime = timeMilliseconds; // Start the timer for a state change.
    tState = cState;
  }
  else {
    if ((timeMilliseconds - tTime) > 50) {
      isUpright = cState;
    }
  }
}



/**************************************************************************.
 * switches()
 *      
 **************************************************************************/
void switches() {
  static unsigned int buTimer = 0;
  static boolean buState = false;
  static boolean oldBuState = false;

  static unsigned int reTimer = 0;
  static boolean reState = false;
  static boolean oldReState = false;

  static unsigned int xTimer = 0;
  static boolean xState = false;
  static boolean oldXState = false;

  static unsigned int yTimer = 0;
  static boolean yState = false;
  static boolean oldYState = false;

  static unsigned int zTimer = 0;
  static boolean zState = false;
  static boolean oldZState = false;

  static boolean oldRadioState = false;
  
  // Debounce Blue
  boolean bu = digitalRead(BU_SWITCH) == LOW;
  if (bu) buTimer = timeMilliseconds;
  if ((timeMilliseconds - buTimer) > 50) buState = false;
  else buState = true;
  
  // Debounce Red
  boolean re = digitalRead(RE_SWITCH) == LOW;
  if (re) reTimer = timeMilliseconds;
  if ((timeMilliseconds - reTimer) > 50) reState = false;
  else reState = true;
  
  // Debounce X
  boolean x = digitalRead(X_SWITCH) == LOW;
  if (x) xTimer = timeMilliseconds;
  if ((timeMilliseconds - xTimer) > 50) xState = false;
  else xState = true;

  // Blue press transition
  if (buState && (!oldBuState)) {
    if (!isRunning) {
      if (rwState == RW_RUNNING) rwState = RW_SPIN_DOWN;
      else if (rwState == RW_STOP) rwState = RW_SPIN_UP;
      else if (rwState == RW_SPIN_UP) rwState = RW_SPIN_DOWN;
      else if (rwState == RW_SPIN_DOWN) rwState = RW_SPIN_UP;
    }
  }

  // Red press transition
  if ((reState) && (!oldReState)) isRunReady = !isRunReady;

  // Radio
  if (ch3sw != oldRadioState) {
    if (ch3sw) isRunReady = true;
    else isRunReady = false;
  }
  oldRadioState = ch3sw;

  // X transition
  if (xState != oldXState) isBattAlarmDisabled = true;

  oldBuState = buState;
  oldReState = reState;
  oldXState = xState;
  oldYState = yState;
  oldZState = zState;
}


/**************************************************************************.
 *  blinkLed() Call at least 10/sec
 **************************************************************************/
void blinkLed() {
  static unsigned long blinkTrigger = 0L;
  boolean isRoll = (mode == MODE_FP) && !isRunReady;
  if (isRoll) {
    blinkRoll();
  }

  if (timeMilliseconds > blinkTrigger) {
    blinkTrigger = timeMilliseconds + 100;  // 10 per second

    int b = (patternBd[blinkPtrBd++] == 1) ? HIGH: LOW;
    if (patternBd[blinkPtrBd] == END_MARKER) blinkPtrBd = 0;
    digitalWrite(BD_LED, b);

    if (!isRoll) {
      b = (patternBlue[blinkPtrBlue++] == 1) ? HIGH : LOW;
      if (patternBlue[blinkPtrBlue] == END_MARKER) blinkPtrBlue = 0;
      digitalWrite(BU_LED, b);
      
      b = (patternRed[blinkPtrRed++] == 1) ? HIGH : LOW;
      if (patternRed[blinkPtrRed] == END_MARKER) blinkPtrRed = 0;
      digitalWrite(RE_LED, b);
    }
  }
}



/**************************************************************************.
 *  setBlink() Set blink patter for led
 **************************************************************************/
void setBlink(int led, byte* pattern) {
  switch (led) {
    case RE_LED:
      if (patternRed != pattern) {
        patternRed = pattern;
        blinkPtrRed = 0;
      }
      break;
    case BU_LED:
      if (patternBlue != pattern) {
        patternBlue = pattern;
        blinkPtrBlue = 0;
      }
      break;
    case BD_LED:
      if (patternBd != pattern) {
        patternBd = pattern;
        blinkPtrBd = 0;
      }
      break;
    default:
      break;
  }
}



/**************************************************************************.
 *  blinkRoll() Blink red & blue light to indicate roll
 **************************************************************************/
void blinkRoll() {
  static unsigned int lastBlink;
  static boolean toggle = false;
  
  int blinkPeriod = abs((int) (200.0 * gaRoll));
  if (blinkPeriod > 1000) blinkPeriod = 1000;
  if ((timeMilliseconds - lastBlink) > blinkPeriod) {
    toggle = !toggle;
    if ( gaRoll < 0.0) {
      digitalWrite(BU_LED, toggle ? HIGH : LOW);
      digitalWrite(RE_LED, LOW);
    } else {
      digitalWrite(RE_LED, toggle ? HIGH : LOW);
      digitalWrite(BU_LED, LOW);
    }
    lastBlink = timeMilliseconds;
  }    
}



/**************************************************************************.
 *  beep() 
 **************************************************************************/
void beep(int seq[]) {
  beepPtr = 0;
  beepSequence = seq;
  Timer3.attachInterrupt(beepIsr);
  setBeep();
}

void setBeep() {
  int freq = beepSequence[beepPtr];
  if (freq != 0) {
    int halfCycle = (1000000 / 2) / freq;
    int dur = beepSequence[beepPtr + 1];
    beepCycleCount = (dur * 1000) / halfCycle;
    Timer3.start(halfCycle);
    beepPtr += 2;
  }
  else {
    Timer3.stop();
    Timer3.detachInterrupt();
    digitalWrite(SPEAKER, LOW);
  }
}



void beepIsr() {
  if (--beepCycleCount <= 0) {
    setBeep();
  }
  beepStat = !beepStat;
  digitalWrite(SPEAKER, beepStat);
}



/**************************************************************************.
 *  rcRadioInit() 
 **************************************************************************/
void rcRadioInit() {
  pinMode(CH2_RADIO, INPUT);
  pinMode(CH3_RADIO, INPUT);
  pinMode(CH4_RADIO, INPUT);
  attachInterrupt(CH2_RADIO, ch2Isr, CHANGE);
  attachInterrupt(CH3_RADIO, ch3Isr, CHANGE);
  attachInterrupt(CH4_RADIO, ch4Isr, CHANGE);
}



/**************************************************************************.
 *  chXIsr() Interrupt routines for radio pulses
 **************************************************************************/
void ch2Isr() {
  unsigned int t = micros();
  boolean enc = (!!(g_APinDescription[CH2_RADIO].pPort -> PIO_PDSR & g_APinDescription[CH2_RADIO].ulPin)) ? true : false;
  if (enc) {
    ch2riseTime = t;
  } else {
    ch2pw = t - ch2riseTime; 
  }
}
void ch3Isr() {
  unsigned int t = micros();
  boolean enc = (!!(g_APinDescription[CH3_RADIO].pPort -> PIO_PDSR & g_APinDescription[CH3_RADIO].ulPin)) ? true : false;
  if (enc) {
    ch3riseTime = t;
  } else {
    ch3pw = t - ch3riseTime; 
  }
}
void ch4Isr() {
  unsigned int t = micros();
  boolean enc = (!!(g_APinDescription[CH4_RADIO].pPort -> PIO_PDSR & g_APinDescription[CH4_RADIO].ulPin)) ? true : false;
  if (enc) {
    ch4riseTime = t;
  } else {
    ch4pw = t - ch4riseTime; 
  }
}



/**************************************************************************.
 *  readRcRadio() 
 **************************************************************************/
void readRcRadio() {
  float p;
  int deadTime = timeMicroseconds - ch2riseTime;
  if (deadTime > 200000) {
    controllerX = 0.0;
    controllerY = 0.0;
    ch3sw = false;
  } else {
    p = ((float) (ch2pw - 1500));
    controllerY = p / 326.0;
    p = ((float) (ch4pw - 1500));
    controllerX = p / 376.0;
    ch3sw = ch3pw < 1100;
    controllerY = constrain(controllerY, -1.0, 1.0);
    controllerX = constrain(controllerX, -1.0, 1.0);
  }
}


/**************************************************************************.
 *  rwSpin() Spin up and spin down reaction wheel
 **************************************************************************/
void rwSpin() {
  const float UP_DOWN_RATE = 0.01;
  const float TARGET_RATE = 2.0;
  static unsigned int spinTrigger = 0L;
  if (timeMilliseconds > spinTrigger) {
    spinTrigger = timeMilliseconds + 10; // 100/sec
    switch (rwState) {
      case RW_SPIN_UP:
        if (targetRwFps < TARGET_RATE) {
          targetRwFps += UP_DOWN_RATE;
          if (targetRwFps > TARGET_RATE) {
            targetRwFps = TARGET_RATE;
            rwState = RW_RUNNING;
          }
        } else {
          targetRwFps -= UP_DOWN_RATE;
          if (targetRwFps < TARGET_RATE) {
            targetRwFps = TARGET_RATE;
            rwState = RW_RUNNING;
          }
        }
        setTargetSpeed(false, targetRwFps);
        break;
      case RW_SPIN_DOWN:
        if (targetRwFps > 0.0) {
          targetRwFps -= UP_DOWN_RATE;
          if (targetRwFps < 0.0) {
            targetRwFps = 0.0;
            rwState = RW_STOP;
          }
        } else {
          targetRwFps += UP_DOWN_RATE;
          if (targetRwFps > 0.0) {
            targetRwFps = 0.0;
            rwState = RW_STOP;
          }
        }
        setTargetSpeed(false, targetRwFps);
        break;
      case RW_RUNNING:
        if (!isRunReady) {
          if (targetRwFps != TARGET_RATE) rwState = RW_SPIN_UP;
        }
        break;
      case RW_STOP:
        if (targetRwFps != 0.0) rwState = RW_SPIN_DOWN;
        setTargetSpeed(false, targetRwFps);
        break;
    }
  }
} // end rwSpin()

// Routines output -1000 to +1000 for pulse widths of 1 MS to 1 MS
void   ch1Isr() {
//  static unsigned int highTime = 0;
//  unsigned int t = micros();
//  boolean isHigh = (!!(g_APinDescription[CH1_RADIO].pPort -> PIO_PDSR & g_APinDescription[CH1_RADIO].ulPin)) ? true : false;
//  if (isHigh) {
//    highTime = t;
//  } else {
//    ch1Val = ((t - highTime) - 1500) * 2;
//    lastCh1 = t;
//  }
}



/**************************************************************************.
 *  rangeAngle() Set angle value between -180 and +180
 **************************************************************************/
double rangeAngle(double head) {
  while (head > 180.0D) head -= 360.0D;
  while (head <= -180.0D) head += 360.0D;
  return head;
}



/**************************************************************************.
 *  addLog() Put values in the dump arrays.
 **************************************************************************/
void addLog(long aVal, short bVal, short cVal, short dVal, short eVal, short fVal, short gVal) {
  if (!isDumpingData) {
    if (aVal == 0L) aVal = 1L; // don't indicate end
    aArray[dataArrayPtr] = aVal;
    bArray[dataArrayPtr] = bVal;
    cArray[dataArrayPtr] = cVal;
    dArray[dataArrayPtr] = dVal;
    eArray[dataArrayPtr] = eVal;
    fArray[dataArrayPtr] = fVal;
    gArray[dataArrayPtr] = gVal;
    dataArrayPtr++;
    dataArrayPtr = dataArrayPtr %  DATA_ARRAY_SIZE;
  }
}
