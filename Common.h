
const int SEND_MESSAGE     = 129;
const int SEND_FPS         = 130;
const int SEND_PITCH       = 131;
const int SEND_DUMP_DATA   = 135;
const int SEND_STATE       = 136;
const int SEND_BATT_A      = 137;

const int RCV_JOYX         = 129;
const int RCV_JOYY         = 130;
const int RCV_RUN          = 131;
const int RCV_DUMP_START   = 135;
const int RCV_T            = 136;
const int RCV_U            = 137;
const int RCV_V            = 138;
const int RCV_W            = 139;
const int RCV_X            = 140;
const int RCV_Y            = 141;
const int RCV_Z            = 142;
const int RCV_MODE         = 152;

const int SEND_RCV_TERM    =   0;

// Modes of operation, from TwoPotatoe/Common.h
const int MODE_PWM_SPEED      = 1;  // pw values sent from controller
const int MODE_T_SPEED        = 4;  // speed controlled by ticks
const int MODE_FP             = 9;  // FourPotatoe normal operation
const int MODE_RW_ANGLE       = 11;  // Reaction wheel angle
const int BLOCK_DATA          = 100; 

// Motor Modes
const int MM_DRIVE_BRAKE = 0;
const int MM_DRIVE_COAST = 1;
