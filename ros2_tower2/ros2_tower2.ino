////////////////////////////////////////////////////
//
// VACUUM/TOWER CONTROL
//
////////////////////////////////////////////////////

#define USE_TIMER_1 true
#define USE_TIMER_2 false
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 false

#include "TimerInterrupt.h"

// GPIO ASSIGNMENTS

// Tower0 motor pin definitions
#define MT0S0_RESERVED              8
#define MT0S0_BOTTOM_LIMIT_SWITCH   7

#define MT0S0_ST_DIR                6
#define MT0S0_ST_EN                 5
#define MT0S0_ST_PL                 4
#define MT0S0_POS                  A4

// stage 1 unit 0
#define MT0S10_RESERVED             9
#define MT0S10_BOTTOM_LIMIT_SWITCH  13

#define MT0S10_ST_DIR               11
#define MT0S10_ST_EN                10
#define MT0S10_ST_PL                12
#define MT0S10_POS                  A5

// stage 1 unit 1
#define MT0S11_RESERVED             14
#define MT0S11_BOTTOM_LIMIT_SWITCH  15

#define MT0S11_ST_DIR               16
#define MT0S11_ST_EN                17
#define MT0S11_ST_PL                18
#define MT0S11_POS                  A6

//tower 1
#define MT1S0_RESERVED             29
#define MT1S0_BOTTOM_LIMIT_SWITCH  27

#define MT1S0_ST_DIR               25
#define MT1S0_ST_EN                31
#define MT1S0_ST_PL                33
#define MT1S0_POS                  A1

// stage 1 unit 0
#define MT1S10_RESERVED             35
#define MT1S10_BOTTOM_LIMIT_SWITCH  37

#define MT1S10_ST_DIR               39
#define MT1S10_ST_EN                41
#define MT1S10_ST_PL                43
#define MT1S10_POS                  A2

// stage 1 unit 1
#define MT1S11_RESERVED             45
#define MT1S11_BOTTOM_LIMIT_SWITCH  47

#define MT1S11_ST_DIR               49
#define MT1S11_ST_EN                51
#define MT1S11_ST_PL                53
#define MT1S11_POS                  A3

#define TANK_SENSOR                 A0

#define K1                          55
#define OUT_VALVE                   56
#define PUMP_ON1                    57
#define PUMP_ON2                    58

// TOWER STATES
#define TOWER_STOPPED        0
#define TOWER_STOPPED_SWITCH 1
#define TOWER_UP             2
#define TOWER_DOWN           3

#define TOWER_ACC    0
#define TOWER_STEADY 1
#define TOWER_DEC    2

#define LOOP_MODE_IDLE 0
#define LOOP_MODE_UP   1
#define LOOP_MODE_DOWN 2

// Constants
#define MAX_ARGV_LENGTH 16
#define PSI_TICKS 7.1f
#define ACCELERATION_TICK_THRESHOLD 200
#define TIMER_INTERVAL_MS 1
#define DEFAULT_LARGE_TICK_COUNT 100000
#define UNKNOWN_POSITION_TICKS 99999
#define CALIBRATION_LARGE_MOVE 150000

typedef struct MOTOR {
    int motorid;
    int armid;
    char enGpio;
    char dirGpio;
    char stepGpio;
    char positionAnalogPin;
    char reservedGpio;
    char limit1Gpio;
    char inverseDirection;
    char leaveEnabled;
    int  stepDelayTime;
    float  ticksPerMm;

    int  towerUpperLimitTicks;
    int  towerUpperLimitMm;
    int  towerLowerLimitMm;

    volatile int towerMode;
    long towerSteadyTrigger;
    long towerDecTrigger;
    volatile long towerTickCount;
    int towerIntervalCount;
    int towerIntervalTableIndex;
    char towerBuckets;
    long towerIntervalChangeTrigger;
    int towerIntervalChangeIncrement;

    volatile char towerDirection;
    char calibrationComplete;
    volatile long towerLocationTicks;
    long towerLocationMm;
    long towerMoveTargetTicks;
    int towerSpeed;
    int towerCurrentSpeed;

    volatile char switchMask;

    char calibrateState;
    volatile char loopMode;

} MOTOR;

#define ARMS_DEFINED 4

typedef struct ARMDEF  {
  char active;
  char upper;
  int stage0Motor;
  int stage1Motor;
  int highestPositionMm;
  int lowestPositionMm;
  int towerLocationMm;
} ARMDEF;

ARMDEF arms[ARMS_DEFINED] =
{
    { 1, 1, 0, 1, 1900, 340, 0},
    { 1, 1, 2, 3, 1900, 340, 0}
};

#define MOTORS_DEFINED 4

MOTOR motors[MOTORS_DEFINED] = {
     {0, 0, MT0S0_ST_EN,  MT0S0_ST_DIR,  MT0S0_ST_PL,  MT0S0_POS,  MT0S0_RESERVED,  MT0S0_BOTTOM_LIMIT_SWITCH,  1, 1, 100, 25.0,  25000, 850,  187},
     {1, -1, MT0S10_ST_EN, MT0S10_ST_DIR, MT0S10_ST_PL, MT0S10_POS, MT0S10_RESERVED, MT0S10_BOTTOM_LIMIT_SWITCH, 0, 1, 100, 8.0,   7400,  1000, 153},
     {2, 1, MT1S0_ST_EN,  MT1S0_ST_DIR,  MT1S0_ST_PL,  MT1S0_POS,  MT1S0_RESERVED,  MT1S0_BOTTOM_LIMIT_SWITCH,  1, 1, 100, 25.0,  25000, 850,  187},
     {3, -1, MT1S10_ST_EN, MT1S10_ST_DIR, MT1S10_ST_PL, MT1S10_POS, MT1S10_RESERVED, MT1S10_BOTTOM_LIMIT_SWITCH, 1, 1, 100, 8.0,   7500,  1000, 153}
};

#define INTERVAL_SLOTS 8

char towerIntervalTable[MOTORS_DEFINED][INTERVAL_SLOTS] = {
 { 1, 1, 1, 1, 1, 1, 1, 1 },
 { 2, 2, 2, 2, 1, 1, 1, 1 },
 { 1, 1, 1, 1, 1, 1, 1, 1 },
 { 2, 2, 2, 2, 1, 1, 1, 1 },
};

unsigned int switchTest = 0;
unsigned int switchTestLast = 0;

volatile unsigned int tickA = 0;
volatile byte flag = 0;
unsigned long lastTime;
int airValveOn = 0;

#define CALIBRATE_IDLE            0
#define CALIBRATE_GOING_DOWN      1

#define TOP_SWITCH_TRIGGER 1
#define BOT_SWITCH_TRIGGER 2

// Motor states
#define MOTOR_IDLE  0
#define MOTOR_CCW  1
#define MOTOR_CW 2

int speedLimiter = 0;
int speedLimit = 0;

// PUMP Control variables
char pumpAuto = 0;
char pumpOn = 0;
int pumpOnThreshold = 50 * PSI_TICKS;
int pumpOffThreshold = 100 * PSI_TICKS;

int pumpSampleInterval = 1000;
unsigned long pumpSampleTime = 0;

int fakeMode = 0;
int lastLeftTower = 1000;
int lastRightTower = 1000;

// Debug flags set in ISR
volatile char debugFlagMotorStopped[MOTORS_DEFINED] = {0};
volatile char debugFlagSwitchDetected[MOTORS_DEFINED] = {0};

////////////////////////////////////////////////////

// command line values
#define SENSOR   'v'
#define PUMP     'k'
#define RESET    'r'
#define PUMP_MON 'm'
#define AIR_ON   'o'
#define LIFT     'y'
#define LOOP     'l'
#define QUICK    'q'
#define STAGES   'u'
#define TOWER    't'
#define HALT     'h'

#define CALIBRATE 'c'
#define STATE    's'
#define ARM      'p'
#define SPEED    'd'
#define DBUG     'z'
#define FAKE_MODE 'f'
#define LIMIT    'i'

int debugMode = 0;

// command parsing variables
char debugPrint = 0;

int arg = 0;
int aindex = 0;

char chr;
char cmd;

char argv1[MAX_ARGV_LENGTH];
char argv2[MAX_ARGV_LENGTH];
char argv3[MAX_ARGV_LENGTH];
char argv4[MAX_ARGV_LENGTH];

char *px;
unsigned long arg1;
long arg2;
long arg3;
long arg4;

///////////////////////////////////////////////////////////

void enableMotor(MOTOR *mp, bool enable) {
  // Enable logic: LOW = enabled, HIGH = disabled
  digitalWrite(mp->enGpio, enable ? LOW : HIGH);
}

void startMotor(MOTOR *mp, char cmd, long ticks) {

  if (debugMode) {
    Serial.print(F("sm "));
    Serial.print(mp->motorid);
    Serial.print(F(": "));
    Serial.print(cmd);
    Serial.print(F(" "));
    Serial.println(ticks);
  }

  if (cmd == 'u') {
    mp->towerMoveTargetTicks = mp->towerLocationTicks + ticks;
    mp->towerDirection = TOWER_UP;
    mp->switchMask = TOP_SWITCH_TRIGGER;
    if (mp->inverseDirection)
      digitalWrite(mp->dirGpio, LOW);
    else
      digitalWrite(mp->dirGpio, HIGH);
  }
  else if (cmd == 'd') {
    mp->towerMoveTargetTicks = mp->towerLocationTicks - ticks;
    mp->towerDirection = TOWER_DOWN;
    mp->switchMask = BOT_SWITCH_TRIGGER;
    if (mp->inverseDirection)
      digitalWrite(mp->dirGpio, HIGH);
    else
      digitalWrite(mp->dirGpio, LOW);
  }
  else
    return;

  mp->towerMode = TOWER_ACC;
  mp->towerTickCount = 0;
  mp->towerIntervalTableIndex = 0;
  mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];

  mp->towerSteadyTrigger = min(ticks / INTERVAL_SLOTS, ACCELERATION_TICK_THRESHOLD);
  mp->towerDecTrigger = ticks - mp->towerSteadyTrigger;
  mp->towerIntervalChangeIncrement = mp->towerSteadyTrigger / mp->towerBuckets;
  mp->towerIntervalChangeTrigger = mp->towerIntervalChangeIncrement;

  enableMotor(mp, true);
}

int moveToMmLocation(MOTOR *mp, int loc) {
  long t;

  if (debugMode) {
    Serial.print(F("*mml "));
    Serial.print(mp->motorid);
    Serial.print(F(": "));
    Serial.print(loc);
    Serial.print(F(" "));
    Serial.print(mp->towerLocationMm);
    Serial.print(F(" "));
    Serial.println(mp->ticksPerMm);
  }

  if ((loc <= mp->towerUpperLimitMm) && (loc >= mp->towerLowerLimitMm)) {
    t = loc - mp->towerLocationMm;
    t *= mp->ticksPerMm;
    mp->towerLocationMm = loc;

    if (debugMode) {
      Serial.print(F("*  ticks: "));
      Serial.println(t);
    }

    if (t < 0) {
      startMotor(mp, 'd', -t);
    } else {
      startMotor(mp, 'u', t);
    }
    return 0;
  } else {
    Serial.println(F("*mlm range error"));
    return 1;
  }
}

int runCommand() {

  int tank;
  MOTOR *mp;
  int t;
  ARMDEF *ap;
  MOTOR *st0M;
  MOTOR *st1M;
  char buffer[80];
  long pInMm0;
  long pInMm1;
  int moveAmtMm;

  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);

  switch (cmd) {

    case RESET:
      setup();
      Serial.println(F("*OK"));
      return 0;

    case DBUG:
      debugMode = arg1;
      Serial.println(F("*OK"));
      return 0;

    case FAKE_MODE:
      fakeMode = arg1;
      Serial.println(F("*OK"));
      return 0;

    case SENSOR:
      tank = analogRead(TANK_SENSOR);
      Serial.print(F("* "));
      Serial.print(tank);
      Serial.print(F(":"));
      Serial.print((tank * 10) / (PSI_TICKS * 10));
      Serial.println(F("*  OK"));
      return 0;

    case PUMP:
      if (argv1[0] == '1') {
        digitalWrite(PUMP_ON1, LOW);
        digitalWrite(PUMP_ON2, LOW);
      } else {
        digitalWrite(PUMP_ON1, HIGH);
        digitalWrite(PUMP_ON2, HIGH);
      }
      Serial.println(F("*OK"));
      return 0;

    case AIR_ON:
      if (argv1[0] == '1') {
        airValveOn = 1;
        digitalWrite(OUT_VALVE, LOW);
      } else {
        digitalWrite(OUT_VALVE, HIGH);
        airValveOn = 0;
      }
      Serial.println(F("*OK"));
      return 0;

    case SPEED:
      speedLimit = arg1;
      speedLimiter = arg;
      Serial.println(F("*OK"));
      break;

    case PUMP_MON:
      if (argv1[0] == '1') {
        pumpAuto = 1;
        pumpSampleTime = 0;
        pumpOn = 0;

        tank = analogRead(TANK_SENSOR);
        if (tank < pumpOffThreshold) {
          digitalWrite(PUMP_ON1, LOW);
          digitalWrite(PUMP_ON2, LOW);
          pumpOn = 1;
        }
      } else {
        pumpAuto = 0;
        pumpOn = 0;
        digitalWrite(PUMP_ON1, HIGH);
        digitalWrite(PUMP_ON2, HIGH);
      }
      Serial.println(F("*OK"));
      return 0;

    case LIMIT:
      Serial.println(F("*ERR: LIMIT command not implemented"));
      return 0;

    case STATE:
      if (fakeMode) {
        pInMm0 = lastLeftTower;
        pInMm1 = lastRightTower;
      } else {
        ap = &arms[0];
        pInMm0 = motors[ap->stage0Motor].towerLocationTicks / motors[ap->stage0Motor].ticksPerMm +
                 motors[ap->stage1Motor].towerLocationTicks / motors[ap->stage1Motor].ticksPerMm +
                 ap->lowestPositionMm;

        ap = &arms[1];
        pInMm1 = motors[ap->stage0Motor].towerLocationTicks / motors[ap->stage0Motor].ticksPerMm +
                 motors[ap->stage1Motor].towerLocationTicks / motors[ap->stage1Motor].ticksPerMm +
                 ap->lowestPositionMm;
      }

      t = (analogRead(TANK_SENSOR) * 10) / (PSI_TICKS * 10);
      sprintf(buffer, "%ld %ld %d %d %d %d", pInMm0, pInMm1, pumpOn, pumpAuto, airValveOn, t);
      Serial.println(buffer);
      return 0;

    case ARM:
      if (fakeMode) {
        lastLeftTower = arg1;
        lastRightTower = arg2;
        Serial.println(F("OK"));
        return 0;
      }

      for (int arm = 0; arm < 2; arm++) {
        int p = (arm == 0 ? arg1 : arg2);

        if (arm >= ARMS_DEFINED) {
          Serial.println(F("*ERR: Invalid arm index"));
          continue;
        }

        ap = &arms[arm];

        if (p == 0) {
          continue;
        }

        if ((p > ap->highestPositionMm) || (p < ap->lowestPositionMm)) {
          Serial.println(F("*ERR: Invalid Location"));
          continue;
        }

        st0M = &motors[ap->stage0Motor];
        st1M = &motors[ap->stage1Motor];

        moveAmtMm = p - ap->towerLocationMm;

        if (debugMode) {
          Serial.print(F("current loc, movetarget and amount mm: "));
          Serial.print(ap->towerLocationMm);
          Serial.print(F(" "));
          Serial.print(p);
          Serial.print(F(" "));
          Serial.println(moveAmtMm);
        }

        if (moveAmtMm > 0) {
          if ((moveAmtMm + st1M->towerLocationMm) < st1M->towerUpperLimitMm) {
            if (debugMode)
              Serial.println(F("*move only s1 up"));
            moveToMmLocation(st1M, st1M->towerLocationMm + moveAmtMm);
          } else {
            if ((moveAmtMm + st0M->towerLocationMm) < st0M->towerUpperLimitMm) {
              if (debugMode)
                Serial.println(F("*move only s0 up"));
              moveToMmLocation(st0M, st0M->towerLocationMm + moveAmtMm);
            } else {
              int t = st1M->towerLocationMm + moveAmtMm - ((st0M->towerUpperLimitMm - st0M->towerLocationMm));

              if (debugMode) {
                Serial.print(F("*move both stages up "));
                Serial.print(st0M->towerUpperLimitMm);
                Serial.print(F(" "));
                Serial.println(t);
              }
              moveToMmLocation(st0M, st0M->towerUpperLimitMm);
              moveToMmLocation(st1M, t);
            }
          }
        } else {
          if ((st1M->towerLocationMm + moveAmtMm) > st1M->towerLowerLimitMm) {
            if (debugMode) {
              Serial.print(F("*move only s1 down: "));
              Serial.println(st1M->towerLocationMm);
            }
            moveToMmLocation(st1M, st1M->towerLocationMm + moveAmtMm);
          } else {
            if ((moveAmtMm + st0M->towerLocationMm) > st0M->towerLowerLimitMm) {
              if (debugMode)
                Serial.println(F("*move only s0 down"));
              moveToMmLocation(st0M, st0M->towerLocationMm + moveAmtMm);
            } else {
              if (debugMode)
                Serial.println(F("*move both stages down"));
              moveToMmLocation(st1M, st1M->towerLocationMm + moveAmtMm - ((st0M->towerLocationMm - st0M->towerLowerLimitMm)));
              moveToMmLocation(st0M, st0M->towerLowerLimitMm);
            }
          }
        }

        ap->towerLocationMm = p;
      }
      Serial.println(F("*OK"));
      return 0;

    case LIFT:
      Serial.println(F("*OK"));
      break;

    case LOOP:
      for (int i = 0; i < MOTORS_DEFINED; i++) {
        mp = &motors[i];
        mp->towerLocationTicks = CALIBRATION_LARGE_MOVE;
        mp->loopMode = LOOP_MODE_UP;
        mp->switchMask = 0;
        startMotor(mp, 'u', 1000);
        if (debugMode) {
          Serial.print(F("*loop start: "));
          Serial.println(i);
        }
      }
      Serial.println(F("*OK"));
      break;

    case HALT:
      for (int i = 0; i < MOTORS_DEFINED; i++) {
        mp = &motors[i];
        mp->towerDirection = TOWER_STOPPED;
        if (debugMode) {
          Serial.print(F("left "));
          Serial.print(i);
          Serial.print(F(":"));
          Serial.print(mp->towerLocationTicks);
          Serial.print(F(" "));
          Serial.println(mp->towerMoveTargetTicks);
        }
      }
      Serial.println(F("*OK"));
      break;

    case CALIBRATE:
      if (fakeMode) {
        Serial.println(F("OK"));
        return 0;
      }

      for (int i = 0; i < MOTORS_DEFINED; i++) {
        mp = &motors[i];
        mp->towerLocationTicks = CALIBRATION_LARGE_MOVE;
        mp->calibrateState = CALIBRATE_GOING_DOWN;
        mp->switchMask = BOT_SWITCH_TRIGGER;
        mp->calibrationComplete = false;
        startMotor(mp, 'd', DEFAULT_LARGE_TICK_COUNT);
        if (debugMode) {
          Serial.print(F("*cal start: "));
          Serial.println(i);
        }
      }
      break;

    case STAGES:
      if (arg1 >= MOTORS_DEFINED) {
        Serial.println(F("*ERR: Invalid motor ID"));
        return 0;
      }

      mp = &motors[arg1];
      mp->calibrateState = CALIBRATE_IDLE;

      switch (argv2[0]) {

        case 'l':
          Serial.print(F("*TLT "));
          Serial.print(mp->towerLocationTicks);
          Serial.print(F(" TLM "));
          Serial.print(mp->towerLocationMm);
          Serial.print(F("  HLT: "));
          Serial.print(mp->towerUpperLimitTicks);
          Serial.print(F("  HLM: "));
          Serial.print(mp->towerUpperLimitMm);
          Serial.print(F("  LLM: "));
          Serial.print(mp->towerLowerLimitMm);
          Serial.print(F("  ITT: "));
          Serial.println(mp->ticksPerMm);
          return 0;

        case 'h':
          mp->towerDirection = TOWER_STOPPED;
          enableMotor(mp, false);
          Serial.println(F("*OK"));
          break;

        case 'u':
          startMotor(mp, 'u', DEFAULT_LARGE_TICK_COUNT);
          Serial.println(F("*OK"));
          break;

        case 'd':
          mp->calibrateState = CALIBRATE_IDLE;
          mp->switchMask = BOT_SWITCH_TRIGGER;
          startMotor(mp, 'd', DEFAULT_LARGE_TICK_COUNT);
          Serial.println(F("*OK"));
          break;

        case 's':
          mp->towerCurrentSpeed = arg3;
          mp->towerSpeed = arg3;
          break;

        case 'i':
          mp->ticksPerMm = arg3;
          break;

        case 'c':
          mp->towerLocationTicks = CALIBRATION_LARGE_MOVE;
          mp->calibrateState = CALIBRATE_GOING_DOWN;
          mp->switchMask = BOT_SWITCH_TRIGGER;
          startMotor(mp, 'd', DEFAULT_LARGE_TICK_COUNT);
          Serial.println(F("*cal-start"));
          break;

        case 'm':
          mp->loopMode = LOOP_MODE_IDLE;
          if (mp->calibrationComplete) {
            if (moveToMmLocation(mp, arg3) != 0) {
              Serial.println(F("*ERR: Invalid Location"));
              return 0;
            }
          } else {
            Serial.println(F("*ERR: Calibration Incomplete"));
            return 0;
          }
          Serial.println(F("*OK"));
          break;

        case 'n':
          mp->loopMode = LOOP_MODE_IDLE;
          startMotor(mp, argv3[0], arg4);
          Serial.println(F("*OK"));
          break;

        case 'b':
          startMotor(&motors[0], argv3[0], arg4);
          startMotor(&motors[1], argv3[0], arg4);
          Serial.println(F("*OK"));
          break;

        case '>':
          enableMotor(mp, true);
          return 0;

        case '<':
          enableMotor(mp, false);
          return 0;

        case ']':
          digitalWrite(mp->dirGpio, LOW);
          return 0;

        case '[':
          digitalWrite(mp->dirGpio, HIGH);
          return 0;

        case 'x':
          switchTestLast = 0;
          if (arg3 == 1)
            switchTest = 1;
          else
            switchTest = 0;
          Serial.println(F("*OK"));
          break;

        default:
          Serial.println(F("*ERR-bad command"));
          break;
      }
      break;

    default:
      Serial.println(F("*ERR: Invalid cmd"));
      break;
  }
  return 0;
}

void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg = 0;
  aindex = 0;
}

int towerCalcInterval(MOTOR *mp) {
  if (mp->towerIntervalCount != 1) {
    mp->towerIntervalCount--;
    return 0;
  }

  switch (mp->towerMode) {

    case TOWER_ACC:
      if (mp->towerTickCount >= mp->towerSteadyTrigger) {
        mp->towerTickCount++;
        mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerBuckets - 1];
        mp->towerMode = TOWER_STEADY;
        return 1;
      } else {
        if (mp->towerTickCount >= mp->towerIntervalChangeTrigger) {
          if (mp->towerIntervalTableIndex != mp->towerBuckets - 1)
            mp->towerIntervalTableIndex++;

          mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
          mp->towerIntervalChangeTrigger = mp->towerTickCount + mp->towerIntervalChangeIncrement;
          mp->towerTickCount++;
          return 1;
        } else {
          mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
          mp->towerTickCount++;
          return 1;
        }
      }
      break;

    case TOWER_DEC:
      if (mp->towerTickCount >= mp->towerIntervalChangeTrigger) {
        if (mp->towerIntervalTableIndex != 0)
          mp->towerIntervalTableIndex--;

        mp->towerTickCount++;
        mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
        mp->towerIntervalChangeTrigger = mp->towerTickCount + mp->towerIntervalChangeIncrement;
        return 1;
      } else {
        mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
        mp->towerTickCount++;
        return 1;
      }
      break;

    case TOWER_STEADY:
      if (mp->towerTickCount < mp->towerDecTrigger) {
        mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerBuckets - 1];
        mp->towerTickCount++;
        return 1;
      }

      mp->towerMode = TOWER_DEC;
      mp->towerIntervalTableIndex = mp->towerBuckets - 1;
      mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
      mp->towerIntervalChangeTrigger = mp->towerTickCount + mp->towerIntervalChangeIncrement;
      mp->towerTickCount++;
      return 1;
  }
  return 0;
}

void TimerHandler() {
  MOTOR *mp;
  int i;

  if (speedLimiter != 0) {
    speedLimiter--;
    return;
  } else {
    speedLimiter = speedLimit;
  }

  for (i = 0; i < MOTORS_DEFINED; i++) {
    mp = &motors[i];

    if ((mp->towerDirection == TOWER_STOPPED) || (mp->switchMask == 0))
      continue;

    // Check limit switches
    if (digitalRead(mp->limit1Gpio) == 1) {
      if (mp->switchMask == BOT_SWITCH_TRIGGER) {
        mp->switchMask = 0;
        mp->towerDirection = TOWER_STOPPED_SWITCH;
        enableMotor(mp, true);
        debugFlagSwitchDetected[i] = 1;
        continue;
      }
    }

    mp->towerSpeed = mp->towerCurrentSpeed;

    switch (mp->towerDirection) {

      case TOWER_STOPPED:
        break;

      case TOWER_UP:
        if (mp->towerLocationTicks >= mp->towerMoveTargetTicks) {
          mp->towerDirection = TOWER_STOPPED;
          if (!mp->leaveEnabled)
            enableMotor(mp, false);
          debugFlagMotorStopped[i] = 1;
        } else {
          if (towerCalcInterval(mp)) {
            mp->towerLocationTicks++;
            digitalWrite(mp->stepGpio, HIGH);
            delayMicroseconds(mp->stepDelayTime);
            digitalWrite(mp->stepGpio, LOW);
          }
        }
        break;

      case TOWER_DOWN:
        if (mp->towerLocationTicks <= mp->towerMoveTargetTicks) {
          mp->towerDirection = TOWER_STOPPED;
          if (!mp->leaveEnabled)
            enableMotor(mp, false);
          debugFlagMotorStopped[i] = 2;
        } else {
          if (towerCalcInterval(mp)) {
            mp->towerLocationTicks--;
            digitalWrite(mp->stepGpio, HIGH);
            delayMicroseconds(mp->stepDelayTime);
            digitalWrite(mp->stepGpio, LOW);
          }
        }
        break;
    }
  }
}

void setup() {

  Serial.begin(38400);
  pumpAuto = 0;
  pumpOn = 0;

  pinMode(PUMP_ON1, OUTPUT);
  digitalWrite(PUMP_ON1, HIGH);
  pinMode(PUMP_ON2, OUTPUT);
  digitalWrite(PUMP_ON2, HIGH);

  pinMode(OUT_VALVE, OUTPUT);
  digitalWrite(OUT_VALVE, HIGH);

  pinMode(K1, INPUT);

  pinMode(OUT_VALVE, OUTPUT);
  digitalWrite(OUT_VALVE, HIGH);

  airValveOn = 0;
  switchTest = 0;

  for (int i = 0; i < MOTORS_DEFINED; i++) {
    pinMode(motors[i].stepGpio, OUTPUT);
    pinMode(motors[i].enGpio, OUTPUT);
    pinMode(motors[i].dirGpio, OUTPUT);
    pinMode(motors[i].limit1Gpio, INPUT_PULLUP);

    digitalWrite(motors[i].stepGpio, HIGH);
    enableMotor(&motors[i], false);

    motors[i].towerLocationTicks = UNKNOWN_POSITION_TICKS;
    motors[i].towerDirection = TOWER_STOPPED;
    motors[i].switchMask = 0;
    motors[i].calibrateState = CALIBRATE_IDLE;
    motors[i].towerBuckets = INTERVAL_SLOTS;
    motors[i].calibrationComplete = false;
    motors[i].loopMode = LOOP_MODE_IDLE;
  }

  speedLimiter = 0;
  speedLimit = 0;

  ITimer1.init();

  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler)) {
    Serial.print(F("*Starting  ITimer OK, millis() = "));
    Serial.println(millis());
  } else {
    Serial.println(F("*Can't set ITimer. Select another freq. or timer"));
  }

  speedLimiter = 0;
  speedLimit = 0;
  debugMode = 0;

  Serial.print(F("*Tower Setup Done "));
  Serial.print(F(__DATE__));
  Serial.print(F(" at "));
  Serial.println(F(__TIME__));
}

void loop() {

  int tank;
  unsigned int temp;
  MOTOR *mp;

  // Process debug flags from ISR
  for (int i = 0; i < MOTORS_DEFINED; i++) {
    if (debugFlagSwitchDetected[i]) {
      if (debugMode) {
        Serial.print(F("*Switch detection: "));
        Serial.print(i);
        Serial.print(F(" "));
        Serial.println(digitalRead(motors[i].limit1Gpio));
      }
      debugFlagSwitchDetected[i] = 0;
    }

    if (debugFlagMotorStopped[i]) {
      if (debugMode) {
        if (debugFlagMotorStopped[i] == 1) {
          Serial.print(F("*stoppedu: "));
        } else {
          Serial.print(F("*stoppedd: "));
        }
        Serial.print(i);
        Serial.print(F(" "));
        Serial.print(motors[i].towerLocationTicks);
        Serial.print(F(" "));
        Serial.println(motors[i].towerMoveTargetTicks);
      }
      debugFlagMotorStopped[i] = 0;
    }
  }

  // Handle serial commands with buffer overflow protection
  while (Serial.available() > 0) {

    chr = Serial.read();

    if (chr == 13) {
      if (arg == 1) argv1[aindex] = 0;
      else if (arg == 2) argv2[aindex] = 0;
      else if (arg == 3) argv3[aindex] = 0;
      else if (arg == 4) argv4[aindex] = 0;

      runCommand();
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[aindex] = 0;
        arg = 2;
        aindex = 0;
      } else if (arg == 2) {
        argv2[aindex] = 0;
        arg = 3;
        aindex = 0;
      } else if (arg == 3) {
        argv3[aindex] = 0;
        arg = 4;
        aindex = 0;
      } else if (arg == 4) {
        argv4[aindex] = 0;
        arg = 5;
        aindex = 0;
      }
      continue;
    } else {
      if (arg == 0) {
        cmd = chr;
      } else if (arg == 1 && aindex < MAX_ARGV_LENGTH - 1) {
        argv1[aindex] = chr;
        aindex++;
      } else if (arg == 2 && aindex < MAX_ARGV_LENGTH - 1) {
        argv2[aindex] = chr;
        aindex++;
      } else if (arg == 3 && aindex < MAX_ARGV_LENGTH - 1) {
        argv3[aindex] = chr;
        aindex++;
      } else if (arg == 4 && aindex < MAX_ARGV_LENGTH - 1) {
        argv4[aindex] = chr;
        aindex++;
      }
    }
  }

  // Handle calibration state machines
  for (int i = 0; i < MOTORS_DEFINED; i++) {
    mp = &motors[i];

    if (mp->calibrateState != CALIBRATE_IDLE) {
      if (mp->towerDirection == TOWER_STOPPED_SWITCH) {
        mp->towerLocationTicks = 0;
        mp->towerLocationMm = mp->towerLowerLimitMm;

        mp->calibrateState = CALIBRATE_IDLE;
        if (debugMode) {
          Serial.print(F("*Cal Done: "));
          Serial.println(i);
        }
        mp->calibrationComplete = true;

        if (mp->armid >= 0) {
          arms[mp->armid].towerLocationMm = arms[mp->armid].lowestPositionMm;
        }

        if (motors[0].calibrationComplete && motors[1].calibrationComplete &&
            motors[2].calibrationComplete && motors[3].calibrationComplete) {
          Serial.println(F(" OK- Cal Done"));
        }
      }
    }

    if (mp->towerDirection <= TOWER_STOPPED_SWITCH) {
      switch (mp->loopMode) {
        int t;

        case LOOP_MODE_DOWN:
          mp->loopMode = LOOP_MODE_UP;
          mp->switchMask = 0;
          t = random(1000, mp->towerUpperLimitTicks * 0.75) & 0xFFFF;
          startMotor(mp, 'u', t);
          if (debugMode) {
            Serial.print(F("**Going up: "));
            Serial.print(mp->motorid);
            Serial.print(F(": "));
            Serial.println(t);
          }
          break;

        case LOOP_MODE_UP:
          mp->loopMode = LOOP_MODE_DOWN;
          mp->switchMask = BOT_SWITCH_TRIGGER;
          t = 0;
          startMotor(mp, 'd', DEFAULT_LARGE_TICK_COUNT);
          if (debugMode) {
            Serial.print(F("**Going down: "));
            Serial.print(mp->motorid);
            Serial.print(F(": "));
            Serial.println(t);
          }
          break;

        case LOOP_MODE_IDLE:
          break;
      }
    }
  }

  if (switchTest) {
    temp = 0;
    for (int i = 0; i < MOTORS_DEFINED; i++) {
      temp |= (digitalRead(motors[i].limit1Gpio) << i);
    }

    if (temp != switchTestLast) {
      Serial.println(temp, HEX);
      switchTestLast = temp;
    }
  }

  if (pumpAuto) {
    if (pumpSampleTime < millis()) {
      tank = analogRead(TANK_SENSOR);

      if (pumpOn) {
        if (tank > pumpOffThreshold) {
          pumpOn = 0;
          digitalWrite(PUMP_ON1, HIGH);
          digitalWrite(PUMP_ON2, HIGH);
        }
      } else {
        if (tank < pumpOnThreshold) {
          pumpOn = 1;
          digitalWrite(PUMP_ON1, LOW);
          digitalWrite(PUMP_ON2, LOW);
        }
      }

      pumpSampleTime = millis() + pumpSampleInterval;
    }
  }
}
