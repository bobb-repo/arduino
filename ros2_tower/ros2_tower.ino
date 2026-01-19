////////////////////////////////////////////////////
// 
// VACUUM/TOWER CONTROL
//
////////////////////////////////////////////////////

//#include <Wire.h>
//#include <math.h>
//#include <stdio.h>
//#include <LibPrintf.h>


#define USE_TIMER_1 true
#define USE_TIMER_2 false
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 false

#include "TimerInterrupt.h"

// GPIO ASSIGNMENTS


// Tower0 motor pin definitions

// 
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
    float  ticksPerMim  ;
  
    int  towerUpperLimitTicks ;    // upper limit of movement in ticks
    int  towerUpperLimitMim;   // height of  stage
    int  towerLowerLimitMim;    // unreachable lower part of stage            

    int towerMode  ; //= TOWER_ACC;
    long towerSteadyTrigger ;
    long towerDecTrigger ;
    long towerTickCount ;
    int towerIntervalCount ;
    int towerIntervalTableIndex ;
    char towerBuckets ; //= sizeof(towerIntervalTable) / sizeof(towerIntervalTable[0]);
    long towerIntervalChangeTrigger ;
    int towerIntervalChangeIncrement ;

    char towerDirection   ;
    char calibrationComplete;
    long towerLocationTicks;
    long towerLocationMim ;
    long towerMoveTargetTicks;
    int towerSpeed ;
    int towerCurrentSpeed ;
    
    char switchMask;

    char calibrateState;
    char loopMode;
   
  } MOTOR;
 
#define ARMS_DEFINED 4

typedef struct ARMDEF  {
  char active;
  char upper;
  int stage0Motor;
  int stage1Motor;
  int highestPositionMim;
  int lowestPositionMim;
  int towerLocationMim;
} ARMDEF;

ARMDEF arms[ARMS_DEFINED] =
{
    { 1,1,0,1,1900, 340,0},
    { 1,1,2,3,1900, 340,0}
};


#define MOTORS_DEFINED 4

MOTOR motors[MOTORS_DEFINED] = { //                                                                             ticks/mm, upptick, uppmim, lowermm,base
     {0, 0,MT0S0_ST_EN, MT0S0_ST_DIR, MT0S0_ST_PL, MT0S0_POS, MT0S0_RESERVED, MT0S0_BOTTOM_LIMIT_SWITCH, 1, 1, 100, 25.0,  25000,  850,  187},
     {1,-1,MT0S10_ST_EN,MT0S10_ST_DIR,MT0S10_ST_PL,MT0S10_POS,MT0S10_RESERVED,MT0S10_BOTTOM_LIMIT_SWITCH,0, 1, 100, 8.0,   7400,   1000,  153},
     {2, 1,MT1S0_ST_EN, MT1S0_ST_DIR, MT1S0_ST_PL, MT1S0_POS, MT1S0_RESERVED, MT1S0_BOTTOM_LIMIT_SWITCH, 1, 1, 100, 25.0,  25000,   850,  187},
     {3,-1,MT1S10_ST_EN,MT1S10_ST_DIR,MT1S10_ST_PL,MT1S10_POS,MT1S10_RESERVED,MT1S10_BOTTOM_LIMIT_SWITCH,1, 1, 100, 8.0,   7500,   1000,  153}
}; 

#define INTERVAL_SLOTS 8


char towerIntervalTable[MOTORS_DEFINED][INTERVAL_SLOTS] = {
 { 1, 1, 1, 1, 1, 
 1, 1, 1 },
 { 2, 2, 2, 2, 1, 1, 1, 1 },
 { 1, 1, 1, 1, 1, 1, 1, 1 },
 { 2, 1, 1, 1, 1, 1, 1, 1 },
};
 
                                ; 
//IntervalTimer myTimer;
unsigned int switchTest,switchTestLast;

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

// tanks  100psi = 511


int  speedLimiter = 0;
int  speedLimit = 0;

// PUMP Control variables

#define PSI_TICKS 7.1f

char pumpAuto = 0;
char pumpOn = 0;
int pumpOnThreshold = 50*PSI_TICKS;   // above this turn pump on  ~50 psi  - 150 psi sensor
int pumpOffThreshold = 100 * PSI_TICKS;  // below this turn pump off ~100 psi

int pumpSampleInterval = 1000;
unsigned long pumpSampleTime = 0;

int fakeMode = 0;
int lastLeftTower = 1000;
int lastRightTower = 1000;

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

// tower commands format: t towernumber cmd data

//////////////

int debugMode = 0;

// command parsing variables

char debugPrint = 0;

// A pair of varibles to help parse serial commands (thanks Fergs)

int arg = 0;
int aindex = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[8];
char argv2[8];
char argv3[8];
char argv4[8];

// The arguments h to integers
char *px;
unsigned long arg1;
long arg2;
long arg3;
long arg4;

///////////////////////////////////////////////////////////


void startMotor(MOTOR * mp,char cmd, long ticks){

  if (debugMode)
    Serial.print( "sm " + String(mp->motorid) + ": " + String(cmd) + " " + String(ticks));


  if (cmd == 'u')
  { 
            mp->towerMoveTargetTicks = mp->towerLocationTicks + ticks;
            mp->towerDirection = TOWER_UP;
            mp->switchMask = TOP_SWITCH_TRIGGER;
            if (mp->inverseDirection)
              digitalWrite(mp->dirGpio, LOW);
            else
              digitalWrite(mp->dirGpio, HIGH);

  }
  else if (cmd == 'd')
  {
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
           
  mp->towerSteadyTrigger = min(ticks / sizeof(towerIntervalTable), 200);
  mp->towerDecTrigger = ticks - mp->towerSteadyTrigger;
  mp->towerIntervalChangeIncrement = mp->towerSteadyTrigger / mp->towerBuckets;
  mp->towerIntervalChangeTrigger = mp->towerIntervalChangeIncrement;
  
  digitalWrite(mp->enGpio, LOW);
}

int moveToMimLocation(MOTOR * mp, int loc)
{
  long t;
  
  if (debugMode)
    Serial.println("*mml " + String(mp->motorid) + ": " +  String(loc) + " " + String(mp->towerLocationMim) +" " + String(mp->ticksPerMim));
  
  // check if location is okay for the arm. note that the location is relative to each stage and is not absolute. 

    if ((loc <= mp->towerUpperLimitMim)  && (loc >= mp->towerLowerLimitMim)  )
  {
            // its a legal location, see how far to move

            t = loc - mp->towerLocationMim;

           // mp->towerLocationMim + t;

            t *= mp->ticksPerMim;
            
            mp->towerLocationMim = loc;

            if (debugMode)
              Serial.println("*  ticks: " + String(t));

            if (t < 0) {
              startMotor(mp,'d',-t);
            } else {
              startMotor(mp,'u',t);
            }
            return 0;
  } else {
    Serial.println("*mlm range error");
    return 1;
  }  
}

int runCommand() {

  int tank;
  MOTOR * mp;
  int t;
  ARMDEF *ap;
  MOTOR * st0M;
  MOTOR * st1M;
  char buffer[80];
  long pInMm0;
  long pInMm1;


  int moveAmtMim;

  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);

  //Serial.println(cmd);
  //Serial.println (arg1);
  //Serial.println (argv2);

  switch (cmd) {

    case RESET:
      setup();
      Serial.println("*OK");
      return 0;

    case DBUG:
      debugMode = arg1;
      Serial.println("*OK");
      return 0;
    
    case FAKE_MODE:
      fakeMode = arg1;
      Serial.println("*OK");
      return 0;
    
        case SENSOR:
        tank = analogRead(TANK_SENSOR);
        Serial.print("* ");
        Serial.print(tank);
        Serial.print(":");
        Serial.print((tank * 10) / (PSI_TICKS * 10));
        Serial.println("*  OK");
        return 0;

    case PUMP:
    if (argv1[0] == '1')
      {
        digitalWrite(PUMP_ON1, LOW);
        digitalWrite(PUMP_ON2, LOW);
      } else {
        digitalWrite(PUMP_ON1, HIGH);
        digitalWrite(PUMP_ON2, HIGH);
      }
      Serial.println("*OK");
      return 0;

     case AIR_ON:
       if (argv1[0] == '1') {
        airValveOn = 1;
        digitalWrite(OUT_VALVE, LOW);
      } else {
        digitalWrite(OUT_VALVE, HIGH);
        airValveOn = 0;
      }
      Serial.println("*OK");
      return 0;

 case SPEED:
     speedLimit = arg1;
     speedLimiter = arg;
      Serial.println("*OK");
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
      Serial.println("*OK");
      return 0;

    case LIMIT:
    
    
    case STATE:
    
        if (fakeMode)
        {
          pInMm0 = lastLeftTower;
          pInMm1 = lastRightTower;
        }
        else
        {
            
          ap = &arms[0];
          pInMm0 = motors[ap->stage0Motor].towerLocationTicks/motors[ap->stage0Motor].ticksPerMim +  motors[ap->stage1Motor].towerLocationTicks/motors[ap->stage1Motor].ticksPerMim + ap->lowestPositionMim;
  
          ap = &arms[1];
          pInMm1 = motors[ap->stage0Motor].towerLocationTicks/motors[ap->stage0Motor].ticksPerMim +  motors[ap->stage1Motor].towerLocationTicks/motors[ap->stage1Motor].ticksPerMim + ap->lowestPositionMim;
        }

        t =  (analogRead(TANK_SENSOR) *10)/(PSI_TICKS * 10);
           
        //sprintf(buffer, "%ld %ld ---%ld %ld", pInMm0, pInMm1,motors[ap->stage0Motor].towerLocationTicks,motors[ap->stage1Motor].towerLocationTicks );
        // tower left, right, motor on, auto_mode, airvalve, tank pressure
        
        sprintf(buffer, "%ld %ld %d %d %d %d ", pInMm0, pInMm1,pumpOn,pumpAuto,airValveOn , t + random (-3,5 ));
        Serial.println(buffer);
        return 0;
 

    case ARM:
      if (fakeMode)
      {
       lastLeftTower = arg1;
       lastRightTower = arg2;
        Serial.println("OK"); 
        return 0;
      }
      for (int arm = 0; arm < 2; arm++)
      {
        int p = (arm == 0 ? arg1 : arg2);
       
        ap = &arms[arm];

        // arg2 is the desired location in  mim. see if the target is in range

        if (p == 0)  // ros may send 0 if nothing else to do
        {
          continue;
        }
          
        if ((p > ap->highestPositionMim) || (p < ap->lowestPositionMim) )
        {
          Serial.println("*ERR: Invalid Location");  // too high/low
          continue;
        }

        // its a valid target, do we go up or down
        // for now we assume just one arm per tower, we can add the second arm stuff later.

        st0M = &motors[ap->stage0Motor];
        st1M = &motors[ap->stage1Motor];

        // we calculate the amount to move up or down. From here it is just relative movements
        
        moveAmtMim = p  - ap->towerLocationMim ; 
        
        if (debugMode)
          Serial.println ("current loc, movetarget and amount mim: " + String(ap->towerLocationMim) + " " +String(p) + " " + String(moveAmtMim));
        
        if (moveAmtMim > 0)
        {
          // going up, can we do it by moving only stage 1 
          
          if ((moveAmtMim + st1M->towerLocationMim) <  st1M->towerUpperLimitMim)
          {
              // stage 1 is enough, just move it
              if (debugMode)
                Serial.println("*move only s1 up");
              moveToMimLocation(st1M,st1M->towerLocationMim + moveAmtMim);
            }
          else
          {          
              // the move is too much for just stage 1 and so we need to move stage 0
              // can moving just stage 0 enough?

              if ((moveAmtMim + st0M->towerLocationMim) < st0M->towerUpperLimitMim )
              {
                // yes, just move s0 up

                if (debugMode)
                  Serial.println("*move only s0 up");
                moveToMimLocation(st0M,st0M->towerLocationMim + moveAmtMim);
              }
              else
              {
                // no, we need to move move both stage 0 and stage 1. move stage 0 to the top, then stage 1 for the remainder

                int t = st1M->towerLocationMim + moveAmtMim - ((st0M->towerUpperLimitMim - st0M->towerLocationMim));

                if (debugMode)
                  Serial.println("*move both stages up " + String(st0M->towerUpperLimitMim) + " " + String(t)); 
                moveToMimLocation(st0M, st0M->towerUpperLimitMim);
                moveToMimLocation(st1M, t);
              }
          }  
        }
        else
        {
          // going down, can we do it by moving only  stage 1 
          
          if (( st1M->towerLocationMim + moveAmtMim ) >  st1M->towerLowerLimitMim)
          {
              // stage 1 is enough, just move it down

              if (debugMode)
                Serial.println("*move only s1 down: " + String(st1M->towerLocationMim));
              moveToMimLocation(st1M,st1M->towerLocationMim + moveAmtMim);
          }
          else
          {          
              // the move is too much for just stage 1 and so we need to move stage 0
              // can moving just stage 0 enough?

              if ((moveAmtMim + st0M->towerLocationMim) > st0M->towerLowerLimitMim )
              {
                // yes, just move s0 down

                if (debugMode)
                  Serial.println("*move only s0 down");
                moveToMimLocation(st0M,st0M->towerLocationMim + moveAmtMim);
                //ap->towerLocationMim += moveAmtMim;
              }
              else
              {
                // no, we need to move move stage 0 and stage 1. move stage 0 to the bottom, then stage 1 for the remainder
              
                if (debugMode)
                  Serial.println("*move both stages down");
                moveToMimLocation(st1M,st1M->towerLocationMim + moveAmtMim - ((st0M->towerLocationMim - st0M->towerLowerLimitMim )));
                moveToMimLocation(st0M, st0M->towerLowerLimitMim);
            }
          }  
        }
        
        ap->towerLocationMim = p;     
      //  Serial.println("*new arm position: " + String(ap->towerLocationMim));
  }
      Serial.println("*OK");
      return 0;

  case LIFT:
  /*
      if (argv1[0] == 'u')   // up
      {
         digitalWrite(LIFT_MAIN1,LOW );
        digitalWrite(LIFT_P1,LOW);
         digitalWrite(LIFT_P2,LOW);   
      }
      else if (argv1[0] == 'd')   // down
      {
         digitalWrite(LIFT_MAIN1,LOW );
         digitalWrite(LIFT_P1,HIGH);
         digitalWrite(LIFT_P2,HIGH);             
      }
      else   //stop
      {
         digitalWrite(LIFT_MAIN1,HIGH );
         digitalWrite(LIFT_P1,HIGH);
         digitalWrite(LIFT_P2,HIGH);   
     }
  */
      Serial.println("*OK");
      break;
      
      case LOOP:
        for (int i = 0; i < MOTORS_DEFINED; i++)
        {
          mp = &motors[i];
          mp->towerLocationTicks = 150000;
          mp->loopMode = LOOP_MODE_UP;
          mp->switchMask = 0;
          startMotor(mp,'u',1000);
          Serial.println("*loop start: " + String(i));
        }       
        Serial.println("*OK");
        break;

      case HALT:
        for (int i = 0; i < MOTORS_DEFINED; i++)
        {
          mp = &motors[i];
          mp->towerDirection = TOWER_STOPPED;
          if (debugMode)
            Serial.println("left " + String(i) + ":"  + String(mp->towerLocationTicks) + " " + String(mp->towerMoveTargetTicks) );       
        }
        Serial.println("*OK");
        break;
      
      case CALIBRATE:
        if (fakeMode)
        {
          Serial.println("OK");  // only one tower for now
          return 0;
        }
      // calibrate all motors

        for (int i = 0; i < MOTORS_DEFINED; i++)
        {
          mp = &motors[i];
          mp->towerLocationTicks = 150000;
          mp->calibrateState = CALIBRATE_GOING_DOWN;
          mp->switchMask = BOT_SWITCH_TRIGGER;
          mp->calibrationComplete = false;
          startMotor(mp,'d',100000);
          if (debugMode)
            Serial.println("*cal start: " + String(i));
        } 
        //Serial.println("*OK");
        break;

      case STAGES:
      //Serial.println("*si");
      //Serial.println (arg1);
      //Serial.println (argv2);
     // Serial.println (argv3);
    //  Serial.println (argv4);

      if (arg1 > MOTORS_DEFINED)
      {
         Serial.println("*ERR");
         return 0;
      }

      mp = &motors[arg1];
      mp->calibrateState = CALIBRATE_IDLE;
      
      switch (argv2[0]) {

        case 'l':  // return current location
          Serial.print("*TLT ");
          Serial.print(mp->towerLocationTicks);
          Serial.print("TLM ");
          Serial.print(mp->towerLocationMim);
          Serial.print("  HLT: ");
          Serial.print(mp->towerUpperLimitTicks);
          Serial.print("  HLM: ");
          Serial.print(mp->towerUpperLimitMim);
          Serial.print("  LLM: ");
          Serial.print(mp->towerLowerLimitMim);
          Serial.print("  ITT: ");
          Serial.println(mp->ticksPerMim);
          return 0;

        case 'h':  // stop moving
          mp->towerDirection = TOWER_STOPPED;
          digitalWrite(mp->enGpio, HIGH);
          Serial.println("*OK");
          break;

        case 'u':  // start moving up
           startMotor(mp,'u',100000);
           Serial.println("*OK");
           break;

        case 'd':  // start moving down
          mp->calibrateState = CALIBRATE_IDLE;
          mp->switchMask = BOT_SWITCH_TRIGGER;
          startMotor(mp,'d',100000);
          Serial.println("*OK");
           break;

        case 's':
          mp->towerCurrentSpeed = arg3;
          mp->towerSpeed = arg3;
          break;

       case 'i':
          mp->ticksPerMim = arg3;
          break;
        
        case 'c':  // calibrate bottom
          mp->towerLocationTicks = 150000;
          mp->calibrateState = CALIBRATE_GOING_DOWN;
          mp->switchMask = BOT_SWITCH_TRIGGER;
          startMotor(mp,'d',100000);
          Serial.println("*cal-start");
          break;

        case 'm':  // move to mim location

           mp->loopMode = LOOP_MODE_IDLE;
           if (mp->calibrationComplete)
          {
             if (moveToMimLocation( mp, arg3) != 0)
             {
               Serial.println("*ERR: Invalid Location");
               return 0;
             }
          }
          else
          {
               Serial.println("*ERR: Calibration Incomplete");
               return 0;
          }         
          Serial.println("*OK");
          break;

        case 'n':  // move ticks up or 
          //Serial.print( "ticks ");
          //Serial.println( arg4);
           mp->loopMode = LOOP_MODE_IDLE;

           if (1) //mp->calibrationComplete)
            startMotor(mp,argv3[0],arg4);
          else
          {
            Serial.println("*ERR: Calibration Incomplete");
            return 0;
          }
         Serial.println("*OK");
           break;

        case 'b':  // move ticks up or down for 2 motorsd
          //Serial.print( "ticks ");
          //Serial.println( arg4);
          if (1) // mp->calibrationComplete)
          {
            startMotor(&motors[0],argv3[0],arg4);
            startMotor(&motors[1],argv3[0],arg4);
          }  
          else
          {
            Serial.println("*ERR: Calibration Incomplete");
            return 0;
          }
          Serial.println("*OK");
           break;

        case '>':
          digitalWrite(mp->enGpio, LOW);
          return 0;

        case '<':
          digitalWrite(mp->enGpio, HIGH);
          return 0;

        case ']':
          digitalWrite(mp->dirGpio, LOW);
          return 0;

        case '[':
          digitalWrite(mp->dirGpio, HIGH);
          return 0;

        case 'x':
          switchTestLast  =0;
//          Serial.println(arg3);

          if (arg3 == 1)
            switchTest = 1;
          else
            switchTest = 0;
          Serial.println("*OK");
          break;
         
       default:
          Serial.println("*ERR-bad command");
          break;
      }
      break;

    default:
       Serial.println("*ERR: Invalid cmd");
       break;
  }
    return 0;
}

/* Clear the currenLOWt command parameters */
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

int towerCalcInterval(MOTOR * mp) {
  // see if we should do anything

    if (mp->towerIntervalCount != 1) {  // we do nothing, exit with signal that says skip further processing
    mp->towerIntervalCount--;
    return 0;
  }
  // we will generate a tick and so calculate the next interval

  switch (mp->towerMode) {

    case TOWER_ACC:
      //Serial.println("*a");
      if (mp->towerTickCount >= mp->towerSteadyTrigger) {
        // time to transition to steady state motion using the shortest interval

        mp->towerTickCount++;
        mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerBuckets - 1];
       // Serial.println("*acc stddy");
        mp->towerMode = TOWER_STEADY;
        return 1;
      } else {
        // not at steady time, calc interval time

        if (mp->towerTickCount >= mp->towerIntervalChangeTrigger) {
          // time to move to next interval, which will be less than the current one

          if (mp->towerIntervalTableIndex != mp->towerBuckets - 1)
            mp->towerIntervalTableIndex++;

          mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
          mp->towerIntervalChangeTrigger = mp->towerTickCount + mp->towerIntervalChangeIncrement;
          mp->towerTickCount++;
         // Serial.print(mp->towerIntervalCount);
          //Serial.print(" acc inc ");
         // Serial.println(mp->towerIntervalCount);
          return 1;
        } 
        else
        {
          // use current interval
          mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
          mp->towerTickCount++;
          return 1;
        }
      }
      break;

    case TOWER_DEC:
     // Serial.println("*d");

      // see if time to slow down even more

      if (mp->towerTickCount >= mp->towerIntervalChangeTrigger) {
        // time to move to next interval, which will be more than the current one

        if (mp->towerIntervalTableIndex != 0)  // make sure we don't go below max interval
          mp->towerIntervalTableIndex--;

        mp->towerTickCount++;
        mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
        mp->towerIntervalChangeTrigger = mp->towerTickCount + mp->towerIntervalChangeIncrement;
        //Serial.print(" dec  ");
        //Serial.println(mp->towerIntervalCount);
        return 1;
      }
      else
      {
        // use current interval
        mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
        mp->towerTickCount++;
        return 1;
      }
      break;

    case TOWER_STEADY:
      //Serial.println("*s");
      if (mp->towerTickCount < mp->towerDecTrigger)
      {
       //Serial.println("*t");
      //
        // not yet time to dec, return steady inteval

        mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerBuckets - 1];
        mp->towerTickCount++;
        //Serial.print(mp->towerIntervalCount);
        //Serial.print(" stddy ");
        //Serial.println(mp->towerTickCount);
        return 1;
      }

      // time to start dec
      mp->towerMode = TOWER_DEC;
      mp->towerIntervalTableIndex = mp->towerBuckets - 1;
      mp->towerIntervalCount = towerIntervalTable[mp->motorid][mp->towerIntervalTableIndex];
      mp->towerIntervalChangeTrigger = mp->towerTickCount + mp->towerIntervalChangeIncrement;
      mp->towerTickCount++;
     // Serial.print(mp->towerIntervalCount);
     // Serial.print(" stddy dec ");
     //  Serial.println(mp->towerTickCount);
      return 1;
  }
  //Serial.println('x0');
  return 0;
}


void TimerHandler() {
  MOTOR * mp;
  int i;
  
   if (speedLimiter != 0)
  {
      speedLimiter --;
      return;
  }
  else
    speedLimiter = speedLimit;

  for ( i = 0; i < MOTORS_DEFINED; i ++)
  {
    mp = &motors[i];

   if ((mp->towerDirection == TOWER_STOPPED) || (mp->switchMask == 0) )
      continue;
      
    if (digitalRead(mp->limit1Gpio) == 1) // just check bottom switch
    {
      if (mp->switchMask == BOT_SWITCH_TRIGGER)
      {
        mp->switchMask = 0;
        mp->towerDirection = TOWER_STOPPED_SWITCH;
        digitalWrite(mp->enGpio, LOW);
        if (debugMode)
        {
          Serial.print("*Switch detection: " + String(i) + " ");
          Serial.println(digitalRead(mp->limit1Gpio));
        }
        continue;
      } 
    }

   // if (mp->towerSpeed != 0) {
  //    mp->towerSpeed--;
  //    continue;
  //  }

    mp->towerSpeed = mp->towerCurrentSpeed;
    //Serial.print('j');

    switch (mp->towerDirection) {

    case TOWER_STOPPED:
      break;

    case TOWER_UP:
      if (mp->towerLocationTicks >= mp->towerMoveTargetTicks) {
        mp->towerDirection = TOWER_STOPPED;

        if(!mp->leaveEnabled)
          digitalWrite(mp->enGpio, LOW);
          if (debugMode)
            Serial.println("*stoppedu: " + String(i) + " " + String(mp->towerLocationTicks) + " " + String(mp->towerMoveTargetTicks));
      
      }
      else
        if (towerCalcInterval(mp)) {
          mp->towerLocationTicks++;
          digitalWrite(mp->stepGpio, HIGH);
          delayMicroseconds(mp->stepDelayTime);
          digitalWrite(mp->stepGpio, LOW);

          //Serial.print('+');
          //Serial.println("*-mu-");
        }
      break;

    case TOWER_DOWN:
      if (mp->towerLocationTicks <= mp->towerMoveTargetTicks) {
        mp->towerDirection = TOWER_STOPPED;
        if(!mp->leaveEnabled)
           digitalWrite(mp->enGpio, LOW);
           if (debugMode)
             Serial.println("*stoppedd " + String(i) + " " + String(mp->towerLocationTicks) + " " + String(mp->towerMoveTargetTicks));
      }
      else
        if ( towerCalcInterval(mp)) {
          mp->towerLocationTicks--;
          digitalWrite(mp->stepGpio, HIGH);
          delayMicroseconds(mp->stepDelayTime);
          digitalWrite(mp->stepGpio, LOW);
          //Serial.print('-');
          //Serial.print(towerLocation);
          //Serial.println("*-md-");
        }
      break;
    }
  }
}

void setup() {

  Serial.begin(38400);
  pumpAuto = 0;
  pumpOn = 0;

  pinMode(PUMP_ON1, OUTPUT);  // pump off
  digitalWrite(PUMP_ON1, HIGH);
  pinMode(PUMP_ON2, OUTPUT);  // pump off
  digitalWrite(PUMP_ON2, HIGH);

  pinMode(OUT_VALVE, OUTPUT);  // pump off
  digitalWrite(OUT_VALVE, HIGH);

  pinMode(K1, INPUT);   
 
  pinMode(OUT_VALVE, OUTPUT);  // air valve closed
  digitalWrite(OUT_VALVE, HIGH);

 /*
  pinMode(LIFT_MAIN1, OUTPUT);
  digitalWrite(LIFT_MAIN1, HIGH);

  pinMode(LIFT_P1, OUTPUT);
  digitalWrite(LIFT_P1, HIGH);

  pinMode(LIFT_P2, OUTPUT);
  digitalWrite(LIFT_P2, HIGH);
*/

  airValveOn = 0;
  switchTest = 0;

  for (int i = 0; i < MOTORS_DEFINED; i++)
  {
      pinMode(motors[i].stepGpio ,OUTPUT);
      pinMode(motors[i].enGpio ,OUTPUT);
      pinMode(motors[i].dirGpio ,OUTPUT);
      pinMode(motors[i].limit1Gpio,INPUT_PULLUP);
      //pinMode(motors[i].limit2Gpio,INPUT_PULLUP);
      
      digitalWrite(motors[i].stepGpio,HIGH);   
      digitalWrite(motors[i].enGpio,LOW) ;

      motors[i].towerLocationTicks = 99999;  // don't really know where it is
      motors[i].towerDirection = TOWER_STOPPED;
      motors[i].switchMask  = 0;
      motors[i].calibrateState = CALIBRATE_IDLE;
      motors[i].towerBuckets = INTERVAL_SLOTS;
      motors[i].calibrationComplete = false;
      motors[i].loopMode = LOOP_MODE_IDLE;

      //Serial.println(motors[i].towerBuckets);
  }


    speedLimiter = 0;
   speedLimit = 0;
   
   // Init timer ITimer1
  ITimer1.init();

#define TIMER_INTERVAL_MS 1

  // Interval in unsigned long millisecs

  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
    Serial.println("*Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("*Can't set ITimer. Select another freq. or timer");

   speedLimiter = 0;
   speedLimit = 0;
   debugMode = 0;
   
  Serial.print("*Tower Setup Done ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.println(__TIME__);
}


/////////////////////
///
// Main execution loop

void loop() {

  int tank;
  unsigned int temp;
  MOTOR * mp;
  
  
  // handle any pending characters

  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();
     // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[aindex] = 0;
      else if (arg == 2) argv2[aindex] = 0;
      else if (arg == 3) argv3[aindex] = 0;
      else if (arg == 4) argv4[aindex] = 0;

      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
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
        // The first arg is the single-letter command
        cmd = chr;
      } else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[aindex] = chr;
        aindex++;
      } else if (arg == 2) {
        argv2[aindex] = chr;
        aindex++;
      } else if (arg == 3) {
        argv3[aindex] = chr;
        aindex++;
      } else if (arg == 4) {
        argv4[aindex] = chr;
        aindex++;
      }
    }
  }


  // handle state machines
  
  // see if doing calibrate. 

  for (int i = 0; i < MOTORS_DEFINED; i++)
  {
    mp = &motors[i];

    if (mp->calibrateState !=  CALIBRATE_IDLE)
    {
      // we are going down, did we hit hte bottom yet?

      if (mp->towerDirection == TOWER_STOPPED_SWITCH)
      {
        // we hit the bottom switch, 

        mp->towerLocationTicks = 0;
        mp->towerLocationMim = mp->towerLowerLimitMim;
        
        mp->calibrateState = CALIBRATE_IDLE;
        if (debugMode)
          Serial.println("*Cal Done: " + String(i));
        mp->calibrationComplete = true;

        if (mp->armid >= 0){
          arms[mp->armid].towerLocationMim = arms[mp->armid].lowestPositionMim;
        }

        if (motors[0].calibrationComplete && motors[1].calibrationComplete && motors[2].calibrationComplete && motors[3].calibrationComplete)
          Serial.println(" OK- Cal Done");

      }             
    }

    if (mp->towerDirection <= TOWER_STOPPED_SWITCH){
      switch(mp->loopMode)
      {
        int t;
        
        case LOOP_MODE_DOWN:
      
         mp->loopMode = LOOP_MODE_UP;
         mp->switchMask = 0;
         t = random (1000,mp->towerUpperLimitTicks * 0.75) & 0xFFFF;
         startMotor(mp,'u',t );  // eventually random number
         if (debugMode)
           Serial.println("**Going up: " + String(mp->motorid) + ": " + String(t));

         break;
         
        case LOOP_MODE_UP:
      
         mp->loopMode = LOOP_MODE_DOWN;
         mp->switchMask = BOT_SWITCH_TRIGGER;
         t = 0;
         startMotor(mp,'d',100000);
         if (debugMode)
           Serial.println("**Going down: " + String(mp->motorid) + ": " + String(t));
         break;
      
       case LOOP_MODE_IDLE:
         break;
     }
    }
  }

  if (switchTest)
  {
    temp = 0;
    for (int i = 0; i < MOTORS_DEFINED; i ++)
    {
     temp |=  (digitalRead(motors[i].limit1Gpio) << i);
    }
  
    if (temp != switchTestLast)
    {
      Serial.println(temp,HEX);
      switchTestLast = temp;
    }
  }
  
  if (pumpAuto) {
    // we are monitoring it

    if (pumpSampleTime < millis()) {
      // time to check if we should start or stop the pump

      tank = analogRead(TANK_SENSOR);
      //Serial.print(tank);
      //Serial.print(":");
      //Serial.println((tank * 10) / 51);

      if (pumpOn) {
        // pump is on, has it reached the point where we can turn it off

        if (tank > pumpOffThreshold) {
          // yes, turn pump off

          pumpOn = 0;
          digitalWrite(PUMP_ON1, HIGH);
          digitalWrite(PUMP_ON2, HIGH);
          //Serial.println("*pump off");
        }
      } else {
        // pump is off, do we need to turn it on

        if (tank < pumpOnThreshold) {
          // pump needs to be turned on

          pumpOn = 1;
          digitalWrite(PUMP_ON1, LOW);
          digitalWrite(PUMP_ON2, LOW);
          //Serial.println("*pump on");
        }
      }
      // set timer when we should check again

      pumpSampleTime = millis() + pumpSampleInterval;
    }
  }
}
