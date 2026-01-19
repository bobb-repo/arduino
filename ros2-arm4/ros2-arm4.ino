#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <stdio.h>
#include <LibPrintf.h>

#define USE_TIMER_1 false
#define USE_TIMER_2 true
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 false

#include "TimerInterrupt.h"

#define VALVE1       9
#define VALVE2      10
#define SOL3        11
#define LIGHT_IN1   15

#define ST3_LIM1    20
#define ST3_LIM2    21

#define ST2_DIR     22
#define ST2_STEP    23
#define ST2_ENABLE  24

#define ST3_DIR     25
#define ST3_STEP    26
#define ST3_ENABLE  27

#define ST2_LIM1    28
#define ST2_LIM2    29

#define SV1_DO      5
#define SV2_DO      4
#define SV3_DO      3
#define SV4_DO      2
#define SOL1        37
#define SOL2        38

#define ST1_LIM1    42
#define ST1_LIM2    43

#define ST1_ENABLE  44
#define ST1_STEP    46
#define ST1_DIR     47

#define ST1_POS_LEFT A10
#define ST2_POS_LEFT    A1
#define ST3_POS_LEFT    A9
#define LIGHT_IN2  A7
#define VAC_SENSOR_LEFT A3

#define ST1_POS_RIGHT    A0
#define ST2_POS_RIGHT    A1
#define ST3_POS_RIGHT    A2
#define LIGHT_IN2  A7
#define VAC_SENSOR_RIGHT A9

#define SV1_A      A10
#define SV2_A      A11
#define SV3_A      A12
#define SV4_A      A13


// A pair of varibles to help parse serial commands (thanks Fergs)

int arg = 0;
int idx = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
#define ARG_BUFFER_SIZE 16
char argv1[ARG_BUFFER_SIZE];
char argv2[ARG_BUFFER_SIZE];
char argv3[ARG_BUFFER_SIZE];
char argv4[ARG_BUFFER_SIZE];

// The arguments converted to integers
long arg1;
long arg2;
long arg3;
long arg4;

unsigned char breg = 0;
unsigned char limitTest;
unsigned char zin;
unsigned char zlast = 0xff;

#define ROT_CCW           'w'
#define ROT_CW            'c'
#define ROT_CCW_DEGREES   'y'
#define ROT_CW_DEGREES    'e'

#define ROT_HALT          'h'
#define ARM_LOCK          'k'

#define GET_ANGLES        's'
#define SET_ANGLES        'p'
#define ARM_SELECT        'a'

#define POINT             'o'

#define BASKET_OPEN       'b'
#define MINIARM_MOVE      'm'
#define VACUUM_VALVE      'v'
#define VACCUM_SENSOR     'u'
#define ATTACH_MODE       'g'
#define DOOR_POS          'j'

#define SPEED             'd'
#define DBUG              'z'
#define FAKE_MODE         'f'

// Motor states
#define MOTOR_IDLE  0
#define MOTOR_CCW  1
#define MOTOR_CW 2

// Motor stop flags
#define STOP_NONE            0
#define STOP_OUT_OF_TICKS    1
#define STOP_LOW_AV_TARGET   2
#define STOP_HIGH_AV_TARGET  4
#define STOP_NO_PROGRESS     5

#define HALT_LOW_AV  1
#define HALT_HIGH_AV 2

typedef struct MOTOR {
    char enGpio;
    char dirGpio;
    char stepGpio;
    char positionAnalogPin;
    char limit1;
    char limit2;
    
    float ticksPerDegree;
    float avPerDegree;
    int minDegree;
    int avAtMinDegree;
    int avAtMaxDegree;
    int reverse;
    char leaveEnabled;
    int idleEnableOnTimeMs;
     
    char enableOn;
    char direction;
    int beginZeroMonitoring;
    
    int stopFlag;
    char pulseHigh;
    char speedMode;
    int position;
    char haltTrigger;
    int lastPositionDegrees;

    int targetAv;
    int currentAv;
    int lastAv;
    
    int moveTicks;
    int ticksLeft;
    int ticksPerformed;           // how many ticks so far
    int ticksFullSpeedTrigger;    // when to go to full speed
    int ticksSlowDownTrigger;     // when to slow down
       
    int ticksSkip;                // ticks left to skip (for exponential ramp)

    int progressCheckCounter;     // counter to check if motor is making progress
    int lastProgressCheckAv;      // last AV reading when progress was checked

    long idleEnableTimeout;
  } MOTOR;
 
typedef struct MOTOR_LIMITS{
  int lowerLimit;
  int upperLimit;
  int defaultSpeed;
} MOTOR_LIMITS;


#define MOTORS_DEFINED 3

MOTOR leftMotors[MOTORS_DEFINED] = { {ST3_ENABLE,ST3_DIR,ST3_STEP,ST3_POS_LEFT,ST3_LIM1,ST3_LIM2,19.0,3.11,90,223, 783, 0,1,30000,MOTOR_IDLE,0,0},
                                     {ST2_ENABLE,ST2_DIR,ST2_STEP,ST2_POS_LEFT,ST2_LIM1,ST2_LIM2,15.9,3.11,90,234, 794 ,1,1,30000,MOTOR_IDLE,0,0}, 
                                     {ST1_ENABLE,ST1_DIR,ST1_STEP,ST1_POS_LEFT,ST1_LIM1,ST1_LIM2,15.9,3.11,90,196, 756, 1,1,30000,MOTOR_IDLE,0,0}  }; 

MOTOR_LIMITS leftMotorLimits[MOTORS_DEFINED] = {  {90,270,0},{90,270,0},{90,270,0} }  ;

MOTOR rightMotors[MOTORS_DEFINED] = { {ST3_ENABLE,ST3_DIR,ST3_STEP,ST3_POS_RIGHT,ST3_LIM1,ST3_LIM2,19.0,3.11,90,231, 791, 0,1,30000,MOTOR_IDLE,0,0},                                      
                                      {ST2_ENABLE,ST2_DIR,ST2_STEP,ST2_POS_RIGHT,ST2_LIM1,ST2_LIM2,15.9,3.11,90,255, 809 ,0,1,30000,MOTOR_IDLE,0,0}, 
                                      {ST1_ENABLE,ST1_DIR,ST1_STEP,ST1_POS_RIGHT,ST1_LIM1,ST1_LIM2,15.9,3.11,90,220, 780  ,1,1,30000,MOTOR_IDLE,0,0}  }; 


MOTOR_LIMITS rightMotorLimits[MOTORS_DEFINED] = {  {90,270,0},{90,270,0},{90,270,0} }  ;

MOTOR *motors;
MOTOR_LIMITS *motorLimits;

// Exponential ramp parameters (replaces skipIntervalTable)
#define MIN_TICK_DELAY    0     // Full speed - no delay between ticks
#define MAX_TICK_DELAY    8     // Starting speed - delay this many ticks

volatile unsigned int tickA = 0;
volatile byte flag = 0;
unsigned long lastTime;

#define LEFT_ARM  1
#define RIGHT_ARM 2

char whichArm;


#define NUM_SERVOS 4
Servo servo[NUM_SERVOS];  // create servo object to control a servo
int servoGpio [NUM_SERVOS] = {SV1_DO,SV2_DO,SV3_DO,SV4_DO};  // all these pins can do PWM the last two are always moved together
int servoAnalog[NUM_SERVOS] = {SV1_A,SV2_A,SV3_A,SV4_A};  // only two have analog inputs

#define SERVO_FIX  5
#define SERVO_FLAT  6

#define ZEROMASK 0xE0

unsigned char lighttest;
unsigned char lin;
unsigned char llast = 0xff;
unsigned long lighttestTime = 0;

unsigned long avReadTime = 0;
char avReadMode = MOTORS_DEFINED+1;

#define MOTOR_ACC    0
#define MOTOR_STEADY 1
#define MOTOR_DEC    2

#define AV_ADR 1.05
#define TICKS_PADDING 100

double mycurrentposition[3];
double* Robot_Plan;
//double RobotPlan[6] = { 0 };

#define MAX_TRAJ 20
#define NUMB_ANGLES 3

double trajectory[MAX_TRAJ][NUMB_ANGLES];
int trajCount = 0;
int trajIndex;

#define TRAJECTORY_IDLE 0
#define TRAJECTORY_ACTIVE 1

char trajectoryState = TRAJECTORY_IDLE;

int  speedLimiter = 0;
int  speedLimit = 0;
int  basketOpen = 0;
int  vacValveOn = 0;
int attachModeOn = 0;
int fakeMode = 0;
int lastSetAngles[3];
int debugMode = 0;

// Debug rate limiting and buffering
unsigned long lastVerboseDebugTime = 0;
#define DEBUG_PRINT_INTERVAL_MS 100  // Rate limit verbose debug to 10 Hz

// Debug message buffer
#define DEBUG_BUFFER_SIZE 32
struct DebugMessage {
  char msg[80];
  unsigned long timestamp;
};
DebugMessage debugBuffer[DEBUG_BUFFER_SIZE];
volatile int debugBufferHead = 0;
volatile int debugBufferTail = 0;

#define AV_ADJ 1.01
#define TICK_BUFFER 1


double aabs(float absvalue) {
	double myabsvalue;
	if (absvalue < 0)
		myabsvalue = -1 * absvalue;
	else
		myabsvalue = absvalue;

	return myabsvalue;
}


int getAvgAnalog(int port){


  int val = 0;
 // return analogRead(port);
  
  for (int i= 0; i < 2; i++)
  {
      val +=analogRead(port);
  }
  return int(val/2);
}

// Queue a debug message to buffer (can be called from ISR)
void queueDebugMessage(const char* msg) {
  int nextHead = (debugBufferHead + 1) % DEBUG_BUFFER_SIZE;
  if (nextHead != debugBufferTail) {
    strncpy(debugBuffer[debugBufferHead].msg, msg, 79);
    debugBuffer[debugBufferHead].msg[79] = 0;
    debugBuffer[debugBufferHead].timestamp = millis();
    debugBufferHead = nextHead;
  }
  // If buffer full, message is dropped (silent failure to avoid ISR blocking)
}

// Flush buffered debug messages (call from main loop)
void flushDebugBuffer() {
  while (debugBufferTail != debugBufferHead) {
    Serial.print(debugBuffer[debugBufferTail].timestamp);
    Serial.print(": ");
    Serial.println(debugBuffer[debugBufferTail].msg);
    debugBufferTail = (debugBufferTail + 1) % DEBUG_BUFFER_SIZE;
  }
}

int getCurrentPosition(int motor )
{
      int currentPosition;
      motors[motor].currentAv = getAvgAnalog(motors[motor].positionAnalogPin);
      currentPosition = ((float(motors[motor].currentAv - motors[motor].avAtMinDegree) ) / motors[motor].avPerDegree) + motors[motor].minDegree;

      if(debugMode) {
        Serial.print("*$ CPos M");
        Serial.print(motor);
        Serial.print(" av:");
        Serial.print(motors[motor].currentAv);
        Serial.print(" avMin:");
        Serial.print(motors[motor].avAtMinDegree);
        Serial.print(" pos:");
        Serial.println(currentPosition);
      }
      return currentPosition;
}

// Calculate exponential ramp delay for smooth acceleration/deceleration
// Optimized for ISR - uses integer math only, no floating point
int calculateExponentialDelay(int motor) {
    int delay;

    if (motors[motor].speedMode == MOTOR_ACC) {
        // Acceleration phase: exponential speed increase
        // Use integer math: delay = MAX * (remaining/total)^2
        int total = motors[motor].ticksFullSpeedTrigger;
        if (total > 0) {
            int remaining = total - motors[motor].ticksPerformed;
            if (remaining <= 0) {
                return MIN_TICK_DELAY;
            }

            // Calculate (remaining/total)^2 using integer math
            // delay = MAX_TICK_DELAY * (remaining * remaining) / (total * total)
            long numerator = (long)remaining * remaining;
            long denominator = (long)total * total;
            delay = (MAX_TICK_DELAY * numerator) / denominator;
        } else {
            delay = MIN_TICK_DELAY;
        }
        return delay;
    }
    else if (motors[motor].speedMode == MOTOR_DEC) {
        // Deceleration phase: exponential speed decrease
        // Calculate based on remaining ticks (slow down as we approach target)
        int remaining = motors[motor].ticksLeft;
        if (remaining <= 0) {
          // Serial.println("d1");
           return MIN_TICK_DELAY+1;
        }

        // Total deceleration zone size
        int totalDecelSteps = motors[motor].moveTicks - motors[motor].ticksSlowDownTrigger;
        if (totalDecelSteps <= 0) {
           //Serial.println("d2");
            return MIN_TICK_DELAY;
        }

        // Calculate (remaining/total)^2 using integer math
        // As remaining decreases, delay increases (motor slows down)
        long numerator = (long)remaining * remaining;
        long denominator = (long)totalDecelSteps * totalDecelSteps;
        delay = (MAX_TICK_DELAY * numerator) / denominator; 
        if (debugMode)
        
          Serial.println("d2: " + String(motor) + " " + String(remaining) + " " + String(totalDecelSteps));
           
        return delay;
    }

    // MOTOR_STEADY - full speed, no delay
    return MIN_TICK_DELAY;
}



void motorTimer() {
  int m;

  tickA++;

  if (tickA >=2000) //2000 x 250us = 500ms
  {
    flag = 1;
    tickA = 0;
    digitalWrite(13, !digitalRead(13));
  }
  

  for (m=0; m < MOTORS_DEFINED;m++)
  {
    if (motors[m].direction == MOTOR_IDLE)
      continue;

    //  see if we are done
    
    if ((motors[m].direction != MOTOR_IDLE) && (motors[m].stopFlag == STOP_NONE)  )
    {
       // Check target position - currentAv should be updated from main loop, not ISR
      if(motors[m].haltTrigger == HALT_LOW_AV)   // av is getting smaller as we rotate, stop when the current av is less than the target
      {
         if ((motors[m].currentAv < motors[m].targetAv) && (motors[m].targetAv != 0xffff))
         {
             // we are done
             motors[m].stopFlag = STOP_LOW_AV_TARGET;
             continue;
         }
      }
      
      else
      {
         // going CCW - av is getting larger as we rotate, stop when the current av matches or exceeds the target
          if ((motors[m].currentAv >= motors[m].targetAv)  && (motors[m].targetAv != 0xffff))
          {
             // we are done
             motors[m].stopFlag = STOP_HIGH_AV_TARGET;
             continue;
          }
      }

      // Calculate exponential delay and see if we should skip this tick interrupt
      if (motors[m].ticksSkip == 0)
      {
        motors[m].ticksSkip = calculateExponentialDelay(m);
      
//Serial.println("ms " + String(m) + " " +String(motors[m].ticksSkip));
      }
      else
      // if (a motors[m].ticksSkip > 0)
      {
        // Skip this tick, decrement counter
        motors[m].ticksSkip--;
        continue;
      }

      // do we have any ticks left?
      if ((motors[m].ticksLeft + TICKS_PADDING) == 0) {
        // no - set stop flag, will be handled in main loop
        motors[m].stopFlag = STOP_OUT_OF_TICKS;
        continue;
      }

      digitalWrite(motors[m].stepGpio, HIGH);
      delayMicroseconds(100);
      digitalWrite(motors[m].stepGpio, LOW);        
    // Serial.print('-');
        
      if ((debugMode == 3) && (m == 1))
        Serial.println("t " + String(m) + " " + String(motors[m].ticksLeft));
        
      motors[m].ticksLeft --;
      motors[m].position += (motors[m].direction == MOTOR_CW ? 1 : -1);
      motors[m].ticksPerformed++; 
        
      // check if we are actually making progress
        
     //if ( abs(motors[m].currentAv -  motors[m].lastAv) <  1)
      //   Serial.println('j');

      motors[m].lastAv =  motors[m].currentAv;

     // Check acceleration state transitions

      switch (motors[m].speedMode)
      {
      case MOTOR_ACC:
          // Still accelerating. Check if we've reached full speed
          if (motors[m].ticksPerformed >= motors[m].ticksFullSpeedTrigger)
          {
              // Transition to steady state
              motors[m].speedMode = MOTOR_STEADY;
              if (debugMode)
                Serial.println("a->s");
              // Debug message queued to buffer for printing in main loop
          }
          break;

      case MOTOR_DEC:
          // Already decelerating, exponential function handles the ramp
          break;

      case MOTOR_STEADY:
          // Check if time to start decelerating
          if (motors[m].ticksPerformed >= motors[m].ticksSlowDownTrigger)
           {
              // Transition to deceleration
              motors[m].speedMode = MOTOR_DEC;
              if (debugMode)
                Serial.println("s->d");
              // Debug message queued to buffer for printing in main loop
           }
           break;
      }
    }
  }
}

int moveToPosition(int motor,int newPosition,int start)
{
    int deltaAv,currentPosition;
    float degreesFromMin, t2, t3;

        if(debugMode)
          Serial.println("*mtp: " + String(motor) + " " + String(newPosition) );

        if (newPosition > motorLimits[motor].upperLimit )
        {
          Serial.println ("*ERR - too high: " + String(motor) + " " + String(newPosition) + " " + String(motorLimits[motor].upperLimit));
          newPosition = motorLimits[motor].upperLimit-4 ;
        }
        
        if (newPosition < motorLimits[motor].lowerLimit )
        {
          Serial.println ("*ERR - too low: " + String(motor) + " " + String(newPosition) + " " + String(motorLimits[motor].lowerLimit));
          newPosition = motorLimits[motor].lowerLimit +4 ;
        }

        degreesFromMin = newPosition - motors[motor].minDegree;
       
        if(debugMode)
          Serial.println("   *tn: " + String(degreesFromMin) + " "+ String(motors[motor].minDegree) + " " + String(motors[motor].avPerDegree));
        
        motors[motor].targetAv =  float(degreesFromMin) * motors[motor].avPerDegree * AV_ADJ + motors[motor].avAtMinDegree;
        motors[motor].currentAv = getAvgAnalog(motors[motor].positionAnalogPin);

        // if we are going from a higher av to a lower one, make sure the target is not too low
        
        if ((motors[motor].currentAv > motors[motor].targetAv) && ( motors[motor].targetAv < motors[motor].avAtMinDegree))
        {
          if (debugMode)
                Serial.println("targetAv below min" + String(motors[motor].targetAv));
          motors[motor].targetAv = motors[motor].avAtMinDegree;
        }
        else
        {
          // if we are going from a lower av to a higher one make sure the target is not too high
         if ((motors[motor].currentAv < motors[motor].targetAv) && ( motors[motor].targetAv > motors[motor].avAtMaxDegree))
          {
              if (debugMode)
                Serial.println("targetAv above max " + String(motors[motor].targetAv) + " " + String( motors[motor].avAtMaxDegree));
              motors[motor].targetAv = motors[motor].avAtMaxDegree;
           }
        }
        // see how much change av we have to make. it can be positive or negative
        
        deltaAv = motors[motor].targetAv - motors[motor].currentAv;               
        
        motors[motor].moveTicks = (deltaAv / motors[motor].avPerDegree) * motors[motor].ticksPerDegree ;
        
        if(debugMode)
          Serial.println("   *tgt aV: " + String(motors[motor].targetAv) + " cur aV " + String(motors[motor].currentAv) + " aV delta: " + String(deltaAv) + " ticks: " + String(motors[motor].moveTicks) + " dir " + String(motors[motor].reverse) );
 
        if (!start)
          return 1;
          
        // start all three motors now
        noInterrupts();
        for (int m = 0 ; m < MOTORS_DEFINED; m++)
        {
          if(debugMode)
            Serial.println("   *mt " + String(motors[m].moveTicks));
          
          if (motors[m].reverse == 0) 
          {
            if (motors[m].moveTicks >= 0)
            {
              startMotor(m, ROT_CCW,aabs(motors[m].moveTicks),0);
              motors[m].haltTrigger = HALT_HIGH_AV;
            }
            else
            {
              startMotor(m, ROT_CW,aabs(motors[m].moveTicks),0);
              motors[m].haltTrigger = HALT_LOW_AV;
            }
          }
          else
          {
            if (motors[m].moveTicks >= 0)
            {
              startMotor(m, ROT_CW,aabs(motors[m].moveTicks),0);
              motors[m].haltTrigger = HALT_HIGH_AV;
            }
            else
            {
              startMotor(m, ROT_CCW,aabs(motors[m].moveTicks),0);
              motors[m].haltTrigger = HALT_LOW_AV;
            }
          } 
        }
        interrupts();

  return 1;
}

void startMotor(int motor, int cmd,int ticks, int repeat)
{
        if(debugMode)
          Serial.println("*SM " + String(cmd,HEX) + " " + String(motor) + " " + String(ticks));
      
        if (cmd == ROT_CW)
        {
         if(debugMode)
            Serial.println("Going CW "  + String(ticks));
          motors[motor].direction = MOTOR_CW;
          digitalWrite(motors[motor].dirGpio,LOW) ; //(motors[motor].reverse ? HIGH: LOW));
        }
        else
        if (cmd == ROT_CCW)
        {  
          motors[motor].direction = MOTOR_CCW;
          digitalWrite(motors[motor].dirGpio,HIGH ); // (motors[motor].reverse ? LOW: HIGH));
          if(debugMode)
            Serial.println("Going CCW "  + String(ticks));
         }
         else {
          // Serial.println("*ERR - cmd");
         }

        motors[motor].stopFlag = STOP_NONE;
        motors[motor].ticksLeft = (ticks == 0? 2 : ticks);
        motors[motor].pulseHigh = 0;
        motors[motor].ticksPerformed = 0;

        // Calculate acceleration profile for exponential ramp
        // For short moves: less acceleration distance
        // For long moves: more time at full speed
        motors[motor].ticksFullSpeedTrigger = min(50, motors[motor].ticksLeft/8);
        motors[motor].ticksSlowDownTrigger = motors[motor].ticksLeft - min(50, motors[motor].ticksLeft/8);

        if (debugMode) {
           Serial.print("*StartMotor ");
           Serial.print(motor);
           Serial.print(" ticks:");
           Serial.print(motors[motor].ticksLeft);
           Serial.print(" accel:");
           Serial.print(motors[motor].ticksFullSpeedTrigger);
           Serial.print(" decel@:");
           Serial.println(motors[motor].ticksSlowDownTrigger);
        }

        motors[motor].speedMode = MOTOR_ACC;
        motors[motor].ticksSkip = 0;
        motors[motor].progressCheckCounter = 0;
        motors[motor].lastProgressCheckAv = motors[motor].currentAv;

         digitalWrite(motors[motor].enGpio ,LOW);
}


int runCommand() {
  int i = 0;
  int m,v;
  //char *p = argv1;
  //char *str;
  int rept, endpt,ticks;
  int currentPosition[3];
  double* hereismyposition;
  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);
  
    switch(cmd) {

  case '0':    // limit test

    if (arg1 == 1)
      limitTest = 1;
    else
      limitTest = 0;
    Serial.println ("OK");
    break;

  case '1':   // solenoid tests
    switch (arg1){
    case 1:
      digitalWrite(SOL1,(arg2 == 1 ? 1 : 0));
      break;

    case 2:
      digitalWrite(SOL2,(arg2 == 1 ? 1 : 0));
      break;
    
    case 3:
      digitalWrite(SOL3,(arg2 == 1 ? 1 : 0));
      break;
    
    default:
      Serial.println ("ERR");
      return 0;
    }
    Serial.println ("OK");
    break;

  case 2:
    break;

  case '3':  // vac sensor and light sensor
    Serial.print  ("OK: ");
    Serial.print  (getAvgAnalog(VAC_SENSOR_LEFT));
    break;

  case '4':  // vac 
    if (arg1 == 1)
      v = VALVE1;
    else
      v = VALVE2;
      
    if (arg2 == 1)
      digitalWrite(v,1);
    else 
      digitalWrite(v,0);
    Serial.println ("OK");
    break;

  case VACUUM_VALVE:
    if (arg1 == 1){
       vacValveOn = 1;
       digitalWrite(VALVE1,1);
    }
    else 
    {
      vacValveOn = 0;
      digitalWrite(VALVE1,0);
    }
    Serial.println ("OK");  
    break;

  case VACCUM_SENSOR:
     Serial.println ("OK -- 0");  
     break;
   
  case '5':
    while (1)
    {
      Serial.print( "OK: ");
      for (i = 0; i < MOTORS_DEFINED; i ++)
        Serial.print( String(i) + ":" + String(getAvgAnalog(motors[i].positionAnalogPin)) + " ");
      Serial.println("");
      if (arg1 == 0)
        return 0;
      delay(1000);
    }
    return 0;

  case '6':
    switch (argv1[0]) {
      
   case 'l':

        endpt = arg2 ; //(arg2 & 0xff);
        servo[0].write(endpt);
        break;
   
  case 'r':

        endpt = (arg2 & 0xff);
        servo[1].write(endpt);
        break;
    
    
   case 'p':

        endpt = (arg2 & 0xff);
        servo[2].write(endpt);
        break;
    
    case 'q':

        endpt = arg2 ; //(arg2 & 0xff);
        servo[3].write(endpt);
        break;
    
     case 's':
      endpt = (arg2 & 0xff);
      servo[2].write(endpt);
      servo[3].write(180-endpt+SERVO_FIX); 
      break;
      
    default:
      Serial.println ("Err");
      return 0;
    }
    Serial.println ("OK");
    break;

  case '7':
    
      Serial.println( "OK - " + String(getAvgAnalog(servoAnalog[0])) + " " + String(getAvgAnalog(servoAnalog[1])));
      
    break;
  
  case '8':    // sterpper test   6 servo# endpt repeat

   moveToPosition(arg1,arg2,1);
   Serial.println ("OK");
   break;

    if (arg1 < NUM_SERVOS)
     {
        arg1--;
        endpt = arg2 & 0xff;
        rept = (arg3 == 0 ? 1 : arg3);

        for (i = 0 ; i < rept; i++)
        {
          servo[arg1].write(0);
          delay(500);
          servo[arg1].write(endpt);
          delay(500);
        }
        Serial.println ("OK");
     }
     else
        Serial.println ("Err");
      break;

  case '9':
    
    if (arg1 <  MOTORS_DEFINED)   // for reading stepper motor pots
    {
      if (arg2 == 1)
      {
        avReadMode = arg1;
        avReadTime = millis() + 1000;
      } 
      else
        avReadMode = MOTORS_DEFINED+1;     
    }
    Serial.println ("OK");
    break;

  case DBUG:
    debugMode = arg1;
    Serial.println ("OK");
    break;
  
  case FAKE_MODE:
    fakeMode = arg1;
    Serial.println ("OK");
    break;



  case MINIARM_MOVE:
    Serial.println ("OK");
    break;
  
  case ATTACH_MODE:
  case DOOR_POS:   
    Serial.println ("OK");
    break;

  case ARM_SELECT:
  
    if (whichArm != RIGHT_ARM)
    {
      Serial.println("ERR");
      return 0;
    }

    if (argv1[0] == 'l')
    {
      motors = leftMotors;
      motorLimits = leftMotorLimits;
      whichArm = LEFT_ARM;
    }
    Serial.println("*OK ");
    break;
 
  case ROT_CCW_DEGREES:
  case ROT_CW_DEGREES:
  
    trajectoryState = TRAJECTORY_IDLE;
    if (arg1 < MOTORS_DEFINED)
    {
        ticks = int(motors[arg1].ticksPerDegree * arg2);
        
        cmd = (cmd == ROT_CCW_DEGREES ? 'w' : 'c');

        Serial.println(ticks);
        startMotor(arg1, cmd,ticks, 0);
        Serial.println("*OK ");
    }
    else
    {
      Serial.println("*ERR"); 
    }
    break;

  case ROT_CCW:
  case ROT_CW:
      
    trajectoryState = TRAJECTORY_IDLE;
    
       
      if (arg1 < MOTORS_DEFINED)
      {
        motors[arg1].targetAv = 0xffff;
        startMotor(arg1, cmd,arg2, 0);
        Serial.println("*OK ");
      }
      else
        Serial.println("*ERR"); 
      break;

  case ROT_HALT:
      for (i = 0; i < MOTORS_DEFINED; i++)
      {
        motors[i].stopFlag = STOP_OUT_OF_TICKS;
        motors[i].ticksLeft = 0;
        motors[i].pulseHigh = 0;
        digitalWrite(motors[i].enGpio ,HIGH);
        Serial.println("*OK");
      }
      break; 

  case ARM_LOCK:
      for (i=0; i < MOTORS_DEFINED;i++)
      {
        digitalWrite(motors[i].enGpio,(arg1 == 1? LOW : HIGH));
      }
      Serial.println("*OK");
      break;

  
  case SET_ANGLES:
    // set all three angles but only if all are idle

    if ((motors[0].direction != MOTOR_IDLE) || (motors[1].direction != MOTOR_IDLE) || (motors[2].direction != MOTOR_IDLE))
    {
           Serial.println("ERR-BUSY");
           return 0;
    }
    
    if (debugMode){
      Serial.println("set target:  (" + String(arg1) + ',' + String(arg2) + ','+ String(arg3) + ')');
      Serial.println("    current: (" + String(getCurrentPosition(0)) + ',' + String(getCurrentPosition(1)) + ',' + String(getCurrentPosition(2))  +')');
    }

    if (fakeMode)
    {
      lastSetAngles[0] =  arg1;
      lastSetAngles[1] =  arg2;
      lastSetAngles[2] =  arg3;
      Serial.println("*OK");
      return 0;
    }
    
    if (arg1 != 0)
      if (moveToPosition(0,abs(arg1),0) == 0)
      {
        Serial.println("*ERR 0");
        break;
      }
    
    if (arg2 != 0)
      if (moveToPosition(1,abs(arg2),0) == 0)
      {
        Serial.println("*ERR 1");
        break;
      }
 
    if (arg3 != 0)
      if (moveToPosition(2,abs(arg3),1) == 0)
      {
        Serial.println("*ERR 2");
        break;
      }
    
    Serial.println("*OK");
    break;

  case GET_ANGLES:
    for (int i = 0; i < 3; i++)
    {
      if (fakeMode == 1)
        currentPosition[i] = lastSetAngles[i];
      else
       currentPosition[i] =getCurrentPosition(i);
    //  Serial.println(currentPosition[i]);
    }

    Serial.print(String(currentPosition[0]) + ' ' + String(currentPosition[1]) + ' ' + String(currentPosition[2])  );
    
    if (whichArm == RIGHT_ARM)
    {
      Serial.println(" " + String(random(0, 255)) + " " + String(vacValveOn) + " " + String(attachModeOn) + " " + String(random(0, 60)) + " " + String(0) );   // return  vac valve, attach mode, vac sensor, attach status
    }else
    {
      Serial.println(" " + String(basketOpen) );   // return just door position, for now a 0
    }
    
    break;

  case POINT:
    trajectoryState = TRAJECTORY_IDLE;
    if (argv1[0] == 'g')
    {
      trajIndex = 0;
          
      trajectoryState = TRAJECTORY_ACTIVE;
      for (m = 0; m < MOTORS_DEFINED-1; m++)
        moveToPosition(m, trajectory[0][m], 1);            
    }  
    else if (argv1[0] == 'l')
      {
        trajIndex = 0;
        Serial.println(trajCount);

        for (m = 0; m < trajCount; m++)
        {
          Serial.print("(" + String(trajectory[m][0]) ); 
          Serial.print("," + String(trajectory[m][1]) ); 
          Serial.println("*," + String(trajectory[m][2]) + ")" ); 
        }                
      } 
 
    Serial.println("*OK");
    break;

   case BASKET_OPEN:
     if (argv1[0] == '1')
     {
        basketOpen = 1;
        
        // rotate basket 90 degrees, a bit at atime

        for (i = SERVO_FLAT; i != 140; i++)
        {
          servo[2].write(i);
          servo[3].write(180-i+SERVO_FIX);
           delay(50);
       }
     }
     else
     {  // move it back to flat
        basketOpen = 0;
         for (i = 140; i != SERVO_FLAT; i--)
        {
          servo[2].write(i);
          servo[3].write(180-i+SERVO_FIX);
          delay(50);
        }
     }
     Serial.println("*OK");
     break;
  }
}
   

/* Clear the current command parameters */
void resetCommand() {
   
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
 
  idx = 0;
  arg = 0;
}

void setup() {

    int i;
  
    Serial.begin(38400);
    Wire.begin();
   
    pinMode(SOL1 ,OUTPUT);
    pinMode(SOL2 ,OUTPUT);
    pinMode(SOL3 ,OUTPUT);
    pinMode( VALVE1,OUTPUT);
    pinMode( VALVE2,OUTPUT);

    pinMode( LIGHT_IN1,INPUT_PULLUP);

    debugMode = 0;
    
    motors = rightMotors;
    motorLimits = rightMotorLimits;
 

    whichArm = RIGHT_ARM;

    for (i = 0; i < MOTORS_DEFINED; i++)
    {
      pinMode(motors[i].stepGpio ,OUTPUT);
      pinMode(motors[i].enGpio ,OUTPUT);
      pinMode(motors[i].dirGpio ,INPUT);
      pinMode(motors[i].limit1,INPUT_PULLUP);
      pinMode(motors[i].limit2,INPUT_PULLUP);
      
      digitalWrite(motors[i].stepGpio,HIGH);   
     
      if (motors[i].leaveEnabled)
      {
        digitalWrite(motors[i].enGpio,LOW) ;
        motors[i].idleEnableTimeout = millis() + motors[i].idleEnableOnTimeMs;
        motors[i].enableOn = 1;
      }
      else
      {
        digitalWrite(motors[i].enGpio,HIGH) ;
        motors[i].enableOn = 0;
      }   
        motors[i].direction = MOTOR_IDLE; 
      //motors[i].ticksPerAv = motors[i].ticksPerDegree/motors[i].avPerDegree;

    }
   
    for (i = 0; i < NUM_SERVOS;i++)
    {
     servo[i].attach(servoGpio[i]);
     servo[i].write(SERVO_FLAT);
    }
     servo[3].write(SERVO_FLAT+SERVO_FIX);


  // Init timer ITimer1

  ITimer2.init();

#define TIMER_INTERVAL_MS 2

  // Interval in unsigned long millisecs

  ITimer2.attachInterruptInterval(TIMER_INTERVAL_MS, motorTimer);

    avReadMode = MOTORS_DEFINED+1;

  speedLimiter = 0;
  speedLimit = 0;
  basketOpen = 0;
  vacValveOn = 0;
  attachModeOn = 0;

  Serial.print("* Arm Setup Done: " );
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.println(__TIME__);

 
}

void loop() {

  // Handle serial input - process up to 10 characters per loop to avoid blocking
  int charCount = 0;
  while (Serial.available() > 0 && charCount < 10) {
    charCount++;

    // Read the next character
    chr = Serial.read();

    // Skip invalid characters (control chars except CR and LF, high ASCII)
    if ((chr < ' ' && chr != 13 && chr != 10) || chr > 126) {
      continue;
    }

    // Treat both CR and LF as command terminators
    if (chr == 13 || chr == 10) {
      // Only process if we have a command
      if (cmd != 0) {
        // Null-terminate current argument
        if (arg == 1) argv1[idx] = 0;
        else if (arg == 2) argv2[idx] = 0;
        else if (arg == 3) argv3[idx] = 0;
        else if (arg == 4) argv4[idx] = 0;

        runCommand();
        resetCommand();
      }
      continue;
    }

    // Use spaces to delimit parts of the command
    if (chr == ' ') {
      // Skip multiple spaces
      if (idx == 0 && arg != 0) {
        continue;
      }

      // Step through the arguments
      if (arg == 0) {
        arg = 1;
      } else if (arg == 1)  {
        argv1[idx] = 0;
        arg = 2;
        idx = 0;
      } else if (arg == 2)  {
        argv2[idx] = 0;
        arg = 3;
        idx = 0;
      } else if (arg == 3)  {
        argv3[idx] = 0;
        arg = 4;
        idx = 0;
      } else if (arg == 4)  {
        argv4[idx] = 0;
        arg = 5;
        idx = 0;
      }
      continue;
    }

    // Process regular characters
    if (arg == 0) {
      // The first arg is the single-letter command
      cmd = chr;
    }
    else if (arg == 1) {
      // Subsequent arguments can be more than one character
      if (idx < ARG_BUFFER_SIZE - 1) {
        argv1[idx] = chr;
        idx++;
      } else {
        // Buffer overflow protection
        if (debugMode)
          Serial.println("*ERR: argv1 overflow");
        resetCommand();
      }
    }
    else if (arg == 2) {
      if (idx < ARG_BUFFER_SIZE - 1) {
        argv2[idx] = chr;
        idx++;
      } else {
        if (debugMode)
          Serial.println("*ERR: argv2 overflow");
        resetCommand();
      }
    }
    else if (arg == 3) {
      if (idx < ARG_BUFFER_SIZE - 1) {
        argv3[idx] = chr;
        idx++;
      } else {
        if (debugMode)
          Serial.println("*ERR: argv3 overflow");
        resetCommand();
      }
    }
    else if (arg == 4) {
      if (idx < ARG_BUFFER_SIZE - 1) {
        argv4[idx] = chr;
        idx++;
      } else {
        if (debugMode)
          Serial.println("*ERR: argv4 overflow");
        resetCommand();
      }
    }
  }
  
  // Flush any buffered debug messages first
  if (debugMode) {
    flushDebugBuffer();
  }

  // Update analog readings for active motors (moved out of ISR for performance)
  unsigned long now = millis();
  bool timeForVerboseDebug = (debugMode >= 2) && (now - lastVerboseDebugTime >= DEBUG_PRINT_INTERVAL_MS);

  for (int m=0; m < MOTORS_DEFINED; m++)
  {
    if (motors[m].direction != MOTOR_IDLE)
    {
      // Update current position from analog feedback
      motors[m].currentAv = getAvgAnalog(motors[m].positionAnalogPin);

      // Rate-limited verbose debug (max 10 Hz)
      if (timeForVerboseDebug)
      {
        Serial.print("*AV M");
        Serial.print(m);
        Serial.print(" cur:");
        Serial.print(motors[m].currentAv);
        Serial.print(" tgt:");
        Serial.println(motors[m].targetAv);
      }

      // Check if motor is making progress every ~100 loop iterations
      motors[m].progressCheckCounter++;
      if (motors[m].progressCheckCounter >= 200)
      {
        // Check if AV has changed by at least 2 units (accounting for noise)
        int avDelta = abs(motors[m].currentAv - motors[m].lastProgressCheckAv);

        if (avDelta < 2)
        {
          // Motor is not making progress - might be stalled or hitting obstacle
          if (debugMode) {
            Serial.print("*WARNING M");
            Serial.print(m);
            Serial.print(" NO PROGRESS av:");
            Serial.print(motors[m].currentAv);
            Serial.print(" last:");
            Serial.print(motors[m].lastProgressCheckAv);
            Serial.print(" delta:");
            Serial.print(avDelta);
            Serial.print(" ticks:");
            Serial.println(motors[m].ticksPerformed);
          }

          // Could optionally stop the motor here:
          // motors[m].stopFlag = STOP_NO_PROGRESS;
        }
        else if (debugMode >= 3)
        {
          Serial.print("*PROGRESS M");
          Serial.print(m);
          Serial.print(" OK delta:");
          Serial.println(avDelta);
        }

        // Reset for next check
        motors[m].progressCheckCounter = 0;
        motors[m].lastProgressCheckAv = motors[m].currentAv;
      }
    }
  }

  // Update timestamp after printing verbose debug
  if (timeForVerboseDebug) {
    lastVerboseDebugTime = now;
  }

  // do idle setting here since cannot to mcp calls from inside an ISR

  for (int m=0; m < MOTORS_DEFINED;m++)
  {
    if (motors[m].direction == MOTOR_IDLE)
    {
      if (motors[m].leaveEnabled)
      {
          if (millis() > motors[m].idleEnableTimeout)
          {  
              // time to turn off 

              if (motors[m].enableOn)  // on, turn it off
              {
                 //motors[m].idleEnableTimeout = millis() + (10000-motors[m].idleEnableOnTimeMs);
                 motors[m].enableOn = 0;
                 digitalWrite(motors[m].enGpio,HIGH);
              }
              else
              {
                 //motors[m].idleEnableTimeout = millis() + (motors[m].idleEnableOnTimeMs);
                 //motors[m].enableOn = 1;
                // digitalWrite(motors[m].enGpio,LOW);
              }
          }
      }  
    }
    else
    {
      if (motors[m].stopFlag != STOP_NONE)
      {
          if (debugMode)
          {
            const char* reason;
            if (motors[m].stopFlag == STOP_OUT_OF_TICKS) reason = "[OUT_OF_TICKS]";
            else if (motors[m].stopFlag == STOP_LOW_AV_TARGET) reason = "[LOW_AV_TARGET]";
            else if (motors[m].stopFlag == STOP_HIGH_AV_TARGET) reason = "[HIGH_AV_TARGET]";
            else if (motors[m].stopFlag == STOP_NO_PROGRESS) reason = "[NO_PROGRESS/STALLED]";
            else reason = "[UNKNOWN]";

            Serial.print("*STOP M");
            Serial.print(m);
            Serial.print(" ");
            Serial.print(reason);
            Serial.print(" ticks:");
            Serial.print(motors[m].ticksLeft);
            Serial.print(" av:");
            Serial.print(motors[m].currentAv);
            Serial.print("/");
            Serial.println(motors[m].targetAv);
          }
          motors[m].direction = MOTOR_IDLE;

          if (motors[m].leaveEnabled)
          {
              motors[m].idleEnableTimeout = millis() + (motors[m].idleEnableOnTimeMs);
              motors[m].enableOn = 1;  
              digitalWrite(motors[m].enGpio,LOW);
          }
          else
            digitalWrite(motors[m].enGpio,HIGH);
       }
    }
  }
  
  
  if (trajectoryState == TRAJECTORY_ACTIVE)
  {
    // we are doing point move, we go on to the next point when all three motors are stopped

    if (motors[0].stopFlag && motors[1].stopFlag ) // && motors[2].stopFlag )
    { 
       Serial.print("step complete ");

       // move on to the next point if there is one
  
      trajIndex++;
      
      if (trajIndex < trajCount)
       {
          // got another point
         Serial.print("next point ");
         Serial.print(trajIndex);
         Serial.println( " (" + String(trajectory[trajIndex][0]) + "," +String(trajectory[trajIndex][1]) + "," +String(trajectory[trajIndex][2]) + ")"  );

          for (int m = 0; m < MOTORS_DEFINED-1; m++)
            moveToPosition(m, trajectory[trajIndex][m], (m ==2 ? 0 : 0));  // 1            
       }
       else
       {
        int currentPosition[3];
         trajectoryState = TRAJECTORY_IDLE;
         Serial.println("*Movement Complete");
         for (int  i = 0; i < 3; i++)
         {
             currentPosition[i] = getCurrentPosition(i);
             Serial.println(currentPosition[i]);
         }

       }
    }
  }

  if (limitTest)
  {
     zin = (digitalRead(ST1_LIM1) << 5) + (digitalRead(ST1_LIM2) << 4) + (digitalRead(ST2_LIM1) << 3) + (digitalRead(ST2_LIM2) << 2) + (digitalRead(ST3_LIM1) << 1) + (digitalRead(ST3_LIM2)) ;
     if (1) { //zin != zlast){
       Serial.print("L: ");
       Serial.println(zin,HEX);
       zlast = zin;
     }
  }

  

  if (avReadTime  < millis())
  {
    avReadTime = millis() + 1000;
    switch (avReadMode)
    {
    case 0:
      Serial.println (getAvgAnalog(ST1_POS_LEFT));
      break;
      
    case 1:
      Serial.println (getAvgAnalog(ST2_POS_LEFT));
      break;
      
    case 2:
      Serial.println (getAvgAnalog(ST3_POS_LEFT));
      break;

    default:
      break;
    }
  }
}  
