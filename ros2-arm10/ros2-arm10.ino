#include <TMC2209.h>

#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <stdio.h>
#include <LibPrintf.h>
#include <VL53L1X.h>

char version[] = {" 2.10"};

#define USE_TIMER_1 false
#define USE_TIMER_2 true
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 false

#include "TimerInterrupt.h"

// GPIO Assignments

#define ST1_DIR     6
#define ST1_STEP    7
#define ST1_ENABLE  8
#define ST1_UART_TX 37
#define ST1_UART_RX 38

#define ST2_DIR     3
#define ST2_STEP    19
#define ST2_ENABLE  18
#define ST2_UART_TX 35
#define ST2_UART_RX 36

#define ST3_DIR     29
#define ST3_STEP    30
#define ST3_ENABLE  31
#define ST3_UART_TX 33
#define ST3_UART_RX 34

#define SOL1        10
#define SOL2        11

#define X_GP_1     43
#define X_GP_2     16
#define X_GP_3     15
#define X_GP_4     45
#define X_GP_5     46

#define SV1_DO      4
#define SV2_DO      5

// Analog Assignments

#define SV1_A      A10
#define SV2_A      A11

#define VAC_SENSOR    A9
#define VAC_SENSOR_BU A10

#define ST1_POS       A0
#define ST2_POS       A1
#define ST3_POS       A2

#define ST1_POS_BU    A3
#define ST2_POS_BU    A4
#define ST3_POS_BU    A5

#define X_AN_1     A13
#define X_AN_2     A14
#define X_AN_3     A15
#define X_AN_4     A6
#define X_AN_5     A7

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
#define ARM_PDN           'm'

#define GET_ANGLES        's'
#define SET_ANGLES        'p'
#define ARM_SELECT        'a'

#define POINT             'o'

#define BASKET_OPEN       'b'
#define VACCUM_SENSOR     'u'
#define ATTACH_MODE       'g'
#define DOOR_POS          'j'

#define DISTANCE          'd'
#define DBUG              'z'
#define FAKE_MODE         'f'
#define RESET             'r'
#define SHORT_MOTOR_LIMIT  'x'

// Motor direction
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
    char uart_tx;
    char uart_rx;
    
    float ticksPerDegree;
    float avPerDegree;
    int minDegree;
    int avAtMinDegree;
    int avAtMaxDegree;
    int invertedPosition;
    int invertedRotation;
    char leaveEnabled;
    long idleEnableOnTimeMs;
     
    char motorState;
    char direction;

    int beginZeroMonitoring;
    
    volatile int stopFlag;
    char pulseHigh;
    char speedMode;
    int position;
    char haltTrigger;
    int lastPositionDegrees;

    volatile int targetAv;
    volatile int currentAv;
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

MOTOR leftMotors[MOTORS_DEFINED] = { {ST1_ENABLE,ST1_DIR,ST1_STEP,ST1_POS,ST1_UART_TX,ST1_UART_RX,19.0,3.08,90,188, 760, 0,1,1,30000,MOTOR_IDLE,0,0},
                                     {ST2_ENABLE,ST2_DIR,ST2_STEP,ST2_POS,ST2_UART_TX,ST2_UART_RX,15.9,3.08,90,211 ,770 ,1,1,1,30000,MOTOR_IDLE,0,0}, 
                                     {ST3_ENABLE,ST3_DIR,ST3_STEP,ST3_POS,ST3_UART_TX,ST3_UART_RX,15.0,3.08,90,250, 790, 0,1,1,30000,MOTOR_IDLE,0,0} };
                                     
MOTOR_LIMITS leftMotorLimits[MOTORS_DEFINED] = {  {90,270,0},{90,270,0},{90,270,0} }  ;

MOTOR rightMotors[MOTORS_DEFINED] = { {ST1_ENABLE,ST1_DIR,ST1_STEP,ST1_POS,ST1_UART_TX,ST1_UART_RX,19.0,3.08,90,220, 804 ,1,0,1,30000,MOTOR_IDLE,0,0},
                                      {ST2_ENABLE,ST2_DIR,ST2_STEP,ST2_POS,ST2_UART_TX,ST2_UART_RX,15.9,3.08,90,250, 808 ,0,0,1,30000,MOTOR_IDLE,0,0}, 
                                      {ST3_ENABLE,ST3_DIR,ST3_STEP,ST3_POS,ST3_UART_TX,ST3_UART_RX,15.9,3.08,90,217, 802, 1,0,1, 30000,MOTOR_IDLE,0,0}  };

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
#define END_AV_THRESHOLD 8

char whichArm;

int returnError = 0;


#define NUM_SERVOS 2
Servo servo[NUM_SERVOS];  // create servo object to control a servo
int servoGpio [NUM_SERVOS] = {SV1_DO,SV2_DO};  // all these pins can do PWM the last two are always moved together
int servoAnalog[NUM_SERVOS] = {SV1_A,SV2_A};  // only two have analog inputs

#define SERVO_FIX  5
#define SERVO_FLAT  6

/* Every servo write goes through this.  The handlers below were written for a
   four-servo design and index servo[2] and servo[3], but NUM_SERVOS is 2 and
   only servo[0] and servo[1] are attached in setup().  Servo::write() on an
   out-of-range object reads whatever servoIndex sits in adjacent memory, and
   on a Mega that index can land inside the Servo library's own table -- so the
   write could move a real, attached servo instead of doing nothing.

   Bounded rather than remapped.  The intended physical pair is not derivable
   from the code, and guessing wrong drives a real mechanism through a full
   sweep.  Returns false so callers can report; see BASKET_OPEN. */
bool servoWrite(int idx, int angle)
{
  if (idx < 0 || idx >= NUM_SERVOS)
    return false;
  servo[idx].write(angle);
  return true;
}




int lastDistance;

#define POSITION_READ_INTERVAL_MS 60
long timeToReadPosition = 0;


unsigned long avReadTime = 0;
char avReadMode = MOTORS_DEFINED+1;

#define MOTOR_ACC    0
#define MOTOR_STEADY 1
#define MOTOR_DEC    2
#define MOTOR_SHORT  3

#define MOTOR_ACTIVE     2
#define MOTOR_HOLD       1
#define MOTOR_DISABLED   0


int shortModeLimit = 9999;
int allowedJointOffset = 4;

#define AV_ADR 1.05
#define TICKS_PADDING 1000

/* STEP pulse width, microseconds.  Was 300, which blocked the 2 ms timer ISR
   for up to 900 us with three motors stepping -- 45% of the period with
   interrupts disabled, enough to drop Timer0 ticks and drift millis().  The
   TMC2209 needs roughly 100 ns, so 5 us is still ~50x margin.  Raise it if
   steps are missed on long cabling. */
#define STEP_PULSE_US 5

double mycurrentposition[3];
double* Robot_Plan;
//double RobotPlan[6] = { 0 };

#define MAX_TRAJ 1
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
int errorCode = 0;

char firstBusyCall = 0;

// Debug rate limiting and buffering
unsigned long lastVerboseDebugTime = 0;
#define DEBUG_PRINT_INTERVAL_MS 100  // Rate limit verbose debug to 10 Hz

#define AV_ADJ 1.05
#define TICK_BUFFER 1

VL53L1X sensor;


void setMotorState(int m,int newState)
{
   if (motors[m].motorState == newState)
   {
      if(debugMode)   
          Serial.println("Motor lp " + String(m) + " " + String(newState));
     return;
   }
   
   switch (newState) {
    
   case MOTOR_ACTIVE:
      digitalWrite(motors[m].enGpio,LOW) ;
      digitalWrite(motors[m].uart_tx,HIGH) ;
      digitalWrite(motors[m].uart_rx,HIGH) ;
      if(debugMode)
          Serial.println("Motor " + String(m) + " Active" );
      motors[m].motorState= newState;
      return;

   case MOTOR_DISABLED:
      digitalWrite(motors[m].enGpio,HIGH) ;
      digitalWrite(motors[m].uart_tx,LOW) ;
      digitalWrite(motors[m].uart_rx,LOW) ;
       if(debugMode)
          Serial.println("Motor " + String(m) + " Disabled" );
      motors[m].motorState= newState;
      return;
   
   case MOTOR_HOLD:     
      digitalWrite(motors[m].enGpio,LOW) ;
      digitalWrite(motors[m].uart_tx,HIGH) ;
      digitalWrite(motors[m].uart_rx,HIGH) ;
      if(debugMode)
          Serial.println("Motor " + String(m) + " Hold" );
      
      motors[m].idleEnableTimeout = millis() + motors[m].idleEnableOnTimeMs;
      motors[m].motorState= newState;
      if(debugMode)
          Serial.println("Lock arm " + String(m) );
      return;
   }
}

void jointPdn(int m, int val)
{
      digitalWrite(motors[m].uart_tx,val) ;
      digitalWrite(motors[m].uart_rx,val) ;
      
      if(debugMode)
          Serial.println("pdn arm " + String(m) + " " + String(val));

}


double aabs(float absvalue) {
	double myabsvalue;
	if (absvalue < 0)
		myabsvalue = -1 * absvalue;
	else
		myabsvalue = absvalue;

	return myabsvalue;
}

#define ANALOG_READS 4

int lastAnalogReads[MOTORS_DEFINED][ANALOG_READS];
int analogReadCount[MOTORS_DEFINED];

int getAvgAnalog(int motor,int port,int oneRead,int invert){

  int val = analogRead(port);
  int s,t;
  
  if (invert) 
    val = 1023-val;
    
 // return val;
  
 // Serial.println("ar :" +String(motor) + ":" +  String(invert) + " " + String(val));
  
  lastAnalogReads[motor][analogReadCount[motor]++ & (ANALOG_READS-1)] = val;

  if (oneRead)
    return val;
    
  if (analogReadCount[motor] <= ANALOG_READS)
  {
  //  Serial.println("");
    return val;
  }
  s = 0;
  for (int i=0; i < ANALOG_READS; i++)
  {
    s += lastAnalogReads[motor][i];
    //Serial.print(" (" + String(lastAnalogReads[motor][i]) + " " + String(s) + ")" );
  }
  //Serial.println("");

   t = s/ANALOG_READS;
  //Serial.println(" sum "+ String(t));
   

    
  return t;

}


int getCurrentPosition(int motor )
{
      int currentPosition;

      motors[motor].currentAv = getAvgAnalog(motor,motors[motor].positionAnalogPin,false,motors[motor].invertedPosition);

     // if (motors[motor].invertedPosition)
    //    currentPosition = ((float(motors[motor].avAtMinDegree - motors[motor].currentAv) ) / motors[motor].avPerDegree) + motors[motor].minDegree;
     // else
        currentPosition = ((float(motors[motor].currentAv - motors[motor].avAtMinDegree) ) / motors[motor].avPerDegree) + motors[motor].minDegree;

      if(debugMode) {
        Serial.print("*$ CPos M");
        Serial.print(motor);
        Serial.print(" rev:");
        Serial.print(motors[motor].invertedPosition);
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
  int t;
  
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

    //  we are still moving, have we previously detected the end?
    
    if (motors[m].stopFlag == STOP_NONE)
    {
      // not yet at end
#ifdef xx
      
      if(motors[m].haltTrigger == HALT_LOW_AV)   // av is getting smaller as we rotate, stop when the current av is less than the target
      {
         if ((motors[m].currentAv <= motors[m].targetAv) && (motors[m].targetAv != 0xffff))
         {
             // we are done
             motors[m].stopFlag = STOP_LOW_AV_TARGET;

             if (debugMode)
               Serial.println("lo " + String(m) + " c " +String(motors[m].currentAv) + "/t " + String(motors[m].targetAv));
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

             if (debugMode)
               Serial.println("hi " + String(m) + " c " +String(motors[m].currentAv) + "/t " + String(motors[m].targetAv) );
             continue;
          }
      }
#endif
     t = abs(motors[m].currentAv - motors[m].targetAv);
     
      if (t < END_AV_THRESHOLD)
      {
        motors[m].stopFlag = STOP_HIGH_AV_TARGET;
        /* No Serial here: this is an ISR.  loop() already prints the stop with
           the same information (see the "*STOP M" block). */
        continue;
      }
      
      // Calculate exponential delay and see if we should skip this tick interrupt
      if (motors[m].ticksSkip == 0)
      {
        motors[m].ticksSkip = 0; //calculateExponentialDelay(m);
      
//Serial.println("ms " + String(m) + " " +String(motors[m].ticksSkip));
      }
      else
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
      delayMicroseconds(STEP_PULSE_US);
      digitalWrite(motors[m].stepGpio, LOW);
      /* No per-tick Serial here: at up to ~500 ticks/s per motor this ran
         inside the ISR. */
        
      motors[m].ticksLeft --;
      motors[m].position += (motors[m].direction == MOTOR_CW ? 1 : -1);
      motors[m].ticksPerformed++; 
        
      
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
           }
           break;
       
       case MOTOR_SHORT:
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
        
        motors[motor].targetAv =  float(degreesFromMin) * motors[motor].avPerDegree + motors[motor].avAtMinDegree;
        motors[motor].currentAv = getAvgAnalog(motor,motors[motor].positionAnalogPin,false,motors[motor].invertedPosition);

        // if we are going from a higher av to a lower one, make sure the target is not too low
        
        if ((motors[motor].currentAv > motors[motor].targetAv) && ( motors[motor].targetAv < motors[motor].avAtMinDegree))
        {
          if (1)  //debugMode)
            Serial.println("targetAv below min" + String(motors[motor].targetAv));
          
          motors[motor].targetAv = motors[motor].avAtMinDegree;
        }
        else
        {
          // if we are going from a lower av to a higher one make sure the target is not too high
          if ((motors[motor].currentAv < motors[motor].targetAv) && ( motors[motor].targetAv > motors[motor].avAtMaxDegree))
          {
            if (1) //debugMode)
              Serial.println("targetAv above max " + String(motors[motor].targetAv) + " " + String( motors[motor].avAtMaxDegree));
            motors[motor].targetAv = motors[motor].avAtMaxDegree;
          }
        }
        // see how much change av we have to make. it can be positive or negative
        
        deltaAv = motors[motor].targetAv - motors[motor].currentAv;               
        
        motors[motor].moveTicks = (deltaAv / motors[motor].avPerDegree) * motors[motor].ticksPerDegree ;
        
        if(debugMode)
          Serial.println("   *tgt aV: " + String(motors[motor].targetAv) + " cur aV " + String(motors[motor].currentAv) + " aV delta: " + String(deltaAv) + " ticks: " + String(motors[motor].moveTicks) + " dir " + String(motors[motor].invertedPosition) );
 
        noInterrupts();

        if(debugMode)
            Serial.println("   *mt " + String(motors[motor].moveTicks));
          
        if (motors[motor].invertedRotation == 0) 
        {
            if (motors[motor].moveTicks >= 0)
            {
              startMotor(motor, ROT_CCW,aabs(motors[motor].moveTicks),0);
              motors[motor].haltTrigger = HALT_HIGH_AV;
            }
            else
            {
              startMotor(motor, ROT_CW,aabs(motors[motor].moveTicks),0);
              motors[motor].haltTrigger = HALT_LOW_AV;
            }
        }
        else
        {
            if (motors[motor].moveTicks >= 0)
            {
              startMotor(motor, ROT_CW,aabs(motors[motor].moveTicks),0);
              motors[motor].haltTrigger = HALT_HIGH_AV;
            }
            else
            {
              startMotor(motor, ROT_CCW,aabs(motors[motor].moveTicks),0);
              motors[motor].haltTrigger = HALT_LOW_AV;
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
          digitalWrite(motors[motor].dirGpio,LOW) ; //(motors[motor].invertedPosition ? HIGH: LOW));
        }
        else
        if (cmd == ROT_CCW)
        {  
          if(debugMode)
            Serial.println("Going CCW "  + String(ticks));
          
          motors[motor].direction = MOTOR_CCW;
          digitalWrite(motors[motor].dirGpio,HIGH ); // (motors[motor].invertedPosition ? LOW: HIGH));
          
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

        if ( ticks < shortModeLimit)
        {
          motors[motor].speedMode = MOTOR_SHORT;
        }
        else
        {
          motors[motor].speedMode = MOTOR_ACC;
        }
        motors[motor].ticksSkip = 0;
        motors[motor].progressCheckCounter = 0;
        motors[motor].lastProgressCheckAv = motors[motor].currentAv;
        analogReadCount[motor] = 0;
  
        setMotorState(motor,MOTOR_ACTIVE);
}

   
int moveJoint(int joint,int arg)
{
   int t;
   
   
    t = abs(abs(arg)-getCurrentPosition(joint));
    
    if ((t < allowedJointOffset) || (arg == 0))
    {
      if (debugMode)
        Serial.println("m" + String(joint) + " same angle, close, no move req "+String(t));
 
 
      return 0;
    }

     if (debugMode)
       Serial.println("m" + String(joint) + " same angle, move req " + String(t));
 

    return 1;
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
    
 
    default:
      Serial.println ("ERR");
      return 0;
    }
    Serial.println ("OK");
    break;

  case 2:
    digitalWrite(motors[arg1].dirGpio,arg2);   
    Serial.println ("OK");

    break;

  case '3':  // vac sensor
    while (1)
    {
    
      Serial.println(analogRead(VAC_SENSOR_BU));
      if (arg1 == 0)
        return;
        
      delay(1000);

    }
    

  case VACCUM_SENSOR:
     Serial.print ("OK -- ");
     Serial.println(  analogRead(VAC_SENSOR));
     break;
   
 case '4':
    while (1)
    {
      int t[MOTORS_DEFINED];
      
      Serial.print( "OK: ");
      for (i = 0; i < MOTORS_DEFINED; i ++)
      {
         t[i] = getAvgAnalog(i,motors[i].positionAnalogPin,false,motors[i].invertedPosition);        
      }
      Serial.println( "AV: ("+  String(t[0]) + ", "+  String(t[1]) + ", "+  String(t[2]) + ")");

      if (arg1 == 0)
        return 0;
      delay(1000);
    }
    return 0;

  case '5':
    while (1)
    {
      int t[MOTORS_DEFINED];
      
      Serial.print( "OK: ");
      for (i = 0; i < MOTORS_DEFINED; i ++)
      {
         t[i] = getAvgAnalog(i,motors[i].positionAnalogPin,true,motors[i].invertedPosition);        
      }
      Serial.println( "AV: ("+  String(t[0]) + ", "+  String(t[1]) + ", "+  String(t[2]) + ")");

      if (arg1 == 0)
        return 0;
      delay(1000);
    }
    return 0;

  case '6':
    switch (argv1[0]) {
      
   case 'l':

        endpt = arg2 ; //(arg2 & 0xff);
        servoWrite(0, endpt);
        break;
   
  case 'r':

        endpt = (arg2 & 0xff);
        servoWrite(1, endpt);
        break;
    
    
   case 'p':

        endpt = (arg2 & 0xff);
        servoWrite(2, endpt);
        break;
    
    case 'q':

        endpt = arg2 ; //(arg2 & 0xff);
        servoWrite(3, endpt);
        break;
    
     case 's':
      endpt = (arg2 & 0xff);
      servoWrite(2, endpt);
      servoWrite(3, 180-endpt+SERVO_FIX);
      break;
      
    default:
      Serial.println ("Err");
      return 0;
    }
    Serial.println ("OK");
    break;

  case '7':
    
      Serial.println( "OK - " + String(getAvgAnalog(0,servoAnalog[0],true,0)) + " " + String(getAvgAnalog(0,servoAnalog[1],true,0)));
      
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
          servoWrite(arg1, 0);
          delay(500);
          servoWrite(arg1, endpt);
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

  case RESET:
    setup();
    Serial.println ("OK");
    break;

  case SHORT_MOTOR_LIMIT:
    shortModeLimit = arg1;
    Serial.println ("OK");
    break;
    
  case DISTANCE:
   
   lastDistance = sensor.read(); //readRangeSingleMillimeters(true);
   Serial.println("OK " + String(lastDistance));
   return;

  
  case ATTACH_MODE:
  case DOOR_POS:
/*
  while (1){
    digitalWrite(ST1_STEP,1);
    digitalWrite(ST1_DIR,1);

    delay(1);
    digitalWrite(ST1_STEP,0);
    digitalWrite(ST1_DIR,0);
    delay(1);
  }
  */
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
        setMotorState(i,MOTOR_DISABLED);
        Serial.println("*OK");
      }
      break; 

  case ARM_LOCK:
      for (i=0; i < MOTORS_DEFINED;i++)
      {

        if ((motors[i].leaveEnabled) && (arg1 == 1))
        {
          setMotorState(i, MOTOR_HOLD);
        }
        else
        {
          setMotorState(i, MOTOR_DISABLED);
        } 
      }
      Serial.println("*OK");
      break;

  case ARM_PDN:
     for (i=0; i < MOTORS_DEFINED;i++)
      {
         jointPdn(i,arg1);
 
      }
      Serial.println("*OK");
      break;
  
  case SET_ANGLES:
    // set all three angles but only if all are idle

    firstBusyCall = 1;
    
    if ((motors[0].direction != MOTOR_IDLE) || (motors[1].direction != MOTOR_IDLE) || (motors[2].direction != MOTOR_IDLE))
    {
           Serial.println("ERR-BUSY");
           return 0;
    }

    // if all three values are zero then don't do anything.
    
    if ((arg1 == 0) && (arg2==0) && (arg3 ==0))
    {
      Serial.println("OK");
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

    if (moveJoint(0,arg1))
    {    
      lastSetAngles[0] = arg1;
    
      if (moveToPosition(0,abs(arg1),0) == 0)
      {
        Serial.println("*ERR 0");
        break;
      }
    }
    
    if (moveJoint(1,arg2))
    {
      lastSetAngles[1] = arg2;
    
       if (moveToPosition(1,abs(arg2),0) == 0)
      {
        Serial.println("*ERR 1");
        break;
      }
    }
    
    if (moveJoint(2,arg3))
    {
      lastSetAngles[2] = arg3;
    
       if (moveToPosition(2,abs(arg3),1) == 0)
      {
        Serial.println("*ERR 2");
        break;
      }
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
    
    if ((motors[0].direction == MOTOR_IDLE) && 
        (motors[1].direction == MOTOR_IDLE) && 
        (motors[2].direction == MOTOR_IDLE) &&
        (firstBusyCall == 0) )
    {
        v = 0;
    }
    else
       v = 1;

     firstBusyCall = 0;

    if (whichArm == RIGHT_ARM)
    {
      Serial.println(" " + String (v) + " " + String (errorCode) + " " + String(random(0, 255)) + " " + String(vacValveOn) + " " + String(attachModeOn) + " " + String(analogRead(VAC_SENSOR)) + " " + String(3) + " " + String(lastDistance) );   // return  vac valve, attach mode, vac sensor, attach status
    }else
    {
      Serial.println(" " + String (v) + " " + String (errorCode) + " " + String(basketOpen) );   // return just door position, for now a 0
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
     /* Was driving servo[2] and servo[3] through a 134-step sweep -- both out
        of range, see servoWrite().  Reports rather than remapping: the basket
        is a ganged pair, only servo[0] and servo[1] physically exist, and
        picking the wrong pair would sweep an unrelated mechanism through 134
        steps.  Confirm the mapping, then restore the sweep using servoWrite().

        Note this also blocked for ~6.7 s of delay(50) with the firmware
        completely unresponsive -- no status, no halt. */
     Serial.println("*ERR basket servo mapping unconfirmed");
     break;

  default:
    /* An unrecognised command used to fall straight off the end of this switch
       and produce no output at all.  The host's send_msg() then sat in ReadLine
       until it timed out -- 1000 ms of the shared controller_manager thread,
       every time, with nothing in any log to say why.

       That is how `v` (arm_set_vacuum_on) went unnoticed: the host has been
       sending a command this firmware has never implemented, silently, for as
       long as both have existed.

       Replying turns every future protocol mismatch into an immediate, visible
       failure instead.  Echoes the decimal value as well as the character so a
       non-printable byte is still identifiable. */
    Serial.print("ERR-CMD ");
    Serial.print(cmd);
    Serial.print(" (");
    Serial.print((int)cmd);
    Serial.println(")");
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
    uint8_t  width;
    uint8_t  height;
    
    
    Serial.begin(57600);
    Wire.begin();

    analogReference(EXTERNAL);
    analogRead(A0);
    analogRead(A0);
    analogRead(A0);
    
    pinMode(SOL1 ,OUTPUT);
    pinMode(SOL2 ,OUTPUT);

    debugMode = 0;
    
    motors = rightMotors;
    motorLimits = rightMotorLimits;
    whichArm = RIGHT_ARM;

    for (i = 0; i < MOTORS_DEFINED; i++)
    {
      pinMode(motors[i].stepGpio ,OUTPUT);
      pinMode(motors[i].enGpio ,OUTPUT);
      /* OUTPUT, not INPUT.  As an input, digitalWrite() on this pin only
         toggles the internal pullup, so startMotor()'s direction setting was
         a weak pullup one way and a floating pin the other -- direction was
         being decided by whatever external pulldown the driver board happens
         to have, if any. */
      pinMode(motors[i].dirGpio ,OUTPUT);
      pinMode(motors[i].uart_tx,OUTPUT);
      pinMode(motors[i].uart_rx,OUTPUT);

      digitalWrite(motors[i].enGpio,HIGH);   
      digitalWrite(motors[i].dirGpio,LOW);   
      digitalWrite(motors[i].uart_tx,HIGH);   
      digitalWrite(motors[i].uart_rx,HIGH);   
  
      motors[i].motorState = MOTOR_DISABLED;
      motors[i].direction = MOTOR_IDLE; 
      firstBusyCall = 0;
      motors[i].stopFlag = STOP_NONE;
      
      //motors[i].ticksPerAv = motors[i].ticksPerDegree/motors[i].avPerDegree;

    }
   
    for (i = 0; i < NUM_SERVOS;i++)
    {
     servo[i].attach(servoGpio[i]);
     servo[i].write(SERVO_FLAT);
    }


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
  errorCode = 99;
  
  Wire.setClock(400000); // use 400 kHz I2C
  sensor.setTimeout(500);
  
  //Serial.println(F(" booting VL53L1X"));
  
  if (!sensor.init(false))
  {
    // Serial.println("Failed to detect and initialize sensor!");
 }
 

  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(33000);
  sensor.startContinuous(100);
 
  sensor.setROISize(4,4);
 
  sensor.getROISize(&width,&height);

 // Serial.println("ROI " + String(width) + " " + String(height));
  
  
  
  Serial.print("* Arm Setup Done: Version " );
  Serial.print(String (version) + " ");
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

  int readPosition = millis() > timeToReadPosition;
  

  for (int m=0; m < MOTORS_DEFINED; m++)
  {
    if (motors[m].direction != MOTOR_IDLE)
    {
      // Update current position from analog feedback
          
      if(readPosition)
      {
        motors[m].currentAv = getAvgAnalog(m,motors[m].positionAnalogPin,false,motors[m].invertedPosition);
      
        // Rate-limited verbose debug (max 10 Hz)
        if (debugMode)

        {
        Serial.print("*AV M");
        Serial.print(m);
        Serial.print(" c:");
        Serial.print(motors[m].currentAv);
        Serial.print(" t:");
        Serial.println (motors[m].targetAv);
        //Serial.print(" time:");
        //Serial.println(millis());
        }
      }
      // see if stopped
      
      if (motors[m].stopFlag != STOP_NONE)
      {
          // mark it stopped
          
          motors[m].direction = MOTOR_IDLE;

          if (debugMode)
          {
            const char* reason;
            if (motors[m].stopFlag == STOP_OUT_OF_TICKS) reason = "[OUT_OF_TICKS]";
            else if (motors[m].stopFlag == STOP_LOW_AV_TARGET) reason = "[LOW_AV_TARGET]";
            else if (motors[m].stopFlag == STOP_HIGH_AV_TARGET) reason = "[AT TARGET]";
            else if (motors[m].stopFlag == STOP_NO_PROGRESS) reason = "[NO_PROGRESS/STALLED]";
            else reason = "[UNKNOWN]";

            Serial.print("*STOP M");
            Serial.print(m);
            Serial.print(" ");
            Serial.print(reason);
            Serial.print(" tks:");
            Serial.print(motors[m].ticksLeft);
            Serial.print(" cv:");
            Serial.print(motors[m].currentAv);
            Serial.print(" tv: ");
            Serial.println(motors[m].targetAv);
          }
      
          // see if we should leave motor enabled
          
          if (motors[m].leaveEnabled)
          {
            setMotorState(m, MOTOR_HOLD);
          }
          else
          {  
            setMotorState(m, MOTOR_DISABLED);
          }
        //  continue;
       }
    
      // since the motor is still moveing Check if making progress every ~100 loop iterations
      motors[m].progressCheckCounter++;
      
      if (motors[m].progressCheckCounter >= 200)
      {
        // Check if AV has changed by at least 2 units (accounting for noise)
        int avDelta = abs(motors[m].currentAv - motors[m].lastProgressCheckAv);

        if (avDelta < 2)
        {
          // Motor is not making progress - might be stalled or hitting obstacle
          if (0) { // {debugMode) {
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
    else
    {
      if (motors[m].leaveEnabled)
      {
          if ((millis() > motors[m].idleEnableTimeout) && (motors[m].motorState != MOTOR_DISABLED))
          {  
            if(debugMode)
              Serial.println("Time to Disable Motor " + String(m));
              
              // time to turn off 

              setMotorState(m,MOTOR_DISABLED);
          }         
      }  
    }
  }
  
  if (readPosition)
    timeToReadPosition = millis() + POSITION_READ_INTERVAL_MS;
    
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

  

  if (avReadTime  < millis())
  {
    avReadTime = millis() + 1000;
    switch (avReadMode)
    {
    case 0:
      Serial.println (getAvgAnalog(0,ST1_POS,true,0));
      break;
      
    case 1:
      Serial.println (getAvgAnalog(0,ST2_POS,true,0));
      break;
      
    case 2:
      Serial.println (getAvgAnalog(0,ST3_POS,true,0));
      break;

    default:
      break;
    }
  }
}  
