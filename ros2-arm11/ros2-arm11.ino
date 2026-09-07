#include <TMC2209.h>

#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <stdio.h>
#include <LibPrintf.h>
#include <VL53L1X.h>

#define VERSION " 2.12"

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
    unsigned long idleEnableOnTimeMs;
     
    char motorState;
    char direction;

    volatile int stopFlag;
    char pulseHigh;
    char speedMode;

    volatile int targetAv;
    volatile int currentAv;
    
    int moveTicks;
    int ticksLeft;
    int ticksPerformed;           // how many ticks so far
    int ticksFullSpeedTrigger;    // when to go to full speed
    int ticksSlowDownTrigger;     // when to slow down
       
    int ticksSkip;                // ticks left to skip (for exponential ramp)

    int progressCheckCounter;     // counter to check if motor is making progress
    int lastProgressCheckAv;      // last AV reading when progress was checked

    unsigned long idleEnableTimeout;
  } MOTOR;
 
typedef struct MOTOR_LIMITS{
  int lowerLimit;
  int upperLimit;
} MOTOR_LIMITS;


#define MOTORS_DEFINED 3

MOTOR leftMotors[MOTORS_DEFINED] = { {ST1_ENABLE,ST1_DIR,ST1_STEP,ST1_POS,ST1_UART_TX,ST1_UART_RX,19.0,3.08,90,188, 760, 0,1,1,30000,MOTOR_IDLE,0},
                                     {ST2_ENABLE,ST2_DIR,ST2_STEP,ST2_POS,ST2_UART_TX,ST2_UART_RX,15.9,3.08,90,211 ,770 ,1,1,1,30000,MOTOR_IDLE,0}, 
                                     {ST3_ENABLE,ST3_DIR,ST3_STEP,ST3_POS,ST3_UART_TX,ST3_UART_RX,15.0,3.08,90,250, 790, 0,1,1,30000,MOTOR_IDLE,0} };
                                     
MOTOR_LIMITS leftMotorLimits[MOTORS_DEFINED] = {  {90,270},{90,270},{90,270} }  ;

MOTOR rightMotors[MOTORS_DEFINED] = { {ST1_ENABLE,ST1_DIR,ST1_STEP,ST1_POS,ST1_UART_TX,ST1_UART_RX,19.0,3.08,90,220, 804 ,1,0,1,30000,MOTOR_IDLE,0},
                                      {ST2_ENABLE,ST2_DIR,ST2_STEP,ST2_POS,ST2_UART_TX,ST2_UART_RX,15.9,3.08,90,250, 808 ,0,0,1,30000,MOTOR_IDLE,0}, 
                                      {ST3_ENABLE,ST3_DIR,ST3_STEP,ST3_POS,ST3_UART_TX,ST3_UART_RX,15.9,3.08,90,217, 802, 1,0,1, 30000,MOTOR_IDLE,0}  };

MOTOR_LIMITS rightMotorLimits[MOTORS_DEFINED] = {  {90,270},{90,270},{90,270} }  ;

MOTOR *motors;
MOTOR_LIMITS *motorLimits;

// Exponential ramp parameters (replaces skipIntervalTable)
#define MIN_TICK_DELAY    0     // Full speed - no delay between ticks
#define MAX_TICK_DELAY    8     // Starting speed - delay this many ticks

/* Ramp length, in ticks: min(RAMP_TICKS_MAX, moveTicks/RAMP_TICKS_DIVISOR) is
   spent accelerating and the same decelerating.  Raise RAMP_TICKS_MAX if a
   heavily loaded joint still loses steps on the way up to speed. */
#define RAMP_TICKS_MAX      100
#define RAMP_TICKS_DIVISOR  4

volatile unsigned int tickA = 0;

#define LEFT_ARM  1
#define RIGHT_ARM 2
/* Arrival window, in AV counts, used by the timer ISR.  In 2.10 this was 8
   counts (~2.6 deg) while the feedback feeding it was 100-150 ms stale -- the
   motor was already 10-15 counts past target before the ISR noticed, so every
   move overshot and the host had to re-command.  With POSITION_READ_INTERVAL_MS
   down at 10 ms the feedback lags ~2-3 counts, so the window can be tightened.
   Do not raise this without also re-checking allowedJointOffset below: the two
   are tied together by jointDeadbandDegrees(). */
#define END_AV_THRESHOLD 5

char whichArm;

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

/* How often loop() samples the joint pots.  Was 60 ms, which together with the
   4-deep boxcar in getAvgAnalog() put ~100-150 ms of lag between the arm and
   the AV the ISR stops on -- more than the arrival window was wide.  At 10 ms
   the boxcar spans 40 ms and the lag is ~2-3 AV counts.
   Three analogRead() calls cost ~340 us per sample, i.e. ~3% of loop time. */
#define POSITION_READ_INTERVAL_MS 10
unsigned long timeToReadPosition = 0;


unsigned long avReadTime = 0;
char avReadMode = MOTORS_DEFINED+1;

#define MOTOR_ACC    0
#define MOTOR_STEADY 1
#define MOTOR_DEC    2
#define MOTOR_SHORT  3

#define MOTOR_ACTIVE     2
#define MOTOR_HOLD       1
#define MOTOR_DISABLED   0


/* Moves shorter than this skip the acceleration ramp and run at a flat rate.
   In 2.10 this was 9999, and since a full 90 deg swing is only ~1710 ticks that
   meant EVERY move was "short": speedMode was always MOTOR_SHORT, the
   MOTOR_ACC/STEADY/DEC machine never ran, and every move was a standing start
   at the full 500 steps/s.  That is the stutter.  0 = always ramp. */
int shortModeLimit = 0;

/* Smallest commanded change, in degrees, that is worth starting a move for.
   Set from END_AV_THRESHOLD in setup() via jointDeadbandDegrees() -- it used to
   be a hard-coded 4 deg against a 2.6 deg arrival window, so any correction
   between 2.6 and 4 deg was refused by moveJoint() and the arm could not
   converge.  Keeping it derived means the two can never drift apart again. */
int allowedJointOffset = 3;


/* How far past its estimated tick count a move may run while waiting for the
   pot to reach the target.  Was 1000, which at the old flat 500 steps/s was
   ~2 s of overrun.  Now that a move that outruns its estimate crawls at
   MAX_TICK_DELAY (~18 ms/tick), 1000 ticks would be 18 s of grinding against a
   jam; 150 keeps the overrun at ~2.7 s and is still ~9 deg of slack, far more
   than the tick estimate is ever wrong by. */
#define TICKS_PADDING 150

/* STEP pulse width, microseconds.  Was 300, which blocked the 2 ms timer ISR
   for up to 900 us with three motors stepping -- 45% of the period with
   interrupts disabled, enough to drop Timer0 ticks and drift millis().  The
   TMC2209 needs roughly 100 ns, so 5 us is still ~50x margin.  Raise it if
   steps are missed on long cabling. */
#define STEP_PULSE_US 5

#define MAX_TRAJ 1
#define NUMB_ANGLES 3

double trajectory[MAX_TRAJ][NUMB_ANGLES];
int trajCount = 0;
int trajIndex;

#define TRAJECTORY_IDLE 0
#define TRAJECTORY_ACTIVE 1

char trajectoryState = TRAJECTORY_IDLE;

int  basketOpen = 0;
int  vacValveOn = 0;
int attachModeOn = 0;
int fakeMode = 0;
int lastSetAngles[3];

/* A SET_ANGLES that arrives while a joint is still moving used to be answered
   with ERR-BUSY and thrown away.  ros2_control pushes a new setpoint on every
   write() cycle, so whichever update happened to land in an idle gap was the
   one that took effect and the rest were lost -- corrections arrived at
   effectively random times.  Hold the most recent one instead and apply it as
   soon as all three joints are idle. */
int pendingAngles[MOTORS_DEFINED];
char pendingAnglesValid = 0;
int debugMode = 0;
int errorCode = 0;

char firstBusyCall = 0;

VL53L1X sensor;


/* motorTimer() runs from a timer ISR and shares 16-bit fields with loop().
   On AVR an int store is two instructions, so an ISR landing between them sees
   a torn value -- e.g. currentAv stepping 511 -> 512 can be read as 767, which
   is enough to trip the arrival test at the wrong moment.  loop() must touch
   those fields through these; the ISR already runs with interrupts off. */
static inline void atomicSetInt(volatile int *p, int v)
{
  uint8_t s = SREG;
  cli();
  *p = v;
  SREG = s;
}

static inline int atomicGetInt(volatile int *p)
{
  uint8_t s = SREG;
  cli();
  int v = *p;
  SREG = s;
  return v;
}

/* The arrival window END_AV_THRESHOLD is in AV counts; moveJoint() works in
   degrees.  Convert, rounding up, so that a move refused as "too small" is
   exactly one the ISR would have declared arrived immediately.  Below this and
   the motor kicks and instantly stops; above it and the arm parks off target
   with no way to ask for the difference. */
int jointDeadbandDegrees(int motor)
{
  int d = (int)(END_AV_THRESHOLD / motors[motor].avPerDegree);

  if (d < 1)
    d = 1;
  else
    d++;

  return d;
}

void setMotorState(int m,int newState)
{
   if (motors[m].motorState == newState)
   {
      if(debugMode)   
      {
          Serial.print(F("*Motor lp "));
          Serial.print(m);
          Serial.print(F(" "));
          Serial.println(newState);
      }
     return;
   }
   
   switch (newState) {
    
   case MOTOR_ACTIVE:
      digitalWrite(motors[m].enGpio,LOW) ;
      digitalWrite(motors[m].uart_tx,HIGH) ;
      digitalWrite(motors[m].uart_rx,HIGH) ;
      if(debugMode)
      {
          Serial.print(F("*Motor "));
          Serial.print(m);
          Serial.println(F(" Active"));
      }
      motors[m].motorState= newState;
      return;

   case MOTOR_DISABLED:
      digitalWrite(motors[m].enGpio,HIGH) ;
      digitalWrite(motors[m].uart_tx,LOW) ;
      digitalWrite(motors[m].uart_rx,LOW) ;
       if(debugMode)
       {
          Serial.print(F("*Motor "));
          Serial.print(m);
          Serial.println(F(" Disabled"));
       }
      motors[m].motorState= newState;
      return;
   
   case MOTOR_HOLD:     
      digitalWrite(motors[m].enGpio,LOW) ;
      digitalWrite(motors[m].uart_tx,HIGH) ;
      digitalWrite(motors[m].uart_rx,HIGH) ;
      if(debugMode)
      {
          Serial.print(F("*Motor "));
          Serial.print(m);
          Serial.println(F(" Hold"));
      }
      
      motors[m].idleEnableTimeout = millis() + motors[m].idleEnableOnTimeMs;
      motors[m].motorState= newState;
      if(debugMode)
      {
          Serial.print(F("*Lock arm "));
          Serial.println(m);
      }
      return;
   }
}

void jointPdn(int m, int val)
{
      digitalWrite(motors[m].uart_tx,val) ;
      digitalWrite(motors[m].uart_rx,val) ;
      
      if(debugMode)
      {
          Serial.print(F("*pdn arm "));
          Serial.print(m);
          Serial.print(F(" "));
          Serial.println(val);
      }

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

/* Write cursor and fill level for the boxcar.  2.10 used a single int that was
   incremented on every read and never bounded: after 32767 reads it wrapped
   negative, "analogReadCount[motor] <= ANALOG_READS" became permanently true,
   and getAvgAnalog() silently stopped averaging for the rest of the session.
   A byte cursor wraps cleanly (256 % ANALOG_READS == 0) and the fill level
   saturates instead of counting. */
unsigned char analogReadIndex[MOTORS_DEFINED];
unsigned char analogReadFilled[MOTORS_DEFINED];

int getAvgAnalog(int motor,int port,int oneRead,int invert){

  int val = analogRead(port);
  int s,t;
  
  if (invert) 
    val = 1023-val;

  /* oneRead means "just tell me what the pin reads" -- it must not disturb the
     filter.  2.10 stored the sample first and checked oneRead second, so the
     debug commands ('4', '5', '7', '9') and the servo-analog reads, which all
     pass motor 0 whatever pin they are really reading, were injecting foreign
     samples into motor 0's control filter. */
  if (oneRead)
    return val;

  lastAnalogReads[motor][analogReadIndex[motor]] = val;
  analogReadIndex[motor] = (analogReadIndex[motor] + 1) & (ANALOG_READS-1);

  if (analogReadFilled[motor] < ANALOG_READS)
  {
    analogReadFilled[motor]++;

    // not enough history to average yet
    if (analogReadFilled[motor] < ANALOG_READS)
      return val;
  }

  s = 0;
  for (int i=0; i < ANALOG_READS; i++)
  {
    s += lastAnalogReads[motor][i];
  }

  t = s/ANALOG_READS;

  return t;

}


/* Take one filtered sample for a joint and publish it to the field the ISR
   stops on.  loop() calls this for every motor on a fixed POSITION_READ_INTERVAL_MS
   cadence whether or not the motor is moving, so the filter has a constant time
   base and currentAv is already fresh when a move starts. */
void sampleMotorAv(int motor)
{
  int val = getAvgAnalog(motor,motors[motor].positionAnalogPin,false,motors[motor].invertedPosition);

  atomicSetInt(&motors[motor].currentAv, val);
}


/* Report a joint angle.  This deliberately does NOT read the ADC: it returns
   the value sampleMotorAv() last published.  In 2.10 this took its own filtered
   read, and because ros2_control polls GET_ANGLES on every read() cycle that
   meant host traffic was injecting samples into the same 4-deep boxcar the
   control loop stops on -- the filter's effective time base, and so the
   overshoot, depended on how fast an unrelated ROS node happened to be
   polling. */
int getCurrentPosition(int motor )
{
      int currentPosition;
      int av = atomicGetInt(&motors[motor].currentAv);

        currentPosition = ((float(av - motors[motor].avAtMinDegree) ) / motors[motor].avPerDegree) + motors[motor].minDegree;

      if(debugMode) {
        Serial.print(F("*$ CPos M"));
        Serial.print(motor);
        Serial.print(F(" rev:"));
        Serial.print(motors[motor].invertedPosition);
        Serial.print(F(" av:"));
        Serial.print(av);
        Serial.print(F(" avMin:"));
        Serial.print(motors[motor].avAtMinDegree);
        Serial.print(F(" pos:"));
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

        /* Out of estimated ticks but the pot says we are not there yet -- the
           tick count is only an estimate and TICKS_PADDING lets the move run
           past it.  Crawl at the slowest rate until the AV target is met.  2.10
           returned near-full speed here, so a move that undershot its estimate
           decelerated and then snapped back up to 500 steps/s. */
        if (remaining <= 0)
           return MAX_TICK_DELAY;

        /* Deceleration zone size.  This used moveTicks, which moveToPosition()
           sets to a SIGNED tick count -- negative for one direction -- so the
           whole decel ramp silently collapsed to full speed on half of all
           moves.  startMotor() now stores the absolute count it was given. */
        int totalDecelSteps = motors[motor].moveTicks - motors[motor].ticksSlowDownTrigger;
        if (totalDecelSteps <= 0)
            return MIN_TICK_DELAY;

        /* Delay must GROW as the move runs out, so measure how far into the
           decel zone we are rather than how much is left.  The original
           expression was (remaining/total)^2, which is the acceleration curve:
           it went slow at the top of the zone and returned to FULL SPEED for
           the last few steps, i.e. every move ended with a full-speed slam
           into its stopping point.  The comment above it described the
           behaviour intended here, not the behaviour it had. */
        if (remaining > totalDecelSteps)
            remaining = totalDecelSteps;

        long travelled  = (long)(totalDecelSteps - remaining);
        long numerator  = travelled * travelled;
        long denominator = (long)totalDecelSteps * totalDecelSteps;
        delay = (MAX_TICK_DELAY * numerator) / denominator;

        /* No Serial here: this runs from the timer ISR. */
        return delay;
    }

    // MOTOR_STEADY - full speed, no delay
    return MIN_TICK_DELAY;
}



void motorTimer() {
  int m;
  int t;
  
  tickA++;

  if (tickA >= 500)   // 500 x 2 ms = 1 s heartbeat
  {
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

      t = abs(motors[m].currentAv - motors[m].targetAv);

      if (t < END_AV_THRESHOLD)
      {
        motors[m].stopFlag = STOP_HIGH_AV_TARGET;
        /* No Serial here: this is an ISR.  loop() already prints the stop with
           the same information (see the "*STOP M" block). */
        continue;
      }
      
      /* Calculate exponential delay and see if we should skip this tick.
         The call here was commented out in 2.10, so ticksSkip was pinned at 0
         and calculateExponentialDelay() was never reached from anywhere: every
         move ran flat out at one step per 2 ms tick (500 steps/s) from a
         standing start.  A skip count of N gives one step every N+1 ticks, so
         MAX_TICK_DELAY 8 starts the ramp at ~55 steps/s. */
      if (motors[m].ticksSkip == 0)
      {
        motors[m].ticksSkip = calculateExponentialDelay(m);
      }
      else
      {
        // Skip this tick, decrement counter
        motors[m].ticksSkip--;
        continue;
      }

      // do we have any ticks left?  (<=, not ==: exact equality could be stepped past)
      if ((motors[m].ticksLeft + TICKS_PADDING) <= 0) {
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
      motors[m].ticksPerformed++; 

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



int moveToPosition(int motor,int newPosition)
{
    int deltaAv;
    int newTargetAv, currentAvNow;
    float degreesFromMin;

        if(debugMode)
        {
          Serial.print(F("*mtp: "));
          Serial.print(motor);
          Serial.print(F(" "));
          Serial.println(newPosition);
        }

        if (newPosition > motorLimits[motor].upperLimit )
        {
          Serial.print(F("*ERR - too high: "));
          Serial.print(motor);
          Serial.print(F(" "));
          Serial.print(newPosition);
          Serial.print(F(" "));
          Serial.println(motorLimits[motor].upperLimit);
          newPosition = motorLimits[motor].upperLimit-4 ;
        }
        
        if (newPosition < motorLimits[motor].lowerLimit )
        {
          Serial.print(F("*ERR - too low: "));
          Serial.print(motor);
          Serial.print(F(" "));
          Serial.print(newPosition);
          Serial.print(F(" "));
          Serial.println(motorLimits[motor].lowerLimit);
          newPosition = motorLimits[motor].lowerLimit +4 ;
        }

        degreesFromMin = newPosition - motors[motor].minDegree;
       
        if(debugMode)
        {
          Serial.print(F("   *tn: "));
          Serial.print(degreesFromMin);
          Serial.print(F(" "));
          Serial.print(motors[motor].minDegree);
          Serial.print(F(" "));
          Serial.println(motors[motor].avPerDegree);
        }
        
        /* Work out the target on locals, then publish it in one atomic store.
           The ISR compares currentAv against targetAv on every 2 ms tick, and a
           half-written 16-bit target is a target it can spuriously match. */
        newTargetAv = float(degreesFromMin) * motors[motor].avPerDegree + motors[motor].avAtMinDegree;

        // loop() keeps this fresh on a fixed cadence; no need to disturb the filter here
        currentAvNow = atomicGetInt(&motors[motor].currentAv);

        // if we are going from a higher av to a lower one, make sure the target is not too low
        
        if ((currentAvNow > newTargetAv) && ( newTargetAv < motors[motor].avAtMinDegree))
        {
          if (debugMode)
          {
            Serial.print(F("*targetAv below min "));
            Serial.println(newTargetAv);
          }
          
          newTargetAv = motors[motor].avAtMinDegree;
        }
        else
        {
          // if we are going from a lower av to a higher one make sure the target is not too high
          if ((currentAvNow < newTargetAv) && ( newTargetAv > motors[motor].avAtMaxDegree))
          {
            if (debugMode)
            {
              Serial.print(F("*targetAv above max "));
              Serial.print(newTargetAv);
              Serial.print(F(" "));
              Serial.println(motors[motor].avAtMaxDegree);
            }
            newTargetAv = motors[motor].avAtMaxDegree;
          }
        }

        atomicSetInt(&motors[motor].targetAv, newTargetAv);

        // see how much change av we have to make. it can be positive or negative
        
        deltaAv = newTargetAv - currentAvNow;
        
        motors[motor].moveTicks = (deltaAv / motors[motor].avPerDegree) * motors[motor].ticksPerDegree ;
        
        if(debugMode)
        {
          Serial.print(F("   *tgt aV: "));
          Serial.print(newTargetAv);
          Serial.print(F(" cur aV "));
          Serial.print(currentAvNow);
          Serial.print(F(" aV delta: "));
          Serial.print(deltaAv);
          Serial.print(F(" ticks: "));
          Serial.print(motors[motor].moveTicks);
          Serial.print(F(" dir "));
          Serial.println(motors[motor].invertedPosition);
        }
 
        if(debugMode)
        {
            Serial.print(F("   *mt "));
            Serial.println(motors[motor].moveTicks);
        }

        /* No noInterrupts() around this any more.  startMotor() guards the
           fields the ISR shares, and it prints under debugMode -- at 57600 baud
           a debug line inside a critical section holds interrupts off for ~10 ms
           and drops five step ticks, so turning on debugging used to produce the
           very stutter one would be debugging. */
        /* Which way to turn for a rising AV depends on how the joint is
           geared; the ISR's stop test is |currentAv - targetAv| and does not
           care about direction, which is why the old haltTrigger/HALT_* flags
           had no reader left. */
        if ((motors[motor].moveTicks >= 0) == (motors[motor].invertedRotation == 0))
          startMotor(motor, ROT_CCW, aabs(motors[motor].moveTicks));
        else
          startMotor(motor, ROT_CW,  aabs(motors[motor].moveTicks));

  return 1;
}


void startMotor(int motor, int cmd, int ticks)
{
        int newDirection = motors[motor].direction;
        int newTicks     = (ticks == 0 ? 2 : ticks);
        int ramp         = min(RAMP_TICKS_MAX, newTicks/RAMP_TICKS_DIVISOR);
        int newSpeedMode = (ticks < shortModeLimit ? MOTOR_SHORT : MOTOR_ACC);

        if(debugMode)
        {
          Serial.print(F("*SM "));
          Serial.print(cmd, HEX);
          Serial.print(F(" "));
          Serial.print(motor);
          Serial.print(F(" "));
          Serial.println(ticks);
        }
      
        if (cmd == ROT_CW)
        {
         if(debugMode)
         {
            Serial.print(F("*Going CW "));
            Serial.println(ticks);
         }
            
          newDirection = MOTOR_CW;
          digitalWrite(motors[motor].dirGpio,LOW) ; //(motors[motor].invertedPosition ? HIGH: LOW));
        }
        else
        if (cmd == ROT_CCW)
        {  
          if(debugMode)
          {
            Serial.print(F("*Going CCW "));
            Serial.println(ticks);
          }
          
          newDirection = MOTOR_CCW;
          digitalWrite(motors[motor].dirGpio,HIGH ); // (motors[motor].invertedPosition ? LOW: HIGH));
          
         }
         else {
          // Serial.println("*ERR - cmd");
         }

        if (debugMode) {
           Serial.print(F("*StartMotor "));
           Serial.print(motor);
           Serial.print(F(" ticks:"));
           Serial.print(newTicks);
           Serial.print(F(" accel:"));
           Serial.print(ramp);
           Serial.print(F(" decel@:"));
           Serial.println(newTicks - ramp);
        }

        motors[motor].pulseHigh = 0;
        motors[motor].progressCheckCounter = 0;
        motors[motor].lastProgressCheckAv = atomicGetInt(&motors[motor].currentAv);

        /* The boxcar is NOT reset here any more.  loop() samples every joint on
           a fixed cadence whether it is moving or not, so the history is
           already valid and current; clearing it just threw away the filter and
           forced the first reads back to raw single samples. */

        setMotorState(motor,MOTOR_ACTIVE);

        /* Commit the whole move to the ISR at once, with direction LAST --
           direction is what arms motorTimer().  2.10 set direction first and
           then filled in ticksLeft, stopFlag and the ramp state, so a timer tick
           landing in that window stepped the motor using the PREVIOUS move's
           counters and speed mode: a short burst at the wrong rate, at the exact
           moment the ramp is supposed to be starting from a crawl.
           No Serial in here -- a debug line at 57600 baud would hold interrupts
           off for ~10 ms and drop five step ticks. */
        noInterrupts();

        motors[motor].stopFlag             = STOP_NONE;
        motors[motor].ticksLeft            = newTicks;
        motors[motor].moveTicks            = newTicks;
        motors[motor].ticksPerformed       = 0;
        motors[motor].ticksFullSpeedTrigger = ramp;
        motors[motor].ticksSlowDownTrigger  = newTicks - ramp;
        motors[motor].speedMode            = newSpeedMode;

        /* Start the ramp at its slowest step, not at full speed: ticksSkip is
           what the ISR counts down before the first pulse. */
        motors[motor].ticksSkip            = calculateExponentialDelay(motor);

        motors[motor].direction            = newDirection;

        interrupts();
}

   
int moveJoint(int joint,int arg)
{
   int t;
   
   
    t = abs(abs(arg)-getCurrentPosition(joint));
    
    if ((t < allowedJointOffset) || (arg == 0))
    {
      if (debugMode)
      {
        Serial.print(F("*m"));
        Serial.print(joint);
        Serial.print(F(" same angle, close, no move req "));
        Serial.println(t);
      }
 
 
      return 0;
    }

     if (debugMode)
     {
       Serial.print(F("*m"));
       Serial.print(joint);
       Serial.print(F(" same angle, move req "));
       Serial.println(t);
     }
 

    return 1;
}
        

int armIsBusy()
{
  for (int m = 0; m < MOTORS_DEFINED; m++)
  {
    if (motors[m].direction != MOTOR_IDLE)
      return 1;
  }

  return 0;
}


/* Start a move toward (a0,a1,a2).  Split out of SET_ANGLES so that a setpoint
   held while the arm was busy can be applied later from loop() by the same
   path.  Returns 0 on a per-joint range failure. */
int applySetAngles(int a0, int a1, int a2)
{
  int ok = 1;

  if (moveJoint(0,a0))
  {
    lastSetAngles[0] = a0;

    if (moveToPosition(0,abs(a0)) == 0)
      ok = 0;
  }

  if (moveJoint(1,a1))
  {
    lastSetAngles[1] = a1;

    if (moveToPosition(1,abs(a1)) == 0)
      ok = 0;
  }

  if (moveJoint(2,a2))
  {
    lastSetAngles[2] = a2;

    if (moveToPosition(2,abs(a2)) == 0)
      ok = 0;
  }

  return ok;
}


int runCommand() {
  int i = 0;
  int m,v;
  //char *p = argv1;
  //char *str;
  int rept, endpt,ticks;
  int currentPosition[3];
  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);
  
    switch(cmd) {

  case '1':   // solenoid tests
    switch (arg1){
    case 1:
      digitalWrite(SOL1,(arg2 == 1 ? 1 : 0));
      break;

    case 2:
      digitalWrite(SOL2,(arg2 == 1 ? 1 : 0));
      break;
    
 
    default:
      Serial.println(F("ERR"));
      return 0;
    }
    Serial.println(F("OK"));
    break;

  case '2':   // manual direction-pin poke.  Was "case 2:" -- raw 0x02, which the
              // input sanitiser in loop() discards, so it could never be reached.
    if (arg1 < MOTORS_DEFINED)
    {
      digitalWrite(motors[arg1].dirGpio,arg2);
      Serial.println(F("OK"));
    }
    else
      Serial.println(F("ERR"));

    break;

  case '3':  // vac sensor
    while (1)
    {
    
      Serial.println(analogRead(VAC_SENSOR_BU));
      if (arg1 == 0)
        return 0;
        
      delay(1000);

    }
    

  case VACCUM_SENSOR:
     Serial.print(F("OK -- "));
     Serial.println(  analogRead(VAC_SENSOR));
     break;
   
 case '4':
    while (1)
    {
      int t[MOTORS_DEFINED];
      
      Serial.print(F("OK: "));
      for (i = 0; i < MOTORS_DEFINED; i ++)
      {
         t[i] = getAvgAnalog(i,motors[i].positionAnalogPin,false,motors[i].invertedPosition);        
      }
      Serial.print(F("AV: ("));
      Serial.print(t[0]);
      Serial.print(F(", "));
      Serial.print(t[1]);
      Serial.print(F(", "));
      Serial.print(t[2]);
      Serial.println(F(")"));

      if (arg1 == 0)
        return 0;
      delay(1000);
    }
    return 0;

  case '5':
    while (1)
    {
      int t[MOTORS_DEFINED];
      
      Serial.print(F("OK: "));
      for (i = 0; i < MOTORS_DEFINED; i ++)
      {
         t[i] = getAvgAnalog(i,motors[i].positionAnalogPin,true,motors[i].invertedPosition);        
      }
      Serial.print(F("AV: ("));
      Serial.print(t[0]);
      Serial.print(F(", "));
      Serial.print(t[1]);
      Serial.print(F(", "));
      Serial.print(t[2]);
      Serial.println(F(")"));

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
      Serial.println(F("Err"));
      return 0;
    }
    Serial.println(F("OK"));
    break;

  case '7':
    
      Serial.print(F("OK - "));
      Serial.print(getAvgAnalog(0,servoAnalog[0],true,0));
      Serial.print(F(" "));
      Serial.println(getAvgAnalog(0,servoAnalog[1],true,0));
      
    break;
  
  case '8':    // sterpper test   6 servo# endpt repeat

   moveToPosition(arg1,arg2);
   Serial.println(F("OK"));
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
        Serial.println(F("OK"));
     }
     else
        Serial.println(F("Err"));
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
    Serial.println(F("OK"));
    break;

  case DBUG:
    debugMode = arg1;
    Serial.println(F("OK"));
    break;
  
  case FAKE_MODE:
    fakeMode = arg1;
    Serial.println(F("OK"));
    break;

  case RESET:
    setup();
    Serial.println(F("OK"));
    break;

  case SHORT_MOTOR_LIMIT:
    shortModeLimit = arg1;
    Serial.println(F("OK"));
    break;
    
  case DISTANCE:
   
   lastDistance = sensor.read(); //readRangeSingleMillimeters(true);
   Serial.print(F("OK "));
   Serial.println(lastDistance);
   return 0;

  
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
    Serial.println(F("OK"));
    break;

  case ARM_SELECT:
  
    if (whichArm != RIGHT_ARM)
    {
      Serial.println(F("ERR"));
      return 0;
    }

    if (argv1[0] == 'l')
    {
      motors = leftMotors;
      motorLimits = leftMotorLimits;
      whichArm = LEFT_ARM;
    }
    Serial.println(F("*OK "));
    break;
 
  case ROT_CCW_DEGREES:
  case ROT_CW_DEGREES:
  
    trajectoryState = TRAJECTORY_IDLE;
    if (arg1 < MOTORS_DEFINED)
    {
        ticks = int(motors[arg1].ticksPerDegree * arg2);
        
        cmd = (cmd == ROT_CCW_DEGREES ? 'w' : 'c');

        Serial.println(ticks);
        startMotor(arg1, cmd, ticks);
        Serial.println(F("*OK "));
    }
    else
    {
      Serial.println(F("*ERR"));
    }
    break;

  case ROT_CCW:
  case ROT_CW:
      
    trajectoryState = TRAJECTORY_IDLE;
    
       
      if (arg1 < MOTORS_DEFINED)
      {
        motors[arg1].targetAv = 0xffff;
        startMotor(arg1, cmd, arg2);
        Serial.println(F("*OK "));
      }
      else
        Serial.println(F("*ERR"));
      break;

  case ROT_HALT:
      /* Drop any setpoint we were holding: a halt means stop, not "stop and
         then resume into the move the host last asked for". */
      pendingAnglesValid = 0;

      for (i = 0; i < MOTORS_DEFINED; i++)
      {
        noInterrupts();
        motors[i].stopFlag = STOP_OUT_OF_TICKS;
        motors[i].ticksLeft = 0;
        motors[i].direction = MOTOR_IDLE;
        interrupts();

        motors[i].pulseHigh = 0;
        setMotorState(i,MOTOR_DISABLED);
      }
      Serial.println(F("*OK"));
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
      Serial.println(F("*OK"));
      break;

  case ARM_PDN:
     for (i=0; i < MOTORS_DEFINED;i++)
      {
         jointPdn(i,arg1);
 
      }
      Serial.println(F("*OK"));
      break;
  
  case SET_ANGLES:
    // set all three angles

    firstBusyCall = 1;
    
    // if all three values are zero then don't do anything.
    
    if ((arg1 == 0) && (arg2==0) && (arg3 ==0))
    {
      Serial.println(F("OK"));
      return 0;
    }
 
    if (debugMode){
      Serial.print(F("*set target:  ("));
      Serial.print(arg1);
      Serial.print(',');
      Serial.print(arg2);
      Serial.print(',');
      Serial.print(arg3);
      Serial.println(')');
      Serial.print(F("*    current: ("));
      Serial.print(getCurrentPosition(0));
      Serial.print(',');
      Serial.print(getCurrentPosition(1));
      Serial.print(',');
      Serial.print(getCurrentPosition(2));
      Serial.println(')');
    }
    
    if (fakeMode)
    {
      lastSetAngles[0] =  arg1;
      lastSetAngles[1] =  arg2;
      lastSetAngles[2] =  arg3;
      Serial.println(F("*OK"));
      return 0;
    }

    /* Still moving: remember this setpoint rather than dropping it, and let
       loop() start it the moment the arm goes idle.  Only the newest is kept --
       an older one is stale by definition.  Still one line of reply, which is
       all the host reads. */
    if (armIsBusy())
    {
      pendingAngles[0] = arg1;
      pendingAngles[1] = arg2;
      pendingAngles[2] = arg3;
      pendingAnglesValid = 1;

      Serial.println(F("*OK-QUEUED"));
      return 0;
    }

    pendingAnglesValid = 0;

    /* Exactly one line of reply either way: the host does a blocking ReadLine
       for each command it sends. */
    if (applySetAngles(arg1, arg2, arg3))
      Serial.println(F("*OK"));
    else
      Serial.println(F("*ERR"));

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

    Serial.print(currentPosition[0]);
    Serial.print(' ');
    Serial.print(currentPosition[1]);
    Serial.print(' ');
    Serial.print(currentPosition[2]);
    
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
      // right arm: 8 more fields, for 11 in total -- see read_arm_info()'s sscanf
      Serial.print(F(" "));  Serial.print(v);
      Serial.print(F(" "));  Serial.print(errorCode);
      Serial.print(F(" "));  Serial.print(random(0, 255));
      Serial.print(F(" "));  Serial.print(vacValveOn);
      Serial.print(F(" "));  Serial.print(attachModeOn);
      Serial.print(F(" "));  Serial.print(analogRead(VAC_SENSOR));
      Serial.print(F(" "));  Serial.print(3);              // attach status, hardcoded
      Serial.print(F(" "));  Serial.println(lastDistance);
    }else
    {
      // left arm: 3 more fields, for 6 in total
      Serial.print(F(" "));  Serial.print(v);
      Serial.print(F(" "));  Serial.print(errorCode);
      Serial.print(F(" "));  Serial.println(basketOpen);   // door position, for now a 0
    }
    
    break;

  case POINT:
    trajectoryState = TRAJECTORY_IDLE;
    if (argv1[0] == 'g')
    {
      trajIndex = 0;
          
      trajectoryState = TRAJECTORY_ACTIVE;
      for (m = 0; m < MOTORS_DEFINED-1; m++)
        moveToPosition(m, trajectory[0][m]);
    }  
    else if (argv1[0] == 'l')
      {
        trajIndex = 0;
        Serial.println(trajCount);

        for (m = 0; m < trajCount; m++)
        {
          Serial.print(F("("));
          Serial.print(trajectory[m][0]);
          Serial.print(F(","));
          Serial.print(trajectory[m][1]);
          Serial.print(F("*,"));
          Serial.print(trajectory[m][2]);
          Serial.println(F(")"));
        }                
      } 
 
    Serial.println(F("*OK"));
    break;

   case BASKET_OPEN:
     /* Was driving servo[2] and servo[3] through a 134-step sweep -- both out
        of range, see servoWrite().  Reports rather than remapping: the basket
        is a ganged pair, only servo[0] and servo[1] physically exist, and
        picking the wrong pair would sweep an unrelated mechanism through 134
        steps.  Confirm the mapping, then restore the sweep using servoWrite().

        Note this also blocked for ~6.7 s of delay(50) with the firmware
        completely unresponsive -- no status, no halt. */
     Serial.println(F("*ERR basket servo mapping unconfirmed"));
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
    Serial.print(F("ERR-CMD "));
    Serial.print(cmd);
    Serial.print(F(" ("));
    Serial.print((int)cmd);
    Serial.println(F(")"));
    break;
  }

  return 1;
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

    /* The heartbeat toggle in motorTimer() drives this.  Without the pinMode it
       was an input and digitalWrite() only flipped the internal pullup. */
    pinMode(13, OUTPUT);

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
      motors[i].ticksSkip = 0;

      // prime the boxcar so the first move works from a settled reading
      analogReadIndex[i] = 0;
      analogReadFilled[i] = 0;

      analogRead(motors[i].positionAnalogPin);   // let the ADC mux settle

      for (int k = 0; k < ANALOG_READS; k++)
        sampleMotorAv(i);
      
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

  /* Tie the "is this move worth making" threshold to the window the ISR
     actually stops in, instead of hard-coding a number that drifts away from
     it.  Take the widest of the three so no joint is asked for a correction
     finer than it can resolve. */
  allowedJointOffset = 1;

  for (i = 0; i < MOTORS_DEFINED; i++)
  {
    int d = jointDeadbandDegrees(i);

    if (d > allowedJointOffset)
      allowedJointOffset = d;
  }

  pendingAnglesValid = 0;

  basketOpen = 0;
  vacValveOn = 0;
  attachModeOn = 0;
  errorCode = 0;
  
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
  
  
  
  Serial.println(F("* Arm Setup Done: Version " VERSION " " __DATE__ " at " __TIME__));
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
          Serial.println(F("*ERR: argv1 overflow"));
        resetCommand();
      }
    }
    else if (arg == 2) {
      if (idx < ARG_BUFFER_SIZE - 1) {
        argv2[idx] = chr;
        idx++;
      } else {
        if (debugMode)
          Serial.println(F("*ERR: argv2 overflow"));
        resetCommand();
      }
    }
    else if (arg == 3) {
      if (idx < ARG_BUFFER_SIZE - 1) {
        argv3[idx] = chr;
        idx++;
      } else {
        if (debugMode)
          Serial.println(F("*ERR: argv3 overflow"));
        resetCommand();
      }
    }
    else if (arg == 4) {
      if (idx < ARG_BUFFER_SIZE - 1) {
        argv4[idx] = chr;
        idx++;
      } else {
        if (debugMode)
          Serial.println(F("*ERR: argv4 overflow"));
        resetCommand();
      }
    }
  }

  /* millis() > x wraps after 49 days and stalls sampling; the subtraction form
     is rollover-safe. */
  int readPosition = (long)(millis() - timeToReadPosition) >= 0;
  

  for (int m=0; m < MOTORS_DEFINED; m++)
  {
    /* Sample every joint on the fixed cadence, moving or not.  2.10 only
       sampled while a joint was moving, so the first thing a move did was work
       from whatever the pot read before the joint was last disabled. */
    if (readPosition)
      sampleMotorAv(m);

    if (motors[m].direction != MOTOR_IDLE)
    {
      if (readPosition && debugMode)
      {
        Serial.print(F("*AV M"));
        Serial.print(m);
        Serial.print(F(" c:"));
        Serial.print(atomicGetInt(&motors[m].currentAv));
        Serial.print(F(" t:"));
        Serial.println (atomicGetInt(&motors[m].targetAv));
      }

      // see if stopped
      
      if (atomicGetInt(&motors[m].stopFlag) != STOP_NONE)
      {
          // mark it stopped
          
          motors[m].direction = MOTOR_IDLE;

          if (debugMode)
          {
            const __FlashStringHelper* reason;
            if (motors[m].stopFlag == STOP_OUT_OF_TICKS) reason = F("[OUT_OF_TICKS]");
            else if (motors[m].stopFlag == STOP_LOW_AV_TARGET) reason = F("[LOW_AV_TARGET]");
            else if (motors[m].stopFlag == STOP_HIGH_AV_TARGET) reason = F("[AT TARGET]");
            else if (motors[m].stopFlag == STOP_NO_PROGRESS) reason = F("[NO_PROGRESS/STALLED]");
            else reason = F("[UNKNOWN]");

            Serial.print(F("*STOP M"));
            Serial.print(m);
            Serial.print(F(" "));
            Serial.print(reason);
            Serial.print(F(" tks:"));
            Serial.print(atomicGetInt(&motors[m].ticksLeft));
            Serial.print(F(" cv:"));
            Serial.print(atomicGetInt(&motors[m].currentAv));
            Serial.print(F(" tv: "));
            Serial.println(atomicGetInt(&motors[m].targetAv));
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
        int avNow = atomicGetInt(&motors[m].currentAv);

        // Check if AV has changed by at least 2 units (accounting for noise)
        int avDelta = abs(avNow - motors[m].lastProgressCheckAv);

        if (avDelta < 2)
        {
          // Motor is not making progress - might be stalled or hitting obstacle
          if (0) { // {debugMode) {
            Serial.print(F("*WARNING M"));
            Serial.print(m);
            Serial.print(F(" NO PROGRESS av:"));
            Serial.print(avNow);
            Serial.print(F(" last:"));
            Serial.print(motors[m].lastProgressCheckAv);
            Serial.print(F(" delta:"));
            Serial.print(avDelta);
            Serial.print(F(" ticks:"));
            Serial.println(motors[m].ticksPerformed);
          }

          // Could optionally stop the motor here:
          // motors[m].stopFlag = STOP_NO_PROGRESS;
        }
        else if (debugMode >= 3)
        {
          Serial.print(F("*PROGRESS M"));
          Serial.print(m);
          Serial.print(F(" OK delta:"));
          Serial.println(avDelta);
        }

        // Reset for next check
        motors[m].progressCheckCounter = 0;
        motors[m].lastProgressCheckAv = avNow;
      }
    }
    else
    {
      if (motors[m].leaveEnabled)
      {
          if (((long)(millis() - motors[m].idleEnableTimeout) >= 0) && (motors[m].motorState != MOTOR_DISABLED))
          {  
            if(debugMode)
            {
              Serial.print(F("*Time to Disable Motor "));
              Serial.println(m);
            }
              
              // time to turn off 

              setMotorState(m,MOTOR_DISABLED);
          }         
      }  
    }
  }
  
  if (readPosition)
    timeToReadPosition = millis() + POSITION_READ_INTERVAL_MS;

  /* A setpoint that arrived mid-move: start it now that the arm is idle. */
  if (pendingAnglesValid && !armIsBusy())
  {
    pendingAnglesValid = 0;
    applySetAngles(pendingAngles[0], pendingAngles[1], pendingAngles[2]);
  }
    
  if (trajectoryState == TRAJECTORY_ACTIVE)
  {
    // we are doing point move, we go on to the next point when all three motors are stopped

    if (motors[0].stopFlag && motors[1].stopFlag ) // && motors[2].stopFlag )
    { 
       Serial.print(F("step complete "));

       // move on to the next point if there is one
  
      trajIndex++;
      
      if (trajIndex < trajCount)
       {
          // got another point
         Serial.print(F("next point "));
         Serial.print(trajIndex);
         Serial.print(F(" ("));
         Serial.print(trajectory[trajIndex][0]);
         Serial.print(F(","));
         Serial.print(trajectory[trajIndex][1]);
         Serial.print(F(","));
         Serial.print(trajectory[trajIndex][2]);
         Serial.println(F(")"));

          for (int m = 0; m < MOTORS_DEFINED-1; m++)
            moveToPosition(m, trajectory[trajIndex][m]);
       }
       else
       {
        int currentPosition[3];
         trajectoryState = TRAJECTORY_IDLE;
         Serial.println(F("*Movement Complete"));
         for (int  i = 0; i < 3; i++)
         {
             currentPosition[i] = getCurrentPosition(i);
             Serial.println(currentPosition[i]);
         }

       }
    }
  }

  

  if ((long)(millis() - avReadTime) >= 0)
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
