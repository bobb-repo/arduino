/*********************************************************************
 *  ROSArduinoBridge

   A set of simple serial commands to control a differential drive
   robot and receive back sensor and odometry data. Default
   configuration assumes use of an Arduino Mega + Pololu motor
   controller shield + Robogaia Mega Encoder shield.  Edit the
   readEncoder() and setMotorSpeed() wrapper functions if using
   different motor controller or encoder method.

   Created for the Pi Robot Project: http://www.pirobot.org
   and the Home Brew Robotics Club (HBRC): http://hbrobotics.org

   Authors: Patrick Goebel, James Nugen

   Inspired and modeled after the ArbotiX driver by Michael Ferguson

   Software License Agreement (BSD License)

   Copyright (c) 2012, Patrick Goebel.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*********************************************************************
 *  motor10 -- changes from motor9.  NONE OF THIS IS HARDWARE TESTED.
 *
 *  - Removed the Serial.print() calls from both encoder ISRs.  They
 *    injected stray 'l'/'r' characters into the same stream the host
 *    parses, and the host's atoi() returns 0 on a leading non-digit,
 *    so encoder reads could silently come back as 0.  They also
 *    blocked ~174us per character with interrupts disabled.
 *  - Startup kick is now armed only when a wheel starts from rest or
 *    reverses, not on every MOTOR_SPEEDS command (see applyWheelTarget).
 *  - Zero targets are handled per wheel, so a pivot no longer drives
 *    the held wheel backwards at MAX_REV_PWM.
 *  - Fixed the inverted minOutput clamp that capped reverse PWM at 30.
 *  - HALT now actually halts (brakes stay engaged, PID stops).
 *  - PID timer re-bases instead of accumulating a backlog.
 *  - Encoder counters widened from int to long (wrapped at ~194 m).
 *  - SPIN_BOOST applied as feed-forward, not into the PID accumulator.
 *  - Round-to-nearest in the P/D terms to remove the integer deadband.
 *  - PWM writes clamped; raw-PWM arguments range checked.
 *  - Bounds checks on the serial argument buffers and on UPDATE_PID.
 *  - Dropped the unused template files (motor_driver.*, encoder_driver.*,
 *    sensors.h, servos.*) -- all were inert behind USE_BASE/USE_SERVOS.
 *
 *  v1.52, from bench runs:
 *  - Speed limiting moved off MAX_FWD_PWM and onto the target, via the new
 *    MAX_TICKS_PER_FRAME.  MAX_FWD_PWM had been serving as both the speed
 *    limit and the PID's saturation ceiling, which cannot work: the loop ran
 *    pinned at its ceiling with positive error on both wheels and no
 *    authority to correct the left/right imbalance.
 *  - MAX_FWD_PWM 40 -> 50, now purely PID headroom.
 *  - Kick exit thresholds are now per wheel (KICK_MIN_TICKS_LEFT_* /
 *    _RIGHT_*), with the left set higher so the slower-starting left wheel
 *    stays at full PWM longer.
 *
 *  v1.54, from bench runs:
 *  - MAX_FWD_PWM 50 -> 70.  With the governor in place both wheels now hold
 *    the target, but the left needs PWM 50 to do it and so had no headroom
 *    left, while the right cruised at 24.
 *  - Kick PWM split out into KICK_FWD_PWM / KICK_REV_PWM so raising the
 *    ceiling for headroom does not also harden the start.
 *
 *  v1.60, from bench runs:
 *  - Each wheel now remembers the output it settled at and seeds the PID
 *    with it when the startup kick hands off, instead of always starting
 *    from minOutput.  Measured: the left wheel needs ~52 to hold 8
 *    ticks/frame and the right ~35, but both were seeded at 30 -- so the
 *    left spent ~22 frames climbing, longer than a typical move, running a
 *    tick per frame slow the whole way while the right ran a tick fast.
 *    That transient was the remaining source of veer (~18 ticks of
 *    differential, roughly 20 degrees of heading, per move).
 *  - MR_PWM_OFFSET removed.  It was a second, fixed correction for the same
 *    imbalance the PID now handles adaptively; having both made the two
 *    wheels' operating points incomparable.
 *
 *  Learned values are per direction, only used when the new target is close
 *  to the one they were learned at, and are lost on power cycle (relearned
 *  on the first move).  They deliberately survive resetPID(), which runs on
 *  every stop.
 *
 *  v1.61:
 *  - Encoder plausibility filter.  The right channel intermittently reported
 *    22 ticks in a frame while the left held a steady 8 at unchanged PWM,
 *    which is not motion.  Edges closer together than encMinEdgeUs (default
 *    1500) are rejected and counted.  New 'f <us>' command sets the window
 *    live; 'f 0' disables it, which is how to confirm the filter is what
 *    removed the spikes.  Rejected counts appear in the debug encoder line
 *    and in the 'f' reply, and are cleared by RESET_ENCODERS.
 *********************************************************************/


#define VERSION "1.70"


// pin defs
#define MR_SG  3
#define MR_BR  12
#define MR_DIR 11
#define MR_EN  8
#define MR_VR  9

#define ML_SG  2
#define ML_BR  4
#define ML_DIR 5
#define ML_EN  7
#define ML_VR  10

/* NOTE: MR_EN / ML_EN are wired but never configured or driven by this
   firmware.  Kept here as wiring documentation -- do not reuse these pins
   without checking the harness. */


/* Encoder ticks per wheel revolution.  Not used by this firmware (the host
   does the conversion), but it MUST match the enc_counts_per_rev parameter
   in diffbot.ros2_control.xacro. */
#define TICKS_PER_REV 90

/* Forward PWM ceiling.  This is the PID's saturation limit and NOT the speed
   limit -- see MAX_TICKS_PER_FRAME below for that.

   The two jobs are incompatible in one constant: a control loop can only
   regulate below its actuator ceiling, so if the ceiling is set to the
   desired top speed the loop runs pinned at saturation with no authority to
   correct anything.  That is what bench runs showed at 40 -- output clamped
   for an entire move with the error still positive on both wheels.

   Now 70.  50 was not enough: the left wheel needs PWM 50 just to hold 8
   ticks per frame, so it sat pinned at the ceiling with no room to correct
   while the right cruised at 24.  Since speed is governed by the target
   instead, raising this does not make the robot faster -- the only limit on
   it is what the driver and motors will take.

   The startup kick has its own PWM (KICK_FWD_PWM) so it is not dragged up
   with this.

   MAX_REV_PWM is deliberately left at -45; raise it too if reverse needs
   matching headroom. */
#define MAX_FWD_PWM        70
#define MAX_REV_PWM       -45

/* PWM used during the startup kick.  Split from MAX_FWD_PWM so that raising
   the PID's ceiling for headroom does not also make the start more violent.

   Lowered from 50 to 35 in v1.64.  50 was tuned against a front caster that
   was dragging hard; with that replaced by a ball roller the same kick
   overshoots badly.  On blocks -- i.e. with the drag gone -- PWM 50 produced
   21 ticks in one frame against a target of 8.

   The kick can only be applied in whole PID frames, so its minimum duration is
   250 ms -- far longer than stiction breakaway actually needs.  That is why
   the amplitude has to come down rather than the duration.

   Runtime-settable with 'k <fwd> [rev]' so this can be swept without
   reflashing; these are just the power-on defaults. */
#define KICK_FWD_PWM       35
#define KICK_REV_PWM      -45

int kickFwdPwm = KICK_FWD_PWM;
int kickRevPwm = KICK_REV_PWM;

/* Speed governor, in encoder ticks per PID frame.  This is what limits how
   fast the robot travels -- the job MAX_FWD_PWM used to do.  Requested
   targets above this are clamped, in both directions.

   8 ticks/frame is roughly what the wheels were achieving at the old PWM 40
   ceiling, so top speed is about unchanged: at 90 ticks/rev, 4 Hz and a
   0.085 m wheel radius it works out to ~0.19 m/s.  Lower this to slow the
   robot down; it no longer costs the PID its headroom. */
#define MAX_TICKS_PER_FRAME 8

/* MR_PWM_OFFSET removed in v1.60.  It subtracted a fixed 10 counts from the
   right motor's PWM to compensate for that wheel running faster -- which is
   exactly what the PID's accumulated output now does on its own, adaptively
   and per-run.  Two corrections working on the same error made the operating
   points impossible to compare between wheels, and the fixed subtraction went
   non-linear at low duty where it clamped at zero.  To restore it, subtract
   the constant again in rightWheelMove(). */

/* Extra PWM added to both motors when spinning on the spot (wheels moving in
   opposite directions) to overcome the increased friction of counter-rotation. */
#define SPIN_BOOST          5


/* Serial port baud rate */
#define BAUDRATE     57600

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"


  /* Run the PID loop at PID_RATE times per second.  The host's loop_rate
     parameter converts rad/s into ticks per frame and must match this. */
  #define PID_RATE           4     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL =  1000 / PID_RATE;

  /* Timestamp of last PID update, for overflow-safe interval tracking */
  unsigned long lastPIDTime = 0;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 5000
  unsigned long lastMotorCommand = 0;

  /* True while the motors are under an active command (PID or raw PWM).
     Used so the auto-stop fires exactly once rather than re-issuing a stop
     on every pass of loop(), and so it still covers raw PWM, which runs
     with moving == 0. */
  bool motorsCommanded = false;



#define DIR_STOPPED 0
#define DIR_FWD     1
#define DIR_BWD     2

/* Written by the main loop, read by the encoder ISRs. */
volatile char dir_left;
volatile char dir_right;

/* Encoder positions.  These are `long`, not `int`: at 90 ticks/rev and a
   0.085 m wheel radius an int16 wrapped after ~364 revolutions (~194 m of
   travel), which showed up host-side as a ~388 m position jump and a huge
   phantom velocity fed straight into odom. */
volatile long pos_left = 0;       //Left motor encoder position
volatile long pos_right = 0;      //Right motor encoder position

/* Encoder plausibility filter.
 *
 * Bench runs showed the right channel intermittently reporting 22 ticks in a
 * frame while the left held a steady 8 and the commanded PWM was unchanged --
 * physically impossible, so those counts are electrical rather than motion.
 *
 * An edge arriving sooner than any real tick could is rejected.  Raised from
 * 1500 to 5000 us in v1.67: at 1500 the filter was catching only part of the
 * burst -- one bench run rejected 14 edges and still let deltas of 12, 15 and
 * 13 through against a target of 8, so not all the spurious edges are closely
 * spaced.
 *
 * 5000 us still permits 200 ticks/s.  The fastest real rate measured is ~10
 * ticks/frame during the kick, i.e. 40 ticks/s or 25 ms between edges, so this
 * keeps roughly 5x headroom over anything the drivetrain actually does.
 *
 * The left channel is the control: it has rejected zero edges in every run to
 * date.  If `rej` starts climbing on the LEFT, the window is eating real ticks
 * and must come back down -- that failure mode makes the encoder under-count,
 * which drives the wheel faster, so it is the dangerous direction.
 *
 * The window is anchored to the last ACCEPTED edge, not the last rejected
 * one, so a sustained burst of noise cannot hold the gate shut indefinitely.
 *
 * Set encMinEdgeUs to 0 (command 'f 0') to disable the filter and confirm the
 * spikes return -- that A/B is the point of making it runtime-settable. */
#define ENC_MIN_EDGE_US 5000

volatile unsigned long encMinEdgeUs = ENC_MIN_EDGE_US;
volatile unsigned long lastEdgeLeftUs = 0;
volatile unsigned long lastEdgeRightUs = 0;

/* Diagnostic counts of rejected edges.  If these stay at zero while the spikes
   still happen, the extra counts are not closely-spaced chatter and the cause
   is something else (genuine wheel slip, or widely spaced interference). */
volatile unsigned long encRejectedLeft = 0;
volatile unsigned long encRejectedRight = 0;

int debugMode = 0;
/* Variable initialization */

// A pair of variables to help parse serial commands (thanks Fergs)
int arg = 0;
int charIdx = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
#define ARGV_SIZE 16
char argv1[ARGV_SIZE];
char argv2[ARGV_SIZE];

// The arguments converted to integers
long arg1;
long arg2;

void setMotorSpeeds(int leftSpeed, int rightSpeed){
  leftWheelMove(leftSpeed);
  rightWheelMove(rightSpeed);
}

long readEncoder(int i)
{
  long val;
  noInterrupts();
  if (i == LEFT)
    val = pos_left;
  else
    val = pos_right;
  interrupts();
  return val;
}

void resetEncoders()
{
  /* 32-bit writes are not atomic on AVR, and the ISRs touch these. */
  noInterrupts();
  pos_left  = 0;
  pos_right = 0;
  interrupts();
}

/* PID parameters and functions */
#include "diff_controller.h"

/* Clear the current command parameters */
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  charIdx = 0;
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  long reqL, reqR;      /* requested targets, before the speed governor */
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;

  case HALT:
      /* A real stop.  The previous version re-drove ML_BR/MR_BR LOW right
         after stopping, releasing the brakes it had just engaged, and never
         cleared `moving` or the targets -- so the next PID frame (<=250 ms
         later) took the DIR_STOPPED branch and drove both motors again at
         minOutput.  HALT undid itself. */
      leftPID.TargetTicksPerFrame  = 0;
      rightPID.TargetTicksPerFrame = 0;
      moving = 0;
      motorsCommanded = false;
      leftWheelStop();
      rightWheelStop();
      resetPID();
      lastMotorCommand = millis();
      Serial.println("OK");
      break;

  case DBUG:
    debugMode = arg1;
      Serial.println("OK");
      break;

  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.print(readEncoder(RIGHT));
    Serial.print(" ");
    Serial.print(leftPID.output);
    Serial.print(" ");
    Serial.println(rightPID.output);

   break;

   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    noInterrupts();
    encRejectedLeft  = 0;
    encRejectedRight = 0;
    interrupts();
    Serial.println("OK");
    break;

   case KICK_PWM:
    /* 'k <fwd> [rev]' sets the startup kick strength live.  Reverse is given
       as a magnitude and stored negative.  A bare 'k' only reports. */
    if (argv1[0] != '\0')
      kickFwdPwm = constrain(arg1, 0, 255);
    if (argv2[0] != '\0')
      kickRevPwm = -constrain(abs(arg2), 0, 255);
    Serial.print("OK kick ");
    Serial.print(kickFwdPwm);
    Serial.print(" ");
    Serial.println(kickRevPwm);
    break;

   case LEARN_VALUES:
    /* 'l' reports the learned operating points; 'l 0' clears them.  They
       deliberately survive resetPID() (which runs on every stop), so this is
       the only way to drop them short of a power cycle -- needed whenever the
       plant changes underneath them, e.g. moving between blocks and floor, or
       swapping the front support. */
    if (argv1[0] != '\0' && arg1 == 0) {
      leftPID.learnedFwdValid  = false;  leftPID.learnedRevValid  = false;
      rightPID.learnedFwdValid = false;  rightPID.learnedRevValid = false;
    }
    Serial.print("OK learned L ");
    Serial.print(leftPID.learnedFwdValid ? leftPID.learnedFwd : -1);
    Serial.print("/");
    Serial.print(leftPID.learnedRevValid ? leftPID.learnedRev : -1);
    Serial.print(" R ");
    Serial.print(rightPID.learnedFwdValid ? rightPID.learnedFwd : -1);
    Serial.print("/");
    Serial.println(rightPID.learnedRevValid ? rightPID.learnedRev : -1);
    break;

   case ENC_FILTER:
    /* 'f <microseconds>' sets the plausibility window; 'f 0' disables the
       filter, which is how to A/B whether it is what removed the spikes.
       A bare 'f' only reports -- it used to parse as 0 and silently disable
       the filter, which cost a set of test runs. */
    if (argv1[0] != '\0') {
      noInterrupts();
      encMinEdgeUs = (arg1 < 0) ? 0 : (unsigned long)arg1;
      interrupts();
    }
    Serial.print("OK ");
    Serial.print(encMinEdgeUs);
    Serial.print(" rej l/r ");
    Serial.print(encRejectedLeft);
    Serial.print(" ");
    Serial.println(encRejectedRight);
    break;

   case MOTOR_RAW_PWM:
     /* Raw PWM deliberately bypasses the PID, so clear `moving` to stop the
        PID loop overwriting the value on the next frame.

        Arguments are clamped to +/-255 because analogWrite() truncates into
        an 8-bit register, so out-of-range values silently wrapped rather than
        saturating (o 300 became 44). */
     arg1 = constrain(arg1, -255L, 255L);
     arg2 = constrain(arg2, -255L, 255L);
     moving = 0;
     motorsCommanded = (arg1 != 0 || arg2 != 0);
     leftWheelMove(arg1);
     rightWheelMove(arg2);
     Serial.println("OK");
     lastMotorCommand = millis();
     break;

   case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();

    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
      motorsCommanded = false;
      if (debugMode)
        Serial.println("mx");
      Serial.println("OK");
      break;
    }

    moving = 1;
    motorsCommanded = true;

    /* Govern the pair together so the commanded turn survives -- see
       governSpeedPair(). Clamping each wheel separately flattens a turn into a
       straight line whenever both exceed the limit. */
    reqL = arg1;
    reqR = arg2;
    governSpeedPair(&arg1, &arg2);

    if(debugMode)
    {
      Serial.print("ms: ");
      Serial.print(reqL);
      Serial.print(" ");
      Serial.print(reqR);
      if (reqL != arg1 || reqR != arg2) {
        Serial.print(" -> ");
        Serial.print(arg1);
        Serial.print(" ");
        Serial.print(arg2);
      }
      Serial.println();
    }

    /* Per-wheel, so that a zero target on one side is a genuine "hold still"
       rather than falling through to the kick path.  applyWheelTarget also
       decides whether the startup kick actually needs arming -- see the
       comment on that function. */
    applyWheelTarget(&leftPID,  arg1, dir_left);
    applyWheelTarget(&rightPID, arg2, dir_right);

    Serial.println("OK");
    break;

  case UPDATE_PID:
    /* Bounded: the original loop had no `i < 4` guard, so a command with more
       than four fields wrote past pid_args[], and one with fewer applied
       uninitialised stack as gains. */
    while (i < 4 && (str = strtok_r(p, ":", &p)) != NULL) {
       pid_args[i] = atoi(str);
       i++;
    }
    if (i < 4 || pid_args[3] == 0) {   /* Ko == 0 would divide by zero */
      Serial.println("Invalid PID");
      break;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}

void leftWheelStop(){
  digitalWrite(ML_BR,HIGH);
  analogWrite(ML_VR,0);
  if (dir_left != DIR_STOPPED)
    if(debugMode)
      Serial.println("L stop");
  dir_left = DIR_STOPPED;

}

void rightWheelStop(){
  digitalWrite(MR_BR,HIGH);
  analogWrite(MR_VR,0);
  if (dir_right != DIR_STOPPED)
    if(debugMode)
      Serial.println("R stop");
  dir_right = DIR_STOPPED;
}

void leftWheelMove( int speed){

       if (speed == 0)
      {
        leftWheelStop();
        return;
      }
      if(debugMode) {
        Serial.print("Lspd: ");
        Serial.print(speed);
        Serial.print(" ");
        Serial.println(millis());
      }

      /* Clamped: analogWrite() writes into an 8-bit register, so anything
         outside 0..255 wraps rather than saturating. */
      int pwm = constrain(abs(speed), 0, 255);
      analogWrite(ML_VR, pwm);

      if (speed > 0){
        digitalWrite(ML_DIR,LOW);
        dir_left = DIR_FWD;
      }
      else {
         digitalWrite(ML_DIR,HIGH);
         dir_left = DIR_BWD;
      }

      digitalWrite(ML_BR,LOW);
 }

void rightWheelMove(int speed){

      if (speed == 0)
      {
        rightWheelStop();
        return;
      }

      if(debugMode) {
        Serial.print("Rspd: ");
        Serial.print(speed);
        Serial.print(" ");
        Serial.println(millis());
      }

      /* Clamped: analogWrite() writes into an 8-bit register, so anything
         outside 0..255 wraps rather than saturating. */
      int pwm = constrain(abs(speed), 0, 255);
      analogWrite(MR_VR, pwm);

      if (speed > 0){
        digitalWrite(MR_DIR,HIGH);
        dir_right = DIR_FWD;
      }
      else {
         digitalWrite(MR_DIR,LOW);
         dir_right = DIR_BWD;
      }
      digitalWrite(MR_BR,LOW);
 }

/* Encoder ISRs.  Keep these short and free of any Serial I/O: at 57600 baud a
   single character blocks ~174 us with interrupts disabled, which both delays
   millis() and drops the other wheel's edges.  Counting direction is taken
   from the last commanded direction rather than a second encoder channel. */
void rightMotorInt() {

  /* Reject edges closer together than any real tick can be -- see
     encMinEdgeUs.  Unsigned arithmetic makes the micros() rollover harmless. */
  if (encMinEdgeUs) {
    unsigned long now = micros();
    if (now - lastEdgeRightUs < encMinEdgeUs) {
      encRejectedRight++;
      return;
    }
    lastEdgeRightUs = now;
  }

  switch(dir_right) {

  case DIR_STOPPED:
    break;

  case DIR_FWD:
    pos_right ++;
    break;

  case DIR_BWD:
    pos_right --;
    break;
  }
}

void leftMotorInt() {

  if (encMinEdgeUs) {
    unsigned long now = micros();
    if (now - lastEdgeLeftUs < encMinEdgeUs) {
      encRejectedLeft++;
      return;
    }
    lastEdgeLeftUs = now;
  }

  switch(dir_left) {

  case DIR_STOPPED:
    break;

  case DIR_BWD:
    pos_left --;
    break;

  case DIR_FWD:
    pos_left ++;
    break;


  }
}


/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  //wheel left - Setup pins

  pinMode(ML_BR, OUTPUT);    //stop/start - EL
  digitalWrite(ML_BR,HIGH);
  pinMode(ML_SG, INPUT);     //plus       - Signal
  pinMode(ML_DIR, OUTPUT);   //direction  - ZF
  pinMode(ML_VR, OUTPUT);    //pwm output
  analogWrite(ML_VR,0);

  leftWheelStop();

  //Hall sensor detection - Count steps

  attachInterrupt(digitalPinToInterrupt(ML_SG), leftMotorInt, CHANGE);

  //wheel right - Setup pins
  pinMode(MR_BR, OUTPUT);    //stop/start - EL
  digitalWrite(MR_BR,HIGH);

  pinMode(MR_SG, INPUT);     //plus       - Signal
  pinMode(MR_DIR, OUTPUT);   //direction  - ZF
  pinMode(MR_VR, OUTPUT);    //pwm output
  analogWrite(MR_VR,0);

  //Hall sensor detection - Count steps

  attachInterrupt(digitalPinToInterrupt(MR_SG), rightMotorInt, CHANGE);
  rightWheelStop();

  resetPID();
  lastPIDTime = millis();
  Serial.print("* Base Setup Done: " );
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.print(__TIME__);
  Serial.print(" v");
  Serial.println(VERSION);
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[charIdx] = '\0';
      else if (arg == 2) argv2[charIdx] = '\0';
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[charIdx] = '\0';
        arg = 2;
        charIdx = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      /* Bounded: the original incremented charIdx with no check against the
         16-byte buffers, so a long argument ran off the end of argv1/argv2. */
      else if (arg == 1) {
        if (charIdx < ARGV_SIZE - 1) {
          argv1[charIdx] = chr;
          charIdx++;
        }
      }
      else if (arg == 2) {
        if (charIdx < ARGV_SIZE - 1) {
          argv2[charIdx] = chr;
          charIdx++;
        }
      }
    }
  }

  unsigned long nowMillis = millis();
  if (nowMillis - lastPIDTime >= (unsigned long)PID_INTERVAL) {
    updatePID();
    /* Re-base on the current time rather than `lastPIDTime += PID_INTERVAL`.
       Accumulating the interval builds up a backlog whenever the loop stalls,
       then fires several PID frames back to back with almost no encoder delta
       between them, ratcheting the output straight into saturation. */
    lastPIDTime = nowMillis;
  }

  // Check to see if we have exceeded the auto-stop interval
  if (motorsCommanded && (millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    leftPID.TargetTicksPerFrame  = 0;
    rightPID.TargetTicksPerFrame = 0;
    moving = 0;
    motorsCommanded = false;
    if (debugMode)
      Serial.println("auto-stop");
  }
}
