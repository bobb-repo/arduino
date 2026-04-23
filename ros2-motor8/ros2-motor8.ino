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


// pin defs
#define MR_SG  3
#define MR_BR  12
#define MR_DIR 11
// FIX: was 0 (serial RX pin) for both MR_EN and ML_EN — verify correct pins for your hardware
#define MR_EN  8
#define MR_VR  9

#define ML_SG  2
#define ML_BR  4
#define ML_DIR 5
// FIX: was 0 (serial RX pin), same value as MR_EN — verify correct pin for your hardware
#define ML_EN  7
#define ML_VR  10



#define TICKS_PER_REV 90
#define MAX_FWD_PWM       36
#define MAX_REV_PWM       -40

unsigned long lastMilli = 0;


/* Serial port baud rate */
#define BAUDRATE     57600


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"
// FIX: LEFT and RIGHT are already defined in commands.h; duplicate #defines removed.


  /* Run the PID loop at PID_RATE times per second */
  #define PID_RATE           2     // Hz
  // FIX: comment previously said "30 times per second" but value was 2.

  /* Convert the rate into an interval */
  const int PID_INTERVAL =  1000 / PID_RATE;

  /* Timestamp of last PID update, for overflow-safe interval tracking */
  // FIX: replaced nextPID with lastPIDTime to avoid millis() rollover bug.
  unsigned long lastPIDTime = 0;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 5000
  long lastMotorCommand = AUTO_STOP_INTERVAL;



#define DIR_STOPPED 0
#define DIR_FWD     1
#define DIR_BWD     2

char dir_left;
char dir_right;

volatile int pos_left = 0;       //Left motor encoder position
volatile int pos_right = 0;      //Right motor encoder position

int debugMode = 0;
/* Variable initialization */

// A pair of variables to help parse serial commands (thanks Fergs)
int arg = 0;
// FIX: renamed 'index' to 'charIdx' to avoid shadowing the POSIX index() function.
int charIdx = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

void initMotorController()
{}

void setMotorSpeeds(int leftSpeed, int rightSpeed){
  leftWheelMove(leftSpeed);
  rightWheelMove(rightSpeed);
}

long readEncoder(int i)
{
  // FIX: reading a 16-bit volatile variable is non-atomic on 8-bit AVR;
  // disable interrupts while copying to prevent a torn read.
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
  pos_left  = 0;
  pos_right  = 0;
}

/* PID parameters and functions */
#include "diff_controller.h"

/* Clear the current command parameters */
void resetCommand() {
  // FIX: use '\0' (null character) instead of NULL (a pointer macro) for char variables.
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  charIdx = 0;
}

/* Run a command.  Commands are defined in commands.h */
// FIX: return type changed from int to void — function never returned a value.
void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;

  case HALT:
      leftWheelStop();
      digitalWrite(ML_BR,LOW); // cn test change

      rightWheelStop();
      digitalWrite(MR_BR,LOW); // cn test change
      Serial.println("OK");
      break;

  case DBUG:
    debugMode = arg1;
      Serial.println("OK");
      break;

  case READ_ENCODERS:

    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;

   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;

   case MOTOR_RAW_PWM:
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
      if (debugMode)
        Serial.println("mx");
      Serial.println("OK");
      return;
    }
    else moving = 1;

    if(1) // debugMode)
    {
      Serial.print("ms: ");
      Serial.print(arg1);
      Serial.print(" ");
      Serial.println(arg2);
    }

    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK");
    break;

  case UPDATE_PID:
    // FIX: comparison changed from != '\0' to != NULL; strtok_r returns char*, not char.
    while ((str = strtok_r(p, ":", &p)) != NULL) {
       pid_args[i] = atoi(str);
       i++;
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
      if(debugMode)
        Serial.println("Lspd: " + String(speed) + " " + String(millis()));

      analogWrite(ML_VR, abs(speed));

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

      if(debugMode)
        Serial.println("Rspd: " + String(speed) + " " + String(millis()));

      // FIX: was abs(speed-3) which applied the trim asymmetrically inside abs(),
      // giving different offsets for forward vs reverse. Corrected to abs(speed)-3
      // with a floor of 0 to prevent negative PWM values.
      analogWrite(MR_VR, max(0, abs(speed) - 3));

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

void rightMotorInt() {

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

  switch(dir_left) {

  case DIR_STOPPED:
    break;

  case DIR_FWD:
    pos_left ++;
    break;

  case DIR_BWD:
    pos_left --;
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
  Serial.print("* Base Setup Done: " );
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.println(__TIME__);
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
      // FIX: use '\0' (null character) instead of NULL (a pointer macro) for char arrays.
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
        // FIX: use '\0' instead of NULL for char array termination.
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
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[charIdx] = chr;
        charIdx++;
      }
      else if (arg == 2) {
        argv2[charIdx] = chr;
        charIdx++;
      }
    }
  }

// FIX: replaced nextPID (absolute timestamp comparison) with elapsed-time pattern
// to correctly handle millis() rollover after ~49 days.
  if (millis() - lastPIDTime >= (unsigned long)PID_INTERVAL) {

    //if(debugMode)
      //Serial.println("upid " + String(millis()));
    
    updatePID();
    lastPIDTime += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}
