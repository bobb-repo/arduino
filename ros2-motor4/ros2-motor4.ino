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
#define MR_SG  2
#define MR_BR  12
#define MR_DIR 11
#define MR_EN  0   // NOTE: pin 0 is also Serial RX; see hardware notes
#define MR_VR  9

#define ML_SG  3
#define ML_BR  4
#define ML_DIR 5
#define ML_EN  0   // NOTE: pin 0 is also Serial RX; see hardware notes
#define ML_VR  10

// FIX #9: Named constant for right-motor PWM balance trim (replaces magic number -3)
#define RIGHT_MOTOR_PWM_TRIM  3

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


  /* Run the PID loop at 2 times per second */  // FIX #8: corrected comment (was "30 times per second")
  #define PID_RATE           2     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL =  1000 / PID_RATE;

  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 3000
  // FIX #7: Use unsigned long to match millis() type, initialized to 0.
  // Previously initialized to AUTO_STOP_INTERVAL (3000) as a long, causing an unsigned
  // underflow in the auto-stop check for the first ~6 seconds after boot.
  unsigned long lastMotorCommand = 0;




#define DIR_STOPPED 0
#define DIR_FWD     1
#define DIR_BWD     2

#define LEFT  0
#define RIGHT 1

char dir_left;
char dir_right;

// FIX #3: Changed from int to long to prevent overflow at ±32767 ticks
volatile long pos_left = 0;       //Left motor encoder position
volatile long pos_right = 0;      //Right motor encoder position

int debugMode = 0;
/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int idx = 0;   // FIX #10 (minor): renamed from 'index' to avoid shadowing stdlib index()

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

// FIX #4: Disable interrupts around the volatile 16→32-bit read to prevent
// a torn read if the ISR fires between byte reads on AVR.
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
  noInterrupts();
  pos_left  = 0;
  pos_right  = 0;
  interrupts();
}

/* PID parameters and functions */
#include "diff_controller.h"

/* Clear the current command parameters */
void resetCommand() {
  cmd = '\0';   // FIX #10: was NULL (pointer literal) assigned to char; use '\0'
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  idx = 0;
}

/* Run a command.  Commands are defined in commands.h */
// FIX #12: added explicit return 0 on all paths; function declared int to match original
int runCommand() {
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
      digitalWrite(ML_BR,LOW);

      rightWheelStop();
      digitalWrite(MR_BR,LOW);
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
      return 0;
    }
    else moving = 1;

    if(debugMode)
    {
      Serial.print("m  ");
      Serial.print(arg1);
      Serial.print(" ");
      Serial.println(arg2);
    }

    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK");
    break;

  case UPDATE_PID:
    // FIX #11: compare strtok_r return to NULL (pointer), not '\0' (char)
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
  return 0;  // FIX #12: explicit return on all paths
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

void leftWheelMove(int speed){

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

      // FIX #9: RIGHT_MOTOR_PWM_TRIM replaces the magic literal -3
      analogWrite(MR_VR, abs(speed - RIGHT_MOTOR_PWM_TRIM));

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
    pos_right++;
    break;

  case DIR_BWD:
    pos_right--;
    break;
  }
}

void leftMotorInt() {

  switch(dir_left) {

  case DIR_STOPPED:
    break;

  case DIR_FWD:
    pos_left++;
    break;

  case DIR_BWD:
    pos_left--;
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
  Serial.print("* Base Setup Done: ");
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
      if (arg == 1) argv1[idx] = '\0';        // FIX #10: use '\0' not NULL
      else if (arg == 2) argv2[idx] = '\0';   // FIX #10: use '\0' not NULL
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[idx] = '\0';   // FIX #10: use '\0' not NULL
        arg = 2;
        idx = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // FIX #6: bounds check to prevent buffer overflow
        if (idx < (int)(sizeof(argv1) - 1)) {
          argv1[idx] = chr;
          idx++;
        }
      }
      else if (arg == 2) {
        // FIX #6: bounds check to prevent buffer overflow
        if (idx < (int)(sizeof(argv2) - 1)) {
          argv2[idx] = chr;
          idx++;
        }
      }
    }
  }

  // Run a PID calculation at the appropriate intervals
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  // FIX #7: lastMotorCommand is now unsigned long so this subtraction is safe
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}
