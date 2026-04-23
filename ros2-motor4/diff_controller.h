/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:

   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  long PrevErr;                  // last error

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevDelta;                // last input

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  int ITerm;                    //integrated term

  long output;                    // last motor setting
  int idx;
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20;
int KpStart = 40;
int KpRunning = 20;
int Kd = 10;
int Ki = 0;
int Ko = 40;

int minOutput = 25;


unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/

void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.PrevErr = 0;
   leftPID.output = 0;
   leftPID.PrevDelta = 0;
   leftPID.ITerm = 0;
   leftPID.idx = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.PrevErr = 0;  // FIX #1: was leftPID.PrevErr (copy-paste bug)
   rightPID.output = 0;
   rightPID.PrevDelta = 0;
   rightPID.ITerm = 0;
   rightPID.idx = 1;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long outputP;
  long outputD;
  long outputI;
  long output;
  int delta;

  delta = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - delta;

  if(debugMode)
    Serial.println("PID " + String(p->idx)+ " ttpf " + String(p->TargetTicksPerFrame) + " enc " + String(p->Encoder) + " Penc " + String(p->PrevEnc) +" del " +String(delta) + " pe " + String(Perror));

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */

  outputP = (Kp * Perror) / Ko;
  outputD = (Kd * (Perror - p->PrevErr)) / Ko;
  outputI = p->ITerm;
  // FIX #13: ITerm (outputI) was computed but never added to output
  output = outputP + outputD + outputI;

  if(debugMode)
    Serial.println("    p " + String(Kp) + " " + String(outputP) + " d " + String(outputD) + " i " + String(outputI) + " sm " + String(output) + " nw " + String(output + p->output));

  p->PrevEnc = p->Encoder;
  p->PrevErr = Perror;

  output = output + p->output;

  // FIX #2: The original code unconditionally applied "output -= 5" even after
  // clamping to -minOutput, due to a missing else. Fixed so the -5 kick only
  // applies when the output is not already at or beyond -minOutput.
  if (output < 0)
  {
    if (output < -minOutput)
      output = -minOutput;
    else
      output -= 5;
  }
  else
  {
    if (output < minOutput)
      output = minOutput;
  }

  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if ((dir_left != DIR_STOPPED) || (dir_right != DIR_STOPPED))
  {
    if (output >= MAX_FWD_PWM)
      output = MAX_FWD_PWM;
    else if (output <= MAX_REV_PWM)
      output = MAX_REV_PWM;
  }
  else
  {
    // give kick at start to overcome inertia
    if (output < 0)
      output = MAX_REV_PWM;
    else
      output = MAX_FWD_PWM;
  }

  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  p->ITerm += Ki * Perror;
  p->output = output;
  p->PrevDelta = delta;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftPID.PrevDelta != 0 || rightPID.PrevDelta != 0) resetPID();
    return;
  }
  if(debugMode)
    Serial.println("enc l: " + String(leftPID.Encoder) + " r: " + String(rightPID.Encoder));

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}
