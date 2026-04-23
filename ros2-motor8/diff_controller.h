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
   // FIX: was leftPID.PrevErr = 0, so rightPID.PrevErr was never initialized,
   // causing a derivative spike on the first PID update after reset.
   rightPID.PrevErr = 0;
   rightPID.output = 0;
   rightPID.PrevDelta = 0;
   rightPID.ITerm = 0;
   rightPID.idx = 1;
}

/* PID routine to compute the next motor commands.
 *
 * FIX: motor_dir parameter added so clamping uses this motor's own direction
 * instead of checking both global dir_left and dir_right.  Previously, if only
 * one motor was running the other motor's PID would take the wrong clamp branch.
 */
void doPID(SetPointInfo * p, char motor_dir) {
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

  outputP = (Kp * Perror)/Ko;
  outputD = (Kd * (Perror - p->PrevErr))/Ko;
  outputI = p->ITerm;
  // FIX: outputI was computed but never included in output, making Ki completely
  // ineffective regardless of its value. Added here.
  output = outputP + outputD + outputI;

  if(debugMode)
    Serial.println("    p " + String(Kp) + " " + String(outputP) + " d " + String(outputD) + " i " + String(outputI) + " sm " + String(output) + " nw " + String(output + p->output) );

  p->PrevEnc = p->Encoder;
  p->PrevErr = Perror;

  output = output + p->output;

  if (output < 0)
  {
     if (output < -minOutput)
       output = -minOutput;
  }
  else
  {
    if(output < minOutput)
        output = minOutput;
  }

  // Clamp output and track whether we are saturated for anti-windup below.
  // FIX: was checking (dir_left != DIR_STOPPED) || (dir_right != DIR_STOPPED),
  // which used the *other* motor's direction when computing one motor's PID,
  // causing wrong clamping when only one motor was in motion.
  bool saturated = false;
  if (motor_dir != DIR_STOPPED)
  {
    if (output >= MAX_FWD_PWM) {
      output = MAX_FWD_PWM;
      saturated = true;
    } else if (output <= MAX_REV_PWM) {
      output = MAX_REV_PWM;
      saturated = true;
    }
  }
  else
  {
    // Give startup kick to overcome inertia
    saturated = true;
    if (output < 0)
      output = MAX_REV_PWM;
    else
      output = MAX_FWD_PWM;
    lastPIDTime = millis() - (PID_INTERVAL*3);

  }

  // FIX: ITerm previously accumulated unconditionally, causing integrator windup
  // when the output was saturated.  Only accumulate when not at the limit.
  if (!saturated) {
    p->ITerm += Ki * Perror;
  }

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
    Serial.println("enc l: " + String( leftPID.Encoder) + " r: " + String( rightPID.Encoder));

  /* Compute PID update for each motor, passing each motor's own direction. */
  // FIX: pass dir_right / dir_left so doPID uses the correct per-motor direction.
  doPID(&rightPID, dir_right);
  doPID(&leftPID, dir_left);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}
