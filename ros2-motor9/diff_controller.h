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

  /* Startup kick state.
   * inKick is set true by the MOTOR_SPEEDS command and cleared by doPID once
   * the encoder has accumulated KICK_MIN_TICKS, or KICK_TIMEOUT_CYCLES elapse.
   * kickCycles counts PID frames elapsed; kickTicksAccum accumulates encoder
   * ticks seen across all kick frames. */
  bool inKick;
  int  kickCycles;
  int  kickTicksAccum;
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 25;
int KpStart = 40;
int KpRunning = 20;
int Kd = 10;
int Ki = 0;
int Ko = 40;

int minOutput = 30;

/* Minimum encoder ticks that must be observed (accumulated across kick frames)
   before the kick phase ends and PID takes over. Forward and reverse are tuned
   separately because drivetrain stiction is often asymmetric. */
#define KICK_MIN_TICKS_FWD  3
#define KICK_MIN_TICKS_REV  7

/* Maximum PID frames to stay in kick before handing off regardless of ticks,
   to prevent hanging indefinitely if an encoder is broken. At PID_RATE = 2 Hz
   this is 3 * 500 ms = 1.5 s. */
#define KICK_TIMEOUT_CYCLES 5

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
   leftPID.inKick = false;
   leftPID.kickCycles = 0;
   leftPID.kickTicksAccum = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.PrevErr = 0;
   rightPID.output = 0;
   rightPID.PrevDelta = 0;
   rightPID.ITerm = 0;
   rightPID.idx = 1;
   rightPID.inKick = false;
   rightPID.kickCycles = 0;
   rightPID.kickTicksAccum = 0;
}

/* PID routine to compute the next motor commands. */
void doPID(SetPointInfo * p, char motor_dir) {
  long Perror;
  long outputP;
  long outputD;
  long outputI;
  long output;
  int delta;

  delta = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - delta;

  /* --- Startup kick phase ---
   *
   * Apply full power in the target direction until the encoder has accumulated
   * at least KICK_MIN_TICKS across one or more PID frames, confirming inertia
   * has been overcome.  KICK_TIMEOUT_CYCLES acts as a safety ceiling so we
   * never hang here if an encoder is faulty.
   *
   * On exit the PID's accumulated output is seeded at minOutput (in the
   * correct direction) so the handoff to normal PID is smooth — the output
   * doesn't suddenly drop from MAX back to zero and cause a speed dip.
   *
   * PrevEnc and PrevErr are synchronised at the moment of handoff so the
   * derivative term does not produce a spike on the first real PID frame.
   */
  if (p->inKick) {
    p->kickCycles++;
    p->kickTicksAccum += abs(delta);

    output = (p->TargetTicksPerFrame > 0) ? MAX_FWD_PWM : MAX_REV_PWM;

    if (debugMode) {
      Serial.print("KICK ");
      Serial.print(p->idx);
      Serial.print(" cyc=");
      Serial.print(p->kickCycles);
      Serial.print(" delta=");
      Serial.print(delta);
      Serial.print(" ticks=");
      Serial.println(p->kickTicksAccum);
    }

    int kickMinTicks = (p->TargetTicksPerFrame > 0) ? KICK_MIN_TICKS_FWD : KICK_MIN_TICKS_REV;

    // Exit kick when enough ticks have accumulated OR the timeout is reached
    if (p->kickTicksAccum >= kickMinTicks || p->kickCycles >= KICK_TIMEOUT_CYCLES) {
      p->inKick = false;
      // Seed PID from minOutput so the motor doesn't lurch on the first PID frame
      p->output = (p->TargetTicksPerFrame > 0) ? minOutput : -minOutput;
      // Sync history to suppress a D-term spike on the first real PID frame
      p->PrevEnc = p->Encoder;
      p->PrevErr = 0;

      if (debugMode) {
        Serial.print("KICK ");
        Serial.print(p->idx);
        Serial.print(" done (ticks=");
        Serial.print(p->kickTicksAccum);
        Serial.println("), handing to PID");
      }
    } else {
      p->output = output;
    }

    p->PrevDelta = delta;
    return;   // skip normal PID this kick frame
  }

  /* --- Normal PID --- */

  if(debugMode) {
    Serial.print("PID ");
    Serial.print(p->idx);
    Serial.print(" ttpf ");
    Serial.print(p->TargetTicksPerFrame);
    Serial.print(" enc ");
    Serial.print(p->Encoder);
    Serial.print(" Penc ");
    Serial.print(p->PrevEnc);
    Serial.print(" del ");
    Serial.print(delta);
    Serial.print(" pe ");
    Serial.println(Perror);
  }

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */

  outputP = (Kp * Perror)/Ko;
  outputD = (Kd * (Perror - p->PrevErr))/Ko;
  outputI = p->ITerm;
  output = outputP + outputD + outputI;

  if(debugMode) {
    Serial.print("    p ");
    Serial.print(Kp);
    Serial.print(" ");
    Serial.print(outputP);
    Serial.print(" d ");
    Serial.print(outputD);
    Serial.print(" i ");
    Serial.print(outputI);
    Serial.print(" sm ");
    Serial.print(output);
    Serial.print(" nw ");
    Serial.println(output + p->output);
  }

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

  // Clamp output and track saturation for anti-windup
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
    // Motor is stopped but PID is running (e.g. stall): hold at min power
    // in the target direction rather than slamming to MAX.
    saturated = true;
    output = (p->TargetTicksPerFrame > 0) ? minOutput : -minOutput;
  }

  // Only accumulate ITerm when not saturated (anti-windup)
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
  if(debugMode) {
    Serial.print("enc l: ");
    Serial.print(leftPID.Encoder);
    Serial.print(" r: ");
    Serial.println(rightPID.Encoder);
  }

  /* Compute PID update for each motor, passing each motor's own direction. */
  doPID(&rightPID, dir_right);
  doPID(&leftPID, dir_left);

  /* When the wheels are spinning in opposite directions (turning on the spot)
   * friction is higher than during straight travel.  Apply SPIN_BOOST to the
   * magnitude of both outputs, clamped to the existing PWM limits. */
  if ((dir_left == DIR_FWD && dir_right == DIR_BWD) ||
      (dir_left == DIR_BWD && dir_right == DIR_FWD)) {
    if (leftPID.output > 0)
      leftPID.output = min((long)MAX_FWD_PWM, leftPID.output + SPIN_BOOST);
    else if (leftPID.output < 0)
      leftPID.output = max((long)MAX_REV_PWM, leftPID.output - SPIN_BOOST);

    if (rightPID.output > 0)
      rightPID.output = min((long)MAX_FWD_PWM, rightPID.output + SPIN_BOOST);
    else if (rightPID.output < 0)
      rightPID.output = max((long)MAX_REV_PWM, rightPID.output - SPIN_BOOST);

    if (debugMode) {
      Serial.print("SPIN BOOST l=");
      Serial.print(leftPID.output);
      Serial.print(" r=");
      Serial.println(rightPID.output);
    }
  }

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}
