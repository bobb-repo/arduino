/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:

   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  /* Target speed in ticks per PID frame.  Integer rather than double: it is
     only ever assigned whole tick counts parsed from the serial command, and
     an exact == 0 test is needed to recognise a "hold this wheel still"
     command.  It also keeps AVR floating point out of the control loop. */
  long TargetTicksPerFrame;
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  long PrevErr;                  // last error

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  /* Last encoder delta.  long, matching Encoder/PrevEnc: an int would
     truncate if the two ever diverge widely (e.g. a stale PrevEnc after a
     long stall).  Only ever tested against zero. */
  long PrevDelta;               // last input

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  int ITerm;                    //integrated term

  long output;                    // last motor setting
  int idx;

  /* Startup kick state.
   * inKick is armed by applyWheelTarget() ONLY when a wheel starts from rest
   * or reverses direction, and is cleared by doPID once the encoder has
   * accumulated kickMinFwd/kickMinRev ticks, or KICK_TIMEOUT_CYCLES elapse.
   * kickCycles counts PID frames elapsed; kickTicksAccum accumulates encoder
   * ticks seen across all kick frames. */
  bool inKick;
  int  kickCycles;
  int  kickTicksAccum;

  /* Per-wheel kick exit thresholds, seeded from the KICK_MIN_TICKS_* defines
   * in resetPID(). Held per wheel rather than as globals because the two
   * wheels do not spin up at the same rate. */
  int  kickMinFwd;
  int  kickMinRev;

  /* Learned operating point: the output magnitude this wheel settled at last
   * time it was tracking, and the target it settled at.  Used to seed the PID
   * when the kick hands off, instead of starting from minOutput.
   *
   * Deliberately NOT cleared by resetPID() -- resetPID runs on every stop, and
   * clearing here would make the wheel relearn on every single move, which is
   * the whole problem this exists to solve.  Zero-initialised as globals, so
   * they start invalid and the first move falls back to DEFAULT_SEED_PWM. */
  long learnedFwd;
  long learnedRev;
  int  learnedFwdTarget;
  int  learnedRevTarget;
  bool learnedFwdValid;
  bool learnedRevValid;

  /* Consecutive frames spent inside LEARN_ERROR_BAND.  Transient state, so
   * unlike the learned values this IS cleared by resetPID(). */
  int  settledFrames;
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters.  These are only the pre-activation defaults: the ROS host
   overwrites them via the 'u' command in on_activate() (see the pid_p/pid_d/
   pid_i/pid_o params in diffbot.ros2_control.xacro), so tune them there. */
int Kp = 25;
int Kd = 10;
int Ki = 0;
int Ko = 40;

/* Floor on the drive magnitude once a wheel is moving.
 *
 * Lowered from 30 to 15 in v1.62.  With MR_PWM_OFFSET gone the right wheel
 * gets its full commanded duty, and bench runs showed the PID asking for
 * 26-29 every frame and being clamped back up to 30 -- the wheel held 10-11
 * ticks/frame against a target of 8 and the loop had no way down.  The floor,
 * not the motor, was the binding constraint.
 *
 * 30 was masked before by the offset subtracting 10 from whatever the right
 * wheel was commanded.
 *
 * This only needs to be high enough to avoid commanding a *moving* wheel below
 * the duty that sustains rotation; starting from rest is the kick's job, not
 * this one, so it can sit well below the stall threshold. */
int minOutput = 15;

/* Minimum encoder ticks that must be observed (accumulated across kick frames)
   before the kick phase ends and PID takes over.  A HIGHER value keeps that
   wheel at full PWM for longer, because more ticks have to accumulate before
   it hands off.

   Both wheels now use 5 forward.  The asymmetric 8/3 dated from when the left
   wheel was dragging; with the ball roller fitted the two are within ~13% of
   each other and the split had become the dominant error source.

   5 is chosen from measured accumulation, not guessed.  Across six kick
   sequences the wheels held 2-3 ticks at cycle 2 and 7-9 at cycle 3, so a
   threshold anywhere in 4..6 sits in the empty band and both wheels exit
   deterministically on cycle 3.  Thresholds inside those ranges make the exit
   frame turn on a single tick: bench runs where the two exited together
   finished +2 ticks apart, and runs where they exited a frame or two apart
   finished +6 and +14.

   Reverse is set to 5 to match, but that is an untested guess -- the old 12/7
   were compensating for the minOutput clamp bug, which is long fixed.  Reverse
   has never been run, so repeat the cycle-by-cycle tick check before trusting
   it.

   Forward and reverse are tuned separately because drivetrain stiction is
   asymmetric.

   NOTE: the reverse values were raised to compensate for a clamp bug that
   capped reverse PWM at minOutput (now fixed).  With reverse able to reach
   MAX_REV_PWM again, these probably want re-tuning down -- but that needs to
   be measured on the hardware.

   KICK_TIMEOUT_CYCLES still caps the kick, so a threshold set higher than a
   wheel can reach in 5 frames just means it kicks for the full timeout. */
#define KICK_MIN_TICKS_LEFT_FWD    5
#define KICK_MIN_TICKS_LEFT_REV    5
#define KICK_MIN_TICKS_RIGHT_FWD   5
#define KICK_MIN_TICKS_RIGHT_REV   5

/* Maximum PID frames to stay in kick before handing off regardless of ticks,
   to prevent hanging indefinitely if an encoder is broken.  At PID_RATE = 4 Hz
   this is 5 * 250 ms = 1.25 s. */
#define KICK_TIMEOUT_CYCLES 5

/* Bound on the integral accumulator.  Ki defaults to 0 so this is inert today,
   but it stops ITerm running away if integral action is ever enabled. */
#define ITERM_MAX 400

/* Learned-operating-point tuning.

   LEARN_ERROR_BAND: |Perror| at or below this counts as "settled", and the
   output is recorded.  Wide enough to keep encoder glitches out -- a spurious
   tick count produces an error far larger than this.

   Widened from 1 to 2 in v1.68.  At 1 there was a chicken-and-egg: a wheel
   converging toward its operating point sits at |pe| = 2 for exactly the
   frames where its output is becoming informative, so every one of them reset
   the settle counter and the wheel could only learn after it had already
   converged -- often after the run had ended.  One bench run saw the right
   wheel reach settledFrames = 3 on the final frame and store nothing, leaving
   the next run to start cold and finish +13 ticks out.  Capturing at |pe| = 2
   stores a value part-way down the descent, which filterToward() then walks
   the rest of the way on the following run.

   LEARN_TARGET_BAND: only seed from a learned value if the new target is this
   close to the one it was learned at.  The output a wheel needs is a function
   of speed, so a value learned at 8 ticks/frame says nothing useful about a
   move at 3. */
#define LEARN_ERROR_BAND   2
#define LEARN_TARGET_BAND  2

/* Consecutive in-band frames required before the operating point is recorded.
   Note the counter is incremented before the first capture, so this value of 3
   means four consecutive in-band frames are needed to store anything.
   Capturing on any single in-band frame picked up outputs that were still
   mid-transient -- the stored left value bounced 46/52/42 between runs, and
   each stale seed drove an overshoot on the next run.  Requiring a run of
   settled frames keeps transients out of it. */
#define LEARN_SETTLE_FRAMES 3

/* Seed used before anything has been learned, i.e. the first move after power
   up.  Deliberately not minOutput: dropping from the kick's PWM straight to 15
   nearly stalled the left wheel before the loop could climb back.  A mid-range
   guess costs one slightly-off move and is then replaced by a learned value. */
#define DEFAULT_SEED_PWM   35

unsigned char moving = 0; // is the base in motion?

/* Integer divide with round-to-nearest (ties away from zero).

   Plain integer division truncates toward zero, so with the host's gains
   (Kp=30, Ko=50) a one-tick-per-frame error produced a P term of exactly 0 --
   a silent deadband across a large part of the operating range, since typical
   targets are only single-digit ticks per frame.  The D term was worse: it
   needed an error change of 5 before registering at all. */
static long divRound(long num, long den)
{
  if (den == 0) return 0;              /* guard: 'u' can set Ko to 0 */
  long half = (den > 0 ? den : -den) / 2;
  return (num >= 0) ? (num + half) / den : (num - half) / den;
}

/* Apply the speed governor to BOTH wheels together, scaling rather than
   clamping.

   The ratio between the two targets *is* the commanded turn. Clamping each
   wheel independently changes that ratio, and when both exceed the limit it
   flattens them to the same value -- so the robot drives straight through a
   commanded curve.

   Observed on the robot: nav asked for 20/12 (a gentle right), the per-wheel
   clamp made it 8/8, the base went straight, heading error grew, nav escalated
   its angular command to saturation, and then one wheel finally fell below the
   limit and the whole accumulated correction happened in one frame. Drift,
   snap, overshoot, snap back. It was invisible on the bench because every test
   used equal targets (`m 30 30`), where clamping and scaling agree.

   Scaling preserves the ratio: 20/12 becomes 8/5.

   Resolution is coarse at MAX_TICKS_PER_FRAME = 8 -- the tightest expressible
   curve ratio is 1/8 -- so a very tight turn quantises, and a wheel whose share
   rounds to 0 is then held still by the per-wheel zero handling in doPID(),
   which is the right rendering of a near-pivot. */
void governSpeedPair(long *left, long *right)
{
  long magL = (*left  >= 0) ? *left  : -*left;
  long magR = (*right >= 0) ? *right : -*right;
  long peak = (magL > magR) ? magL : magR;

  if (peak <= MAX_TICKS_PER_FRAME)
    return;

  *left  = divRound(*left  * MAX_TICKS_PER_FRAME, peak);
  *right = divRound(*right * MAX_TICKS_PER_FRAME, peak);
}

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/

void resetPID(){
   leftPID.TargetTicksPerFrame = 0;
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
   leftPID.settledFrames = 0;
   leftPID.kickMinFwd = KICK_MIN_TICKS_LEFT_FWD;
   leftPID.kickMinRev = KICK_MIN_TICKS_LEFT_REV;

   rightPID.TargetTicksPerFrame = 0;
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
   rightPID.settledFrames = 0;
   rightPID.kickMinFwd = KICK_MIN_TICKS_RIGHT_FWD;
   rightPID.kickMinRev = KICK_MIN_TICKS_RIGHT_REV;
}

/* Apply a new target to one wheel, arming the startup kick only when it is
   actually needed.

   The kick exists to break stiction when a wheel starts moving.  Re-arming it
   on every MOTOR_SPEEDS command -- as the previous version did -- meant that a
   host streaming commands faster than PID_RATE kept inKick permanently true,
   so doPID took the kick branch and returned early on every single frame and
   the PID never ran at all.  The base then behaved as a two-level bang-bang
   controller that ignored the requested speed magnitude entirely.

   A zero target is handled per wheel rather than only for the pair, so that a
   pivot ("hold this wheel, drive the other") does not leave a wheel with
   target 0 running through the kick path, where (target > 0) is false and the
   wheel would be driven at MAX_REV_PWM. */
void applyWheelTarget(SetPointInfo *p, long target, char motor_dir)
{
  /* No governing here any more. Speed limiting is applied to both wheels
     together, before this is called, by governSpeedPair() -- a per-wheel clamp
     silently destroys the commanded turn. */
  p->TargetTicksPerFrame = target;

  if (target == 0) {
    p->inKick         = false;
    p->kickCycles     = 0;
    p->kickTicksAccum = 0;
    p->output         = 0;
    p->ITerm          = 0;
    return;
  }

  bool wasStopped = (motor_dir == DIR_STOPPED);
  bool reversing  = (target > 0 && motor_dir == DIR_BWD) ||
                    (target < 0 && motor_dir == DIR_FWD);

  if (wasStopped || reversing) {
    p->inKick         = true;
    p->kickCycles     = 0;
    p->kickTicksAccum = 0;
  }
}

/* Pick the output value to hand the PID when the startup kick finishes.

   Without this the seed was always minOutput.  Measured on the bench, the
   left wheel needs an output near 52 to hold 8 ticks/frame while the right
   needs about 35 -- so seeding both at 30 left the left wheel climbing one
   count per frame for ~22 frames, longer than a typical move.  It spent the
   whole move a tick per frame slow while the right sat a tick per frame fast,
   which is roughly 2 ticks of heading error per frame.

   Seeding from the point the wheel last settled at removes that climb.  Falls
   back to minOutput when nothing has been learned yet for a comparable
   target. */
long kickHandoffSeed(SetPointInfo *p)
{
  long target = p->TargetTicksPerFrame;
  long seed;

  if (target > 0) {
    if (p->learnedFwdValid &&
        abs((int)target - p->learnedFwdTarget) <= LEARN_TARGET_BAND)
      seed = p->learnedFwd;
    else
      seed = DEFAULT_SEED_PWM;
  } else {
    if (p->learnedRevValid &&
        abs((int)target - p->learnedRevTarget) <= LEARN_TARGET_BAND)
      seed = -p->learnedRev;
    else
      seed = -DEFAULT_SEED_PWM;
  }

  /* A learned value predates any change to the PWM limits, so clamp. */
  if (seed > MAX_FWD_PWM) seed = MAX_FWD_PWM;
  if (seed < MAX_REV_PWM) seed = MAX_REV_PWM;

  return seed;
}

/* One step of the learned-value filter: 7/8 old, 1/8 new, rounded.
 *
 * The rounding alone is not enough.  The correction per step is diff/8, so once
 * the stored value is within about 4 counts of the truth that correction is
 * less than half a count, integer division discards it, and the value parks
 * there permanently.  Measured on the bench: the right wheel settled at output
 * 30 while its learned value sat stuck at 34, seeding a 4-count overshoot into
 * every start.
 *
 * So when the filter stalls, move one count regardless.  Convergence then takes
 * a few settled frames instead of never, and the 7/8 term still damps anything
 * larger.  A dithering output makes this dither by +/-1, which is harmless. */
static long filterToward(long stored, long target)
{
  long next = (7 * stored + target + 4) / 8;

  if (next == stored && target != stored)
    next += (target > stored) ? 1 : -1;

  return next;
}

/* Record the output a wheel settles at, so the next move can start there.
   Only called from the normal PID path, only while the error is inside
   LEARN_ERROR_BAND, and never while saturated -- a clamped output is not the
   operating point, it is just the ceiling.  Lightly filtered so one odd frame
   cannot move the stored value far. */
void learnOperatingPoint(SetPointInfo *p, long output, long Perror)
{
  long mag = (output >= 0) ? output : -output;

  /* Any excursion outside the band means we are not settled; start counting
     again. */
  if (abs(Perror) > LEARN_ERROR_BAND) {
    p->settledFrames = 0;
    return;
  }

  /* In band, but not for long enough yet to call this the operating point --
     a transient passing through zero error looks identical on one frame. */
  if (p->settledFrames < LEARN_SETTLE_FRAMES) {
    p->settledFrames++;
    return;
  }

  /* Heavily filtered (7/8 old) so a single settled run cannot yank the stored
     value far.  The plant itself varies run to run, and chasing that variance
     is what produced the overshoot-undershoot oscillation. */
  if (p->TargetTicksPerFrame > 0 && output > 0) {
    p->learnedFwd = p->learnedFwdValid ? filterToward(p->learnedFwd, mag) : mag;
    p->learnedFwdTarget = (int)p->TargetTicksPerFrame;
    p->learnedFwdValid = true;
  }
  else if (p->TargetTicksPerFrame < 0 && output < 0) {
    p->learnedRev = p->learnedRevValid ? filterToward(p->learnedRev, mag) : mag;
    p->learnedRevTarget = (int)p->TargetTicksPerFrame;
    p->learnedRevValid = true;
  }
}

/* PID routine to compute the next motor commands. */
void doPID(SetPointInfo * p, char motor_dir) {
  long Perror;
  long outputP;
  long outputD;
  long outputI;
  long output;
  long delta;

  delta = p->Encoder - p->PrevEnc;

  /* A zero target means this wheel is commanded to hold still even though the
     other wheel may still be driving.  Neither the kick path nor the PID path
     can represent "stopped" -- both floor the magnitude at minOutput -- so
     return before either of them can drive the wheel. */
  if (p->TargetTicksPerFrame == 0) {
    p->output    = 0;
    p->ITerm     = 0;
    p->PrevEnc   = p->Encoder;
    p->PrevErr   = 0;
    p->PrevDelta = delta;
    return;
  }

  Perror = p->TargetTicksPerFrame - delta;

  /* --- Startup kick phase ---
   *
   * Apply full power in the target direction until the encoder has accumulated
   * at least this wheel's kick threshold across one or more PID frames, confirming inertia
   * has been overcome.  KICK_TIMEOUT_CYCLES acts as a safety ceiling so we
   * never hang here if an encoder is faulty.
   *
   * On exit the PID's accumulated output is seeded from this wheel's learned
   * operating point where one is available, falling back to minOutput -- see
   * kickHandoffSeed().
   *
   * PrevEnc and PrevErr are synchronised at the moment of handoff so the
   * derivative term does not produce a spike on the first real PID frame.
   */
  if (p->inKick) {
    p->kickCycles++;
    p->kickTicksAccum += abs(delta);

    output = (p->TargetTicksPerFrame > 0) ? kickFwdPwm : kickRevPwm;

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

    int kickMinTicks = (p->TargetTicksPerFrame > 0) ? p->kickMinFwd : p->kickMinRev;

    // Exit kick when enough ticks have accumulated OR the timeout is reached
    if (p->kickTicksAccum >= kickMinTicks || p->kickCycles >= KICK_TIMEOUT_CYCLES) {
      p->inKick = false;
      // Seed from the learned operating point where we have one, else minOutput
      p->output = kickHandoffSeed(p);
      // Sync history to suppress a D-term spike on the first real PID frame
      p->PrevEnc = p->Encoder;
      p->PrevErr = 0;

      if (debugMode) {
        Serial.print("KICK ");
        Serial.print(p->idx);
        Serial.print(" done (ticks=");
        Serial.print(p->kickTicksAccum);
        Serial.print(") seed=");
        Serial.print(p->output);
        Serial.println((p->TargetTicksPerFrame > 0 ? p->learnedFwdValid
                                                  : p->learnedRevValid)
                       ? " (learned)" : " (default)");
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

  outputP = divRound((long)Kp * Perror, Ko);
  outputD = divRound((long)Kd * (Perror - p->PrevErr), Ko);
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

  /* Enforce a minimum drive magnitude in the commanded direction: below
     minOutput the motor stalls rather than turning.

     Both branches must raise the magnitude up to minOutput.  The previous
     reverse test was `output < -minOutput`, which clamped reverse output
     *down* to -minOutput -- a ceiling rather than a floor.  That capped
     reverse PWM at 30, made MAX_REV_PWM (-45) unreachable outside the kick
     phase, and left the `output <= MAX_REV_PWM` test below as dead code. */
  if (output < 0)
  {
     if (output > -minOutput)
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
    // in the target direction rather than slamming to MAX.  The target is
    // known non-zero here, since a zero target returned above.
    saturated = true;
    output = (p->TargetTicksPerFrame > 0) ? minOutput : -minOutput;
  }

  // Only accumulate ITerm when not saturated (anti-windup)
  if (!saturated) {
    /* Accumulate in long and clamp before storing: Ki * Perror is a long, so
       `p->ITerm += ...` would truncate into the int accumulator before the
       clamp could catch it. */
    long iterm = (long)p->ITerm + (long)Ki * Perror;
    if (iterm >  ITERM_MAX) iterm =  ITERM_MAX;
    if (iterm < -ITERM_MAX) iterm = -ITERM_MAX;
    p->ITerm = (int)iterm;
  }

  p->output = output;
  p->PrevDelta = delta;

  /* Remember this operating point once the wheel is actually tracking, so the
     next move can start here instead of climbing to it. Skipped while
     saturated -- a clamped output is the ceiling, not the operating point. */
  if (!saturated) {
    learnOperatingPoint(p, output, Perror);
  }
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
    Serial.print(rightPID.Encoder);
    /* Rejected-edge counts: if a frame shows an implausible tick jump AND
       these are climbing, the extra counts were closely-spaced noise. If they
       stay flat through a spike, the cause is elsewhere. */
    Serial.print(" rej ");
    Serial.print(encRejectedLeft);
    Serial.print("/");
    Serial.println(encRejectedRight);
  }

  /* Compute PID update for each motor, passing each motor's own direction. */
  doPID(&rightPID, dir_right);
  doPID(&leftPID, dir_left);

  /* When the wheels are spinning in opposite directions (turning on the spot)
   * friction is higher than during straight travel.  Apply SPIN_BOOST to the
   * magnitude of both outputs, clamped to the existing PWM limits.
   *
   * This is pure feed-forward: it is applied to the local values handed to the
   * motors and deliberately NOT written back into p->output.  p->output is the
   * PID's accumulator and is used as the base for the next frame's output, so
   * folding the boost into it re-applied the boost on top of the already
   * boosted value every frame -- a ratchet that pinned the output at the PWM
   * limit and disabled speed control for the duration of a spin. */
  long leftOut  = leftPID.output;
  long rightOut = rightPID.output;

  if ((dir_left == DIR_FWD && dir_right == DIR_BWD) ||
      (dir_left == DIR_BWD && dir_right == DIR_FWD)) {
    if (leftOut > 0)
      leftOut = min((long)MAX_FWD_PWM, leftOut + SPIN_BOOST);
    else if (leftOut < 0)
      leftOut = max((long)MAX_REV_PWM, leftOut - SPIN_BOOST);

    if (rightOut > 0)
      rightOut = min((long)MAX_FWD_PWM, rightOut + SPIN_BOOST);
    else if (rightOut < 0)
      rightOut = max((long)MAX_REV_PWM, rightOut - SPIN_BOOST);

    if (debugMode) {
      Serial.print("SPIN BOOST l=");
      Serial.print(leftOut);
      Serial.print(" r=");
      Serial.println(rightOut);
    }
  }

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftOut, rightOut);
}
