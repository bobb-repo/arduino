#include <TMC2209.h>

#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <stdio.h>
#include <LibPrintf.h>
#include <VL53L1X.h>

#define VERSION " 2.13"

#define USE_TIMER_1 false
#define USE_TIMER_2 true
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 false

#include "TimerInterrupt.h"

/* GPIO Assignments -- BOARD REVISION r3-2.
 *
 * arm11 and earlier are pinned for r3-1, which is wired differently; this is the
 * reason arm12 exists. Every number below was taken from r3-2.net (Eeschema
 * 8.0.0 export) rather than carried over. The differences from r3-1 that matter:
 *
 *   ST2's DIR/STEP/EN moved from 3/19/18 to 22/23/24. On r3-2, 18 and 19 are
 *   ST1's UART pair, so the old map actively collides with serial on ST1.
 *   The solenoids moved from 10/11 to 39/40 (10/11 are unconnected on r3-2).
 *   SV1_A/SV2_A moved up one, from A10/A11 to A11/A12; on r3-2 A10 is the
 *   vacuum sensor's second pin, so the old map read the vacuum on servo 1.
 *   The six ST*_UART_TX/RX pins are gone. Each driver now has its own hardware
 *   UART plus a separate PDN pin -- see the ST*_PDN / serial notes below.
 */

#define ST1_DIR     6
#define ST1_STEP    7
#define ST1_ENABLE  8

#define ST2_DIR     22
#define ST2_STEP    23
#define ST2_ENABLE  24

#define ST3_DIR     29
#define ST3_STEP    30
#define ST3_ENABLE  31

/* Driver PDN, module pad 13 ("PDN" on the schematic).
 *
 * NO LONGER DRIVEN AS A GPIO. On a TMC2209 the PDN and UART pads are two
 * connections to the same chip pin, PDN_UART (SilentStepStick datasheet rev
 * 1.20, pin list 11/12 and the note "PDN/UART ... optional UART interface"), and
 * TMC2209::setup() sets GCONF.pdn_disable=1 to take the pin over for serial.
 * Driving it from here would be the MCU fighting the driver on the same wire.
 *
 * What the pin used to do -- HIGH for full current while active, LOW to let the
 * driver reduce current at standstill -- is now IHOLD/IRUN, see tmcConfigure().
 * The defines stay so the pin is documented as spoken for, not free.
 */
#define ST1_PDN     9
#define ST2_PDN     25
#define ST3_PDN     33

/* Driver DIAG output -- real connections, made with physical wires to these
 * header holes. They do not appear as driver-side nodes in r3-2.net because the
 * only available A4988 symbol has no DIAG pin, so the schematic shows each net
 * reaching the header and stopping. Do not "fix" that by deleting these.
 *
 * Asymmetry that matters for anything built on DIAG later: ST1 and ST2 land on
 * INT0/INT1 and can raise an interrupt; pin 32 has no interrupt on a Mega, so
 * ST3's DIAG can only ever be polled.
 */
#define ST1_DIAG    2
#define ST2_DIAG    3
#define ST3_DIAG    32

#define SOL1        39
#define SOL2        40

#define X_GP_1     43
#define X_GP_2     44
#define X_GP_3     45
#define X_GP_4     46
#define X_GP_5     47

#define SV1_DO      4
#define SV2_DO      5

// Analog Assignments

#define SV1_A      A11
#define SV2_A      A12

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
/* 'm' is NOT a command. It is left undefined on purpose so the host's
   arm_set_servo() gets ERR-CMD instead of silently changing motor currents.
   Do not reuse this letter. See the note where jointPdn() used to be. */
#define DRIVER_STATUS     'i'   /* TMC2209 fault flags, see tmcReportStatus() */
#define MOTOR_CURRENT     'n'   /* "n" reports mA, "n <run> <hold>" sets it */
#define STANDSTILL_MODE   't'   /* "t" reports, "t <joint> <0-3>" sets it */

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
    /* Was uart_tx/uart_rx on r3-1, where both were driven together as the
     * power-down control. r3-2 brings PDN out on its own pin and puts the UART
     * on a hardware serial port, so this is one PDN pin plus DIAG. The field
     * COUNT is deliberately unchanged: the initialisers below are positional.
     */
    char pdnGpio;
    char diagGpio;
    
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

/* ------------------------------------------------------------------------
 * TMC2209 UART configuration
 *
 * Each driver has its own hardware serial on r3-2 (ST1 Serial1, ST2 Serial2,
 * ST3 Serial3), with the MCU TX joined to RX through a 1k resistor -- the
 * "bidirectional coupled" wiring in the library README. MS1 and MS2 are both
 * grounded on all three, so every driver answers at address 0 and there is no
 * addressing to do.
 *
 * Three things TMC2209::setup() changes that would quietly break this arm, all
 * undone in tmcConfigure():
 *
 *  1. GCONF.mstep_reg_select=1 takes microstepping off the MS1/MS2 pins and onto
 *     the register, whose default MRES=0 means 256 microsteps. The pins say 8.
 *     ticksPerDegree is calibrated for 8, so leaving this alone would make every
 *     move 1/32 of what was asked for.
 *  2. GCONF.i_scale_analog=0 stops the Vref trimmer setting motor current and
 *     hands it to IRUN/IHOLD. enableAnalogCurrentScaling() puts the pot back in
 *     charge, so run current stays whatever it is set to on the bench today and
 *     this change does not quietly re-tune the motors.
 *  3. minimizeMotorCurrent() then disable() leave TOFF=0 -- driver off. enable()
 *     restores it. Note the library's enable()/disable() only touch TOFF here,
 *     because hardware_enable_pin_ is left unset; the EN pins stay under
 *     setMotorState()'s control exactly as before.
 */
#define STEPPER_MICROSTEPS      8   /* must match the MS1/MS2 strapping (both GND) */

/* 0 = SpreadCycle (torque at speed), 1 = StealthChop (quiet, weaker at speed).
   See the chopper notes in tmcConfigureOne(). */
#define TMC_USE_STEALTHCHOP     0
#define TMC_SERIAL_BAUD         115200

/* Standstill current as a percentage of run current, and how long to wait after
 * the last step before falling back to it. This is the power-down that the PDN
 * pin used to do crudely, and unlike the pin it is a level rather than on/off.
 * HOLD_PERCENT is deliberately not aggressive: too low and a loaded joint sags
 * when it stops. Raise it if a joint drifts at rest, lower it for less heat.
 */
/* Motor current, in milliamps RMS per phase.
 *
 * Set in the driver's registers, NOT with the VREF trimmer. TMC2209::setup()
 * leaves GCONF.i_scale_analog at 0, which per the datasheet means "use internal
 * reference derived from 5VOUT" -- the VREF pin, and therefore the pot at R9, is
 * ignored. Do not call enableAnalogCurrentScaling(); that hands control back to
 * the pot and makes the numbers here meaningless.
 *
 * The trade that was made deliberately: the pot used to be a hardware ceiling no
 * firmware bug could exceed. It no longer is, so keep the default conservative.
 * Full scale on this board is 1768 mA RMS (CS=31, VSENSE=0) and nothing stops a
 * bad value reaching it.
 *
 * setRMSCurrent() picks the VSENSE range itself: below about 800 mA it switches
 * to the high-sensitivity range, which gives finer steps at low current.
 */
#define TMC_SENSE_RESISTOR      0.11f  /* R3/R5 = 0R11, BTT TMC2209 V1.3 */
#define TMC_CURRENT_MAX_MA      1768   /* CS=31 at VSENSE=0; refuse beyond this */
#define TMC_HOLD_DELAY_PERCENT  20

/* Startup defaults, loaded into RAM by tmcLoadDefaultCurrents() before any
 * driver is configured, so the arm is never briefly at whatever the registers
 * happened to contain.
 *
 * Sized from the motors: NEMA 17 bipolar, 1.0 A per phase, 16 Ncm. setRMSCurrent
 * takes RMS, and 1000/sqrt(2) = 707 mA keeps the PEAK phase current at the
 * motor's rating rather than 1.41x over it. The host overrides these per joint
 * from cobo.ros2_control_macro.xacro on every on_activate(); they apply when the
 * firmware runs standalone, on the bench or with the stack down.
 *
 * MEASURED 2026-09-14, joint 0, 190 ticks (10 deg, expect ~31 AV counts):
 *     500 mA   CW  -7   CCW +39     <- stalling, CW could not lift the load
 *     900 mA   CW -38   CCW +44
 *    1300 mA   CW -33   CCW +38     <- no better than 900, past the knee
 * So 500 was too low. Holding was never the problem -- it held with zero drift
 * at 250 mA -- because a static hold needs far less torque than accelerating a
 * load. Judge current by whether a move COMPLETES, not by whether it holds.
 *
 * Low is not automatically safe on this arm: too little current stalls a move
 * and can let a joint be back-driven. Do not drop these expecting "safer".
 */
#define TMC_RUN_CURRENT_MA      707
#define TMC_HOLD_CURRENT_MA     350

/* Standstill behaviour per joint, set by the 't' command.
 *
 * Freewheeling and the two braking modes are ONE register field (CHOPCONF
 * FREEWHEEL), not separate features, so they get one command rather than two --
 * two commands writing the same field could contradict each other.
 *
 * The catch that makes them easy to "try" and wrongly conclude they do nothing:
 * datasheet 6.7 says the FREEWHEEL setting only takes effect when **IHOLD is
 * zero**. So selecting any mode other than NORMAL must also drop IHOLD to 0,
 * and returning to NORMAL must put it back. tmcApplyStandstill() does both.
 *
 * What they actually do, which matters for an arm whose joints need current to
 * hold: only NORMAL produces a restoring force at zero speed. Braking is a
 * DAMPER, not a spring -- the datasheet is explicit that "passive braking will
 * allow slow turning of the motor when a continuous torque is applied", and
 * gravity on a loaded joint is exactly that. Braking makes a joint fall slowly;
 * it does not hold it.
 */
#define TMC_SS_NORMAL     0   /* hold current, the only mode that holds position */
#define TMC_SS_FREEWHEEL  1   /* coils open, free fall */
#define TMC_SS_BRAKE_LS   2   /* coils shorted low-side, damped fall */
#define TMC_SS_BRAKE_HS   3   /* coils shorted high-side, damped fall */

uint8_t tmcStandstill[MOTORS_DEFINED];

/* Live values, per joint, changed by the 'n' command. IHOLD_IRUN is write-only
   on the TMC2209, so what was last written cannot be read back off the chip --
   these are the only record of it. Per joint because the three carry very
   different loads; the shoulder wants more than the wrist. */
uint16_t tmcRunMa[MOTORS_DEFINED];
uint16_t tmcHoldMa[MOTORS_DEFINED];

/* How long after the last step before the driver drops to hold current.
 * TPOWERDOWN units are 2^18 clocks, so at the 12 MHz internal oscillator one
 * unit is about 22 ms; 20 is roughly 0.44 s. Set explicitly rather than left to
 * the library default because this is the knob that decides whether a joint
 * cools down between moves or sags during a pause between them.
 */
#define TMC_POWER_DOWN_DELAY    20

TMC2209 stepperDriver[MOTORS_DEFINED];
HardwareSerial *stepperSerial[MOTORS_DEFINED] = { &Serial1, &Serial2, &Serial3 };
char tmcReady[MOTORS_DEFINED] = { 0, 0, 0 };

/* The IOIN version byte a TMC2209 answers with. The library has this constant
   but keeps it private, so it is repeated here rather than reached for. */
#define TMC_EXPECTED_VERSION  0x21

/* How the UART link failed, which is the difference between a wiring fault and a
   signal fault. Every check the library offers is a register READ, so none of
   them can pass spuriously -- but a failure on its own does not say which
   direction is broken, and that is what costs time at the bench. The version
   byte separates the cases. */
#define TMC_LINK_OK           0   /* replied, right version, serial mode confirmed */
#define TMC_LINK_NO_REPLY     1   /* nothing came back: read() timed out, returns 0 */
#define TMC_LINK_BAD_VERSION  2   /* bytes arrived but garbled: baud or signal integrity */
#define TMC_LINK_NOT_SETUP    3   /* talks, but GCONF.pdn_disable never took */

/* The idle EN timeout is off: leaveEnabled stays 1, idleEnableOnTimeMs is 0.
 *
 * That timeout dropped the EN pin ~30 s after a joint entered MOTOR_HOLD. It
 * existed because EN was the only power lever before UART worked, and it is
 * all-or-nothing: it removes the last 25% of standstill power by removing 100%
 * of the holding torque. These motors need current to hold, so a loaded joint
 * simply fell, uncommanded, with the host never told. IHOLD replaces it and
 * does the job properly -- continuous, per joint, and it still holds position.
 *
 * TURN IT OFF WITH idleEnableOnTimeMs, NOT leaveEnabled. The two read alike and
 * are not. leaveEnabled is tested in THREE places, and only one of them is the
 * timeout:
 *   - end of a move: if (leaveEnabled) HOLD else DISABLED
 *   - the idle timer: if (leaveEnabled) { if (expired) DISABLED }
 *   - ARM_LOCK, until it was corrected on 2026-09-13
 * So leaveEnabled = 0 does not disable the timeout, it de-energises the joint
 * at the end of EVERY move, and it made "k 1" (lock) disable the motors. Both
 * were introduced and caught here on 2026-09-13, before motors were connected.
 *
 * idleEnableOnTimeMs = 0 is read as "never" by the timer, which is the single
 * value to change if a deliberate low-power idle is ever wanted -- though
 * prefer the 't' command's braking mode, which damps the fall rather than
 * allowing a free one.
 */
/* LEFT ARM joint 1 position sense: TRIED A3, REVERTED to A1. 2026-09-16.
 *
 * A1 (ST2_POS) reads a stable 0-4 across 120 deg of travel in both directions
 * with the joint nowhere near a stop, so joint 1's pot signal is not reaching
 * the ADC. A3 (ST1_POS_BU) was tried as an alternative input and MUST NOT be
 * used: measured on hardware, moving joint 0 by 150 ticks changed A3 by +46
 * counts -- exactly tracking A0 -- while moving joint 1 changed it by 1-6
 * counts, i.e. noise. A3 is on the /ST1-POS net with A0, as r3-2.net says.
 *
 * It read a plausible moving number, which is why this needed a real test: with
 * invertedPosition = 1 the reported value even LOOKED independent (j0 raw 561,
 * j1 reported 463, and 1023-463 = 560). Undoing the inversion showed both pins
 * sitting on the same volts. "It reads a number" is not evidence; the
 * discriminator is whether it tracks its OWN joint and not the other one.
 *
 * There is no firmware workaround. A4 (ST2_POS_BU) is on the SAME net as A1, so
 * it is not a spare input, and A3/A5 belong to joints 0 and 2. The fix is
 * physical: joint 1's pot at J1 pin 14, most likely its +5V lead, since a
 * floating wiper on a high-Z input reads erratically rather than a stable 0.
 */
/* LEFT ARM calibrated on hardware 2026-09-17. Joints 0 and 1 from marks set by
 * hand at 90 and 270 deg; joint 2 from 'span', which creeps onto its actual end
 * stops -- only joint 2 stops at 90/270, so only it can be done without a human
 * reference. Every mark was taken with a 1-count spread.
 *
 * The three slopes agree within 3% (3.156 / 3.089 / 3.072), and joint 2's came
 * from physical stops with no judgement involved, which corroborates the two
 * hand-marked joints. Contrast the right arm, whose slopes spread much wider
 * (2.750 / 2.617 / 2.983) -- the arms are NOT interchangeable, and neither
 * arm's numbers may be copied to the other.
 *
 * Read the identification note in arm_calib.py before recalibrating either arm.
 * Two sessions were lost to calibrating the wrong one: port names cannot
 * identify an arm, because udev maps by USB socket and ttyUSB<n> moves on any
 * replug. The left arm has no VL53L1X fitted, so sensor.init() fails and the
 * library prints "f0" at startup. That marker follows the BOARD, and the board
 * is bolted to its arm.
 *
 * ticksPerDegree is unmeasured on this arm and left at its original values.
 * They only size the tick budget for a move that terminates on AV, so a value
 * that is too high costs nothing while one too low truncates the move short.
 */
MOTOR leftMotors[MOTORS_DEFINED] = { {ST1_ENABLE,ST1_DIR,ST1_STEP,ST1_POS,ST1_PDN,ST1_DIAG,19.0,3.156,90,216, 784, 0,1,1,0,MOTOR_IDLE,0},
                                     {ST2_ENABLE,ST2_DIR,ST2_STEP,ST2_POS,ST2_PDN,ST2_DIAG,15.9,3.089,90,187 ,743 ,1,1,1,0,MOTOR_IDLE,0},
                                     {ST3_ENABLE,ST3_DIR,ST3_STEP,ST3_POS,ST3_PDN,ST3_DIAG,18.0,3.072,90,201, 754, 0,1,1,0,MOTOR_IDLE,0} };
                                     
MOTOR_LIMITS leftMotorLimits[MOTORS_DEFINED] = {  {90,270},{90,270},{90,270} }  ;

/* Joints 0 and 1 measured on the bench 2026-09-14 with arm_calib.py, two marks
 * each at 90 and 270 degrees (the 180 mark was eyeballed and discarded). The
 * fits are exact by construction; linearity is not assumed from two points but
 * was established separately by an automated sweep -- equal 300-tick moves gave
 * 74 +/- 6 AV counts all the way across AV 251..834.
 *
 * What the old numbers actually were: avAtMinDegree/avAtMaxDegree are DEFINED as
 * the AV at minDegree and at minDegree+180 -- that is what the conversion at
 * "currentPosition =" below depends on. The previous values were the readings at
 * the MECHANICAL STOPS, which on this arm sit outside 90..270, so every angle
 * the firmware reported was scaled against a span ~190 degrees wide. That is why
 * one slope (3.08) appeared to serve all six joints, and why neither joint's old
 * pair was self-consistent: 220 + 180*3.08 = 774, not the 804 stored beside it.
 *
 * ticksPerDegree is deliberately left ABOVE the measured value (joint 0 measures
 * ~17.9 from a median 6.5 ticks/AV against the new 2.750 slope). It only sizes
 * the tick budget for a move that terminates on AV anyway, so too high costs
 * nothing and too low truncates the move short of target.
 *
 * Joint 2 was done differently, and is the better measurement: its end stops
 * ARE 90 and 270, so arm_calib.py's 'span' creeps onto each stop and reads the
 * AV there -- two physical references rather than two judged angles. Both ends
 * showed the taper of a genuine stop (a 22-count step, then 4) rather than the
 * abrupt halt of a collision, which is what a joint hitting the tower looks
 * like. Do NOT use 'span' on joints 0 or 1: their stops are outside 90..270 and
 * it would silently return a span for the wrong angles.
 *
 * Joint 2 corroborates the whole diagnosis: because its stops really are at
 * 90/270, stop-to-stop and 90-to-270 are the same measurement for this joint
 * alone, and its old numbers came out nearly right (3.08 vs 2.983, 217 vs 222)
 * while joints 0 and 1 were far off. Its ticksPerDegree measured 16.7 by a
 * two-size move that cancels TICKS_PADDING; the stored 15.9 was BELOW that,
 * which is the truncating direction, so it is now 18.0.
 */
MOTOR rightMotors[MOTORS_DEFINED] = { {ST1_ENABLE,ST1_DIR,ST1_STEP,ST1_POS,ST1_PDN,ST1_DIAG,19.0,2.750,90,253, 748 ,1,0,1,0,MOTOR_IDLE,0},
                                      {ST2_ENABLE,ST2_DIR,ST2_STEP,ST2_POS,ST2_PDN,ST2_DIAG,15.9,2.617,90,248, 719 ,0,0,1,0,MOTOR_IDLE,0},
                                      {ST3_ENABLE,ST3_DIR,ST3_STEP,ST3_POS,ST3_PDN,ST3_DIAG,18.0,2.983,90,222, 759, 1,0,1, 0,MOTOR_IDLE,0}  };

MOTOR_LIMITS rightMotorLimits[MOTORS_DEFINED] = {  {90,270},{90,270},{90,270} }  ;

MOTOR *motors;
MOTOR_LIMITS *motorLimits;

// Exponential ramp parameters (replaces skipIntervalTable)
/* Top speed, as ticks skipped between steps. The ISR runs at 2 ms, so 0 is one
   step per tick = 500 steps/s, 1 is 250 steps/s, 2 is ~167 steps/s.
 *
 * Raised to 1 on 2026-09-14 while chasing joint 0 losing ~85% of its steps on
 * long moves, then PUT BACK TO 0 the same day: halving the speed barely helped
 * (48 -> 60 AV counts of 412), because the cause was not speed at all. It was
 * StealthChop running with pwm_autoscale disabled -- see the chopper notes in
 * tmcConfigureOne(). Do not reach for this constant to fix a stall without
 * ruling that out first.
 *
 * 0 gives one step per ISR tick. At TIMER_INTERVAL_MS 2 that is 500 steps/s,
 * about 26 deg/s at 19 ticks/degree, so a 120 deg move takes ~4.6 s. That is
 * the ceiling for this ISR rate; going faster means a shorter timer interval,
 * not a change here.
 *
 * This is a FLOOR, applied to the accel and decel curves as well -- those
 * compute delay from MAX_TICK_DELAY and would otherwise reach 0 whatever this
 * is set to.
 */
#define MIN_TICK_DELAY    0     // Cruise: one step every (this+1) ticks
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

/* How far outside a joint's calibrated AV span a reading may sit before it is
   treated as a sensor fault rather than a position. Wide enough to tolerate
   calibration drift and the few counts of noise measured at rest; narrow enough
   to catch a railed input, which reads near 0 or near 1023. See the check in
   moveToPosition().

   Raised 60 -> 120 on 2026-09-14, when joints 0 and 1 were recalibrated. The
   span avAtMinDegree..avAtMaxDegree is NOT the mechanical range -- it is 90..270
   degrees, and on joints 0 and 1 the stops lie outside it (joint 1's upper stop
   is roughly 34 degrees past 270). At 60 counts a joint parked on its own stop
   at power-up would have read as a sensor fault and refused to move. 120 counts
   is ~44 degrees on joint 0 and still catches a rail: the widest resulting band
   is joint 1's 128..839, and the ~1013 a failing pot produces is outside it.

   Joint 2 is the one joint that does not need the slack -- its stops ARE 90 and
   270, so the margin there covers only noise. It keeps it anyway so all three
   behave alike. */
#define AV_SANITY_MARGIN 120

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

/* Bring one driver up on its UART and put back everything setup() changed.
   Returns 1 if the driver answered, 0 if it did not. */
/* Load the compile-time defaults into RAM, ONCE per power-on. Called from
   setup() before tmcConfigure(), so every driver is brought up at a known
   current rather than whatever its registers held.

   The guard matters because "r" (RESET) calls setup() as a plain function --
   it re-runs initialisation in place, it does not reboot, so globals keep their
   values. Without the guard a mid-session reset would silently throw away the
   currents the host sent from cobo.ros2_control_macro.xacro at on_activate()
   and drop every joint back to the fallback below, while the host went on
   believing its own values were in force. A joint that quietly loses current is
   one that gets back-driven by its load.

   So: a real power-on gets the defaults, a soft reset re-applies whatever is
   actually configured. Anything wanting the defaults back should power-cycle,
   or send "n" explicitly. */
void tmcLoadDefaultCurrents(void)
{
    static char currentsLoaded = 0;
    int m;

    if (currentsLoaded)
    {
      return;
    }
    currentsLoaded = 1;

    for (m = 0; m < MOTORS_DEFINED; m++)
    {
      tmcRunMa[m]  = TMC_RUN_CURRENT_MA;
      tmcHoldMa[m] = TMC_HOLD_CURRENT_MA;

      /* Standstill is deliberately NOT behind the once-only guard: the brake
         and freewheel modes are for testing, and a joint must never be left
         unable to hold itself because someone reset without thinking. 'r' puts
         every joint back to normal hold. */
    }
}

void tmcLoadDefaultStandstill(void)
{
    int m;

    for (m = 0; m < MOTORS_DEFINED; m++)
    {
      tmcStandstill[m] = TMC_SS_NORMAL;
    }
}

/* Push this joint's tmcRunMa / tmcHoldMa into its driver.

   setRMSCurrent takes the hold level as a FRACTION of run, so the two are not
   independent settings on the chip -- IHOLD is derived from the same current
   scale IRUN uses. Expressing hold in mA here and converting keeps the command
   interface in one unit. */
void tmcApplyCurrent(int m)
{
    /* IHOLD must stay at zero while a freewheel/brake mode is selected, or the
       mode silently stops applying. So the configured hold current is only
       written when standstill is NORMAL; 't 0' is what puts it back. */
    float holdFraction = 0.0f;

    if (tmcStandstill[m] == TMC_SS_NORMAL && tmcRunMa[m] > 0)
    {
      holdFraction = (float)tmcHoldMa[m] / (float)tmcRunMa[m];
    }

    stepperDriver[m].setRMSCurrent(tmcRunMa[m], TMC_SENSE_RESISTOR, holdFraction);
    stepperDriver[m].setHoldDelay(TMC_HOLD_DELAY_PERCENT);
}

/* Select the standstill behaviour, and fix up IHOLD to match it. */
void tmcApplyStandstill(int m)
{
    switch (tmcStandstill[m])
    {
      case TMC_SS_FREEWHEEL:
        stepperDriver[m].setStandstillMode(TMC2209::FREEWHEELING);   break;
      case TMC_SS_BRAKE_LS:
        stepperDriver[m].setStandstillMode(TMC2209::STRONG_BRAKING); break;
      case TMC_SS_BRAKE_HS:
        stepperDriver[m].setStandstillMode(TMC2209::BRAKING);        break;
      default:
        stepperDriver[m].setStandstillMode(TMC2209::NORMAL);         break;
    }

    /* Order matters: the mode is chosen first, then the current that gates it. */
    tmcApplyCurrent(m);
}

const __FlashStringHelper *tmcStandstillName(uint8_t mode)
{
    switch (mode)
    {
      case TMC_SS_FREEWHEEL: return F("freewheel  (no holding, free fall)");
      case TMC_SS_BRAKE_LS:  return F("brake-ls   (damped, will still creep)");
      case TMC_SS_BRAKE_HS:  return F("brake-hs   (damped, will still creep)");
      default:               return F("normal     (hold current, holds position)");
    }
}

void tmcReportStandstill(void)
{
    int m;

    for (m = 0; m < MOTORS_DEFINED; m++)
    {
      Serial.print(F("joint "));
      Serial.print(m);
      Serial.print(F(" "));
      Serial.print(tmcStandstillName(tmcStandstill[m]));
      if (tmcStandstill[m] != TMC_SS_NORMAL)
      {
          Serial.print(F("  IHOLD forced 0"));
      }
      Serial.println();
    }
}

/* One line per joint, always all three -- a per-joint setting is easy to apply
   to the wrong index, and showing only the one just changed hides that. */
void tmcReportCurrent(void)
{
    int m;

    for (m = 0; m < MOTORS_DEFINED; m++)
    {
      Serial.print(F("joint "));
      Serial.print(m);
      Serial.print(F(" run "));
      Serial.print(tmcRunMa[m]);
      Serial.print(F(" mA hold "));
      Serial.print(tmcHoldMa[m]);
      Serial.print(F(" mA"));
      if (!tmcReady[m])
      {
          /* Stored but never written to the chip, so the figures above are what
             this joint WOULD use, not what it is using. */
          Serial.print(F("  NOT APPLIED - no UART"));
      }
      Serial.println();
    }
}

/* Bring one driver up and return its TMC_LINK_* state, with the version byte it
   answered with written through versionOut. */
int tmcConfigureOne(int m, uint8_t *versionOut)
{
    stepperDriver[m].setup(*stepperSerial[m], TMC_SERIAL_BAUD);

    /* Order matters: microsteps and current first, enable last, so the driver
       never sits enabled at a setting we did not choose. */
    stepperDriver[m].setMicrostepsPerStep(STEPPER_MICROSTEPS);

    /* Chopper mode. TMC2209::setup() leaves StealthChop selected AND turns off
       both pwm_autoscale and pwm_autograd, which is the worst combination: with
       autoscale off StealthChop drives a FIXED PWM amplitude, and PWM_GRAD is
       exactly what compensates velocity-dependent back-EMF. Effective current
       therefore collapses as speed rises.

       Measured 2026-09-14 before this was fixed: 10 deg moves were accurate and
       repeatable six times running, but a 2541-tick move issued every tick and
       turned the joint through 60 AV counts of the 412 asked for -- about 85%
       of steps lost. Short moves never reach cruise, so only long moves showed
       it, and the joint was confirmed free by hand over its whole range.

       SpreadCycle is the default because this arm carries load and needs torque
       at speed; StealthChop buys silence, which matters less. Autoscale and
       autograd are enabled either way so StealthChop behaves if selected. */
    stepperDriver[m].enableAutomaticCurrentScaling();
    stepperDriver[m].enableAutomaticGradientAdaptation();
#if TMC_USE_STEALTHCHOP
    stepperDriver[m].enableStealthChop();
#else
    stepperDriver[m].disableStealthChop();      /* SpreadCycle */
#endif

    tmcApplyStandstill(m);   /* sets the mode, then the current that gates it */
    stepperDriver[m].setPowerDownDelay(TMC_POWER_DOWN_DELAY);
    stepperDriver[m].enable();

    /* VACTUAL=0. Without this the driver would expect to be told a velocity over
       UART and would ignore the STEP pin entirely, which is not what this
       firmware does -- all motion stays on the step/dir interface and the ISR. */
    stepperDriver[m].moveUsingStepDirInterface();

    return tmcLinkState(m, versionOut);
}

/* Classify the UART link to one driver, and hand back the version byte it
   answered with so the caller can print it. One IOIN read, plus one GCONF read
   only if the version was right. */
int tmcLinkState(int m, uint8_t *versionOut)
{
    uint8_t version = stepperDriver[m].getVersion();

    if (versionOut)
    {
        *versionOut = version;
    }

    if (version == 0x00)
    {
        return TMC_LINK_NO_REPLY;
    }
    if (version != TMC_EXPECTED_VERSION)
    {
        return TMC_LINK_BAD_VERSION;
    }
    if (!stepperDriver[m].isSetupAndCommunicating())
    {
        return TMC_LINK_NOT_SETUP;
    }
    return TMC_LINK_OK;
}

/* Print why the link is not usable, with the evidence and where to look. Prints
   nothing for TMC_LINK_OK -- callers say what they want about a good link. */
void tmcPrintLinkFault(int state, uint8_t version)
{
    switch (state)
    {
      case TMC_LINK_NO_REPLY:
        Serial.print(F(" NO REPLY (version read 0x00) - TX not reaching the pad, "
                       "or nothing on RX"));
        break;

      case TMC_LINK_BAD_VERSION:
        Serial.print(F(" BAD VERSION 0x"));
        Serial.print(version, HEX);
        Serial.print(F(" - bytes arrived garbled, suspect baud or signal integrity"));
        break;

      case TMC_LINK_NOT_SETUP:
        Serial.print(F(" TALKS BUT NOT IN SERIAL MODE - GCONF write did not take"));
        break;
    }
}

/* Report one driver's fault flags. Returns 1 if anything is wrong.

   This reads DRV_STATUS over UART, which is a blocking round trip -- the library
   waits for the TX echo and then the reply. Do NOT call it from the timer ISR or
   between steps of a move. It is a command, not a poll: the host asks when it
   wants to know, so that adding this cannot perturb the step timing the way the
   'a' position polling once did.

   These are the same faults DIAG would raise on a pin, minus stalls: SGTHRS and
   TCOOLTHRS are both left at 0, so StallGuard never asserts and DIAG currently
   only ever means "driver fault". Reading them here gets the useful half without
   needing DIAG wired into an interrupt, and works for all three joints -- pin 32
   has no interrupt, so ST3 could never be done the other way. */
int tmcReportStatus(int m)
{
    int faults = 0;

    Serial.print(F("joint "));
    Serial.print(m);

    /* Check the link is alive BEFORE trusting any flag. TMC2209::read() returns
       0 when the driver does not reply, so a dead UART reads as every fault
       clear -- a fault reporter that answers "OK" because it cannot hear the
       driver is worse than none. This costs one extra round trip to make the
       answer mean something. */
    {
        uint8_t version;
        int link = tmcLinkState(m, &version);

        if (link != TMC_LINK_OK)
        {
            tmcPrintLinkFault(link, version);
            Serial.println();
            return 1;
        }
    }

    TMC2209::Status status = stepperDriver[m].getStatus();

    /* Temperature is graded, so report the hottest threshold reached rather than
       every flag below it -- 157C also sets 150/143/120 and a list is noise. */
    if (status.over_temperature_157c)      { Serial.print(F(" OVERTEMP-157C")); faults++; }
    else if (status.over_temperature_150c) { Serial.print(F(" OVERTEMP-150C")); faults++; }
    else if (status.over_temperature_143c) { Serial.print(F(" OVERTEMP-143C")); faults++; }
    else if (status.over_temperature_120c) { Serial.print(F(" WARM-120C"));     faults++; }

    if (status.over_temperature_shutdown) { Serial.print(F(" THERMAL-SHUTDOWN")); faults++; }
    if (status.short_to_ground_a)         { Serial.print(F(" SHORT-A"));          faults++; }
    if (status.short_to_ground_b)         { Serial.print(F(" SHORT-B"));          faults++; }

    /* Open load reads as set whenever the coil is not being driven, so it is
       only meaningful while the motor is enabled and moving. Reporting it from a
       standstill would cry wolf on every idle joint. */
    if (motors[m].motorState == MOTOR_ACTIVE)
    {
        if (status.open_load_a) { Serial.print(F(" OPEN-A")); faults++; }
        if (status.open_load_b) { Serial.print(F(" OPEN-B")); faults++; }
    }

    if (faults == 0)
    {
        Serial.print(F(" OK"));
    }
    Serial.println();

    return faults;
}

/* Called once from setup(), after the pins are configured. A driver that does
   not answer is reported and left alone: step/dir still works off the pins, so
   the arm keeps running without UART features rather than refusing to start. */
void tmcConfigure(void)
{
    int m;

    for (m = 0; m < MOTORS_DEFINED; m++)
    {
      uint8_t version = 0;
      int link = tmcConfigureOne(m, &version);

      tmcReady[m] = (link == TMC_LINK_OK) ? 1 : 0;

      Serial.print(F("* TMC2209 joint "));
      Serial.print(m);
      if (tmcReady[m])
      {
          Serial.print(F(" ok, "));
          Serial.print(STEPPER_MICROSTEPS);
          Serial.print(F(" microsteps, run "));
          Serial.print(tmcRunMa[m]);
          Serial.print(F(" mA hold "));
          Serial.print(tmcHoldMa[m]);
          Serial.println(F(" mA"));
      }
      else
      {
          tmcPrintLinkFault(link, version);
          Serial.println(F(" - step/dir only, no power-down"));
      }
    }
}

/* jointPdn() and the 'm' / ARM_PDN command were REMOVED on 2026-09-13.
 *
 * 'm' was a live collision: the host's ArduinoComms::arm_set_servo() sends
 * "m <extension>" whenever the r_c_servo command interface changes, and an arm
 * dispatched that to ARM_PDN. After PDN moved from a pin to a current level,
 * the effect became "set hold current = run current on all three joints" --
 * standstill reduction silently off, motors at full current indefinitely, while
 * the 'n' report still showed the configured value. Reported and actual state
 * diverged with nothing to indicate it.
 *
 * It was not relocated because every letter a..z is taken except 'v', and 'v' is
 * the identical trap: arm_set_vacuum_on() sends it, for a valve that is removed
 * hardware. Moving the collision is not fixing it.
 *
 * Deleting costs nothing, because 'n' already does the job:
 *     m 1  (hold at run current)   ->  n <joint> <run> <run>
 *     m 0  (restore configured)    ->  n <joint> <run> <hold>
 * The only thing lost is that 'm' was temporary -- it did not overwrite the
 * stored configuration -- and nothing used that.
 *
 * The host's "m" now falls through to runCommand()'s default: and is answered
 * with ERR-CMD, which is correct: arm servos are a future enhancement that is
 * not built. A visible error beats silently reconfiguring three motors.
 */

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
        /* The curve above reaches 0 at the top of the ramp regardless of
           MIN_TICK_DELAY, so clamp or the speed floor does nothing. */
        if (delay < MIN_TICK_DELAY)
            delay = MIN_TICK_DELAY;
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

        /* Same clamp as the acceleration branch: the curve starts at 0 at the
           top of the decel zone, which is cruise speed and must respect the
           floor. */
        if (delay < MIN_TICK_DELAY)
            delay = MIN_TICK_DELAY;

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

        /* Refuse to move on a position reading that cannot be real.
         *
         * Everything below derives the move from currentAvNow: deltaAv, the tick
         * count, and the direction. A bad reading is therefore not a bad report,
         * it is a bad MOVE -- computed confidently and driven to completion.
         *
         * 2026-09-14: joint 2's pot intermittently railed to ~1013 against a
         * calibrated span of 217..802. The firmware read it as 351 deg while the
         * joint was physically at about 90, computed a large move from the
         * difference, and drove it. It looked exactly like the motor running away
         * in the wrong direction; it was the feedback failing.
         *
         * A reading outside the calibrated span plus AV_SANITY_MARGIN is not a
         * joint that has travelled somewhere unexpected -- the span IS the
         * mechanical range -- so it means a dead wiper, a floating input or a
         * loose connection. Report and stop. A joint that does not move is
         * recoverable; one driven from garbage may not be.
         */
        if (currentAvNow < motors[motor].avAtMinDegree - AV_SANITY_MARGIN ||
            currentAvNow > motors[motor].avAtMaxDegree + AV_SANITY_MARGIN)
        {
            Serial.print(F("*ERR joint "));
            Serial.print(motor);
            Serial.print(F(" position reading "));
            Serial.print(currentAvNow);
            Serial.print(F(" outside "));
            Serial.print(motors[motor].avAtMinDegree - AV_SANITY_MARGIN);
            Serial.print(F(".."));
            Serial.print(motors[motor].avAtMaxDegree + AV_SANITY_MARGIN);
            Serial.println(F(" - check the pot, not moving"));
            return 0;
        }

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

  /* "n" reports the current setting, "n <run_mA> <hold_mA>" changes it on all
     three joints. Both values are RMS per phase.

     Applied immediately, including mid-move -- IRUN takes effect on the next
     chopper cycle. Lowering it while a joint is loaded can drop the load, so
     treat it as a bench command, not something to send during a pick. */
  case MOTOR_CURRENT:
    if (argv1[0] != '\0' || argv2[0] != '\0' || argv3[0] != '\0')
    {
      if (argv1[0] == '\0' || argv2[0] == '\0' || argv3[0] == '\0')
      {
        Serial.println(F("ERR usage: n <joint> <run_mA> <hold_mA>"));
        break;
      }
      if (arg1 < 0 || arg1 >= MOTORS_DEFINED)
      {
        Serial.print(F("ERR joint must be 0.."));
        Serial.println(MOTORS_DEFINED - 1);
        break;
      }
      if (arg2 <= 0 || arg2 > TMC_CURRENT_MAX_MA ||
          arg3 < 0  || arg3 > TMC_CURRENT_MAX_MA)
      {
        Serial.print(F("ERR current out of range 1.."));
        Serial.println(TMC_CURRENT_MAX_MA);
        break;
      }
      if (arg3 > arg2)
      {
        /* IHOLD is a fraction of the IRUN scale on this chip, so a hold above
           run cannot be expressed -- it would silently clamp. */
        Serial.println(F("ERR hold must not exceed run"));
        break;
      }

      tmcRunMa[arg1]  = (uint16_t)arg2;
      tmcHoldMa[arg1] = (uint16_t)arg3;

      if (tmcReady[arg1])
        tmcApplyCurrent(arg1);
    }

    tmcReportCurrent();
    Serial.println(F("OK"));
    break;

  /* "t" reports, "t <joint> <mode>" sets. Modes: 0 normal (hold current),
     1 freewheel, 2 brake low-side, 3 brake high-side.

     TEST COMMAND. Anything other than 0 leaves the joint unable to hold its own
     weight -- modes 2 and 3 damp the fall, they do not stop it. Not persisted
     and not reset by 'r'; a power cycle returns every joint to normal. */
  case STANDSTILL_MODE:
    if (argv1[0] != '\0' || argv2[0] != '\0')
    {
      if (argv1[0] == '\0' || argv2[0] == '\0')
      {
        Serial.println(F("ERR usage: t <joint> <0=normal 1=freewheel 2=brake-ls 3=brake-hs>"));
        break;
      }
      if (arg1 < 0 || arg1 >= MOTORS_DEFINED)
      {
        Serial.print(F("ERR joint must be 0.."));
        Serial.println(MOTORS_DEFINED - 1);
        break;
      }
      if (arg2 < TMC_SS_NORMAL || arg2 > TMC_SS_BRAKE_HS)
      {
        Serial.println(F("ERR mode must be 0..3"));
        break;
      }
      if (!tmcReady[arg1])
      {
        Serial.println(F("ERR needs UART, driver not answering"));
        break;
      }

      tmcStandstill[arg1] = (uint8_t)arg2;
      tmcApplyStandstill(arg1);
    }

    tmcReportStandstill();
    Serial.println(F("OK"));
    break;

  /* "i" reports every joint, "i <n>" just one. Ends with a summary line so a
     caller can act on one token instead of parsing the per-joint detail. */
  case DRIVER_STATUS:
    {
      int faults = 0;

      if (arg1 >= 0 && arg1 < MOTORS_DEFINED && argv1[0] != '\0')
      {
        faults = tmcReportStatus(arg1);
      }
      else
      {
        for (i = 0; i < MOTORS_DEFINED; i++)
          faults += tmcReportStatus(i);
      }

      Serial.println(faults ? F("FAULT") : F("OK"));
    }
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

        /* Disable the AV arrival check for this move, exactly as ROT_CW/ROT_CCW
           do. This is a RELATIVE move by tick count with no position target, but
           the ISR's stop test is unconditional:

               t = abs(currentAv - targetAv);
               if (t < END_AV_THRESHOLD) stop;

           so without this the joint inherits whatever targetAv the last absolute
           move left behind, and halts the moment it happens to pass within 5 AV
           counts of it. Measured 2026-09-14: two identical "e 0 10" commands
           gave -27 then +12 AV counts, the second stopping almost immediately on
           a stale target. That made 'e' useless as the open-loop test command,
           which is the one thing it exists for. */
        motors[arg1].targetAv = 0xffff;

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

  /* "k 1" energise and hold, "k 0" release. This is now the ONLY way to
     de-energise a joint, since the idle EN timeout is off (idleEnableOnTimeMs = 0).

     It must NOT consult leaveEnabled. That flag means "may the idle timeout
     disable this motor", and testing it here conflated two different questions:
     when leaveEnabled went to 0 on 2026-09-13 this condition became permanently
     false, so "k 1" took the else branch and DISABLED the motors -- lock doing
     the opposite of lock, on every on_activate(), which calls arm_lock(1). */
  case ARM_LOCK:
      for (i=0; i < MOTORS_DEFINED;i++)
      {
        if (arg1 == 1)
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

  /* No ARM_PDN case: 'm' is deliberately unhandled and answered by default:
     with ERR-CMD. See the note where jointPdn() used to be. */
  
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
      /* DIAG is a push-pull output on the driver, so a plain input -- no pullup,
         which would fight it. Nothing reads this yet; it is configured here so
         the pin is in a known state rather than left to reset defaults. */
      pinMode(motors[i].diagGpio,INPUT);

      digitalWrite(motors[i].enGpio,HIGH);   
      digitalWrite(motors[i].dirGpio,LOW);   
  
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

    /* After the pins, before anything moves. Talks to each driver over UART and
       restores the microstepping and current that setup() would otherwise have
       replaced with its own defaults -- see the notes by STEPPER_MICROSTEPS.
       Currents must be in RAM before this runs. */
    tmcLoadDefaultCurrents();
    tmcLoadDefaultStandstill();
    tmcConfigure();

    /* NOTHING IS CONNECTED TO THE SERVOS as of 2026-09-13, so this write is
       currently harmless and has never been observed doing anything.

       Before attaching anything to them, know that this runs on EVERY reset --
       power-on, the 'r' command, and every ros2_control activation, because
       RobotSystem::on_activate() now pulses DTR on each arm to guarantee a known
       state. So whatever is on these pins gets commanded to SERVO_FLAT,
       immediately, from whatever position it was in, with no host involvement
       and no way for the host to veto it.

       If SERVO_FLAT is not a safe position to enter from an arbitrary starting
       state, fix it HERE -- read the current position first, or move in steps,
       or do not write at all until commanded. Removing the host's DTR reset is
       the wrong lever: the per-joint stepper currents sent from the URDF depend
       on the board starting from its known defaults. */
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
      /* idleEnableOnTimeMs == 0 means NEVER time out, and is the default on all
         six motors. Without this test a zero interval makes the deadline
         millis()+0, which is already in the past, so the joint would be
         disabled on the very next pass of loop(). */
      if (motors[m].leaveEnabled && motors[m].idleEnableOnTimeMs)
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
