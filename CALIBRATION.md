# Arm calibration and limits — where the numbers live

The arm firmware here and the ROS stack in `robot-drivers-toplevel` hold the
SAME physical constants in two places. They must agree. The firmware **clamps**;
the host only **predicts**. A mismatch is silent: IK hands back a pose the arm
will not reach, and nothing reports the difference.

| | this repo | robot-drivers-toplevel |
|---|---|---|
| travel limits | `MOTOR_LIMITS` in `ros2-arm12.ino` | `cobo_controller.yaml`, `LIMITS_BY_ARM` in `picking_v2/geometry.py` |
| stepper currents | `tmcRunMa` / `tmcHoldMa` defaults | `cobo.ros2_control_macro.xacro` (authoritative; re-sent on every activate) |
| calibration scale | `avPerDegree`, `avAtMinDegree`, `avAtMaxDegree` | not mirrored — firmware only |

Paired commits, 2026-09-17:

* this repo, branch `add-arm11-tower5`: `4b5b7e5` .. `a67a1d9`
* `Robenetix/robot-drivers-toplevel`, branch `picking-v2-review`: `4c718bc`, `4d8aba7`

## Measured limits

|  | joint 0 | joint 1 | joint 2 |
|---|---|---|---|
| left  | 90 .. 260 | 79 .. 286 | 90 .. 270 |
| right | 90 .. 290 | 68 .. 325 | 90 .. 270 |

Left joint 0's upper limit is **lower** than the old 270: its stop is at 268.7
deg, so 270 was commanding the joint into it. Joint 2 is unmeasured on both arms
and keeps 90/270.

## Two things that cost days

**Identify the arm by the `f0` marker, never by the port name.** udev maps by USB
socket, so two boards in swapped sockets give plausible inverted names, and
`ttyUSB<n>` moves on any replug. The right arm has a VL53L1X time-of-flight
sensor and the left does not, so `sensor.init()` fails on the left and the
*library* prints `f0` at startup — which is why grepping the sketch for it finds
nothing. `arm_calib.py` checks this and refuses to run on a mismatch; `whoami
--expect l|r` does it on demand.

**Nothing catches a jam.** The no-progress detector in the timer loop computes
its result and discards it — the `stopFlag` assignment is commented out. A move
that reaches a stop grinds at full current until its tick budget runs out. That
is why every limit is held 8 degrees off its measured stop.
