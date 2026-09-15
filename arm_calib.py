#!/usr/bin/env python3
"""Bench calibration and accuracy measurement for the ros2-arm12 firmware.

Measures the three constants that decide whether a commanded angle is the angle
you get, and reports what they should be:

    avPerDegree      ADC counts per degree of joint travel
    avAtMinDegree    ADC count at the low end of travel
    ticksPerDegree   step pulses per degree

Usage (board on the serial port, stack NOT running):

    ./arm_calib.py status
    ./arm_calib.py scale 0            measure ticksPerDegree for joint 0
    ./arm_calib.py span 0             find the travel limits, gives avPerDegree
    ./arm_calib.py accuracy 0         how close 'p' lands, and which way it errs
    ./arm_calib.py accuracy           all three joints

Nothing here writes to the firmware. It prints values for you to paste into the
MOTOR initialisers.

USING THIS ON THE LEFT ARM
--------------------------
The right arm was calibrated 2026-09-14; the left has never been done and
`leftMotors[]` still carries the original numbers. Four things to change first:

  1. `PORT` below, or pass --port. The left arm is a different device node.
  2. `CURRENT` below holds the RIGHT arm's constants. Replace it with the values
     from `leftMotors[]` -- it is only used to print "currently X" next to each
     result, but a stale table makes the comparison meaningless.
  3. Send `a l` to the board first. Both arms run the same image and the arm is
     selected at startup by that one command, which nothing verifies. Calibrate
     an arm that thinks it is the other one and every number will be wrong.
  4. Confirm which joints stop at 90/270 on the LEFT arm before using `span`.
     On the right arm only joint 2 does; joints 0 and 1 travel beyond, so `span`
     refuses them unless --low-deg/--high-deg are given. Do not assume the left
     arm is mechanically identical.

WHICH MODE TO USE
-----------------
`span` is strictly better than `mark`/`fit` when it applies, because it reads the
AV at two *physical* stops rather than two angles someone judged by eye. Use
`mark`/`fit` only for joints whose stops are not at known angles -- and if you do,
every mark must come from a real reference. An eyeballed 180 mark on the right
arm's joints 0 and 1 produced a convincing but entirely false curvature, and the
fits only came good once it was discarded.

A real end stop tapers: the step before it shrinks (22 counts, then 4). A
collision stops abruptly. That difference is the only thing distinguishing a
limit from the arm hitting the tower, which has happened here.

Three things this has to get right, all learned the hard way:

  * Opening the port pulls DTR low, which holds the board in reset and makes it
    look dead. DTR is asserted high after opening and never pulsed low, so a
    running board keeps running and keeps its configuration.

  * Open-loop moves ('c'/'w'/'e'/'y') set targetAv to 0xffff, so the arrival
    check never fires and the move runs to -TICKS_PADDING. A commanded N ticks
    therefore delivers N + 150. Ignoring that overstates ticksPerDegree by 150
    ticks on every measurement.

  * A failing pot rails to near 0 or near 1023, which is outside any joint's
    calibrated span. Readings are checked for plausibility and discarded rather
    than averaged in, because one railed sample ruins a mean silently.
"""

import argparse
import fcntl
import json
import os
import re
import select
import statistics
import struct
import sys
import termios
import time

PORT = "/dev/ttyUSB0"
BAUD = 57600

TICKS_PADDING = 150          # must match the firmware
END_AV_THRESHOLD = 5         # firmware's arrival window, AV counts
JOINT_MIN_DEG = 90           # motorLimits lower bound
JOINT_MAX_DEG = 270          # motorLimits upper bound

# Current constants, right arm, from the MOTOR initialisers. Used only to show
# what changed -- nothing here depends on them being correct.
CURRENT = {
    # All three measured 2026-09-14. Joints 0 and 1 from marks at 90 and 270
    # deg; joint 2 from 'span', which reads its actual end stops -- only joint 2
    # stops at 90/270, so only joint 2 can be done that way.
    0: dict(ticksPerDegree=19.0, avPerDegree=2.750, avAtMin=253, avAtMax=748),
    1: dict(ticksPerDegree=15.9, avPerDegree=2.617, avAtMin=248, avAtMax=719),
    2: dict(ticksPerDegree=18.0, avPerDegree=2.983, avAtMin=222, avAtMax=759),
}
PLAUSIBLE_MARGIN = 120       # matches the firmware's AV_SANITY_MARGIN


class Arm:
    def __init__(self, port=PORT, baud=BAUD):
        os.system(f"stty -F {port} {baud} raw -echo")
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        # Assert DTR without pulsing it low: a low pulse reboots the board and
        # throws away whatever currents and standstill modes are configured.
        fcntl.ioctl(self.fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
        time.sleep(1.2)
        self._drain(0.8)

    def close(self):
        os.close(self.fd)

    def _drain(self, seconds):
        out = b""
        end = time.time() + seconds
        while time.time() < end:
            ready, _, _ = select.select([self.fd], [], [], 0.2)
            if ready:
                try:
                    out += os.read(self.fd, 4096)
                except BlockingIOError:
                    pass
        return out.decode("ascii", "replace")

    def cmd(self, text, settle=1.2, read=0.8):
        os.write(self.fd, (text + "\r").encode())
        time.sleep(settle)
        return self._drain(read).strip()

    # ---- readings ------------------------------------------------------

    def av(self, filtered=False):
        """Raw per-joint ADC values, or None for any that is not plausible."""
        reply = self.cmd("5 0" if filtered else "4 0")
        match = re.search(r"\((\d+),\s*(\d+),\s*(\d+)\)", reply)
        if not match:
            return [None, None, None]

        values = []
        for joint, raw in enumerate(int(x) for x in match.groups()):
            low = CURRENT[joint]["avAtMin"] - PLAUSIBLE_MARGIN
            high = CURRENT[joint]["avAtMax"] + PLAUSIBLE_MARGIN
            values.append(raw if low <= raw <= high else None)
        return values

    def av_of(self, joint, samples=3):
        """One joint's AV, median of several samples, None if any railed."""
        got = []
        for _ in range(samples):
            value = self.av()[joint]
            if value is not None:
                got.append(value)
        if not got:
            return None
        return int(statistics.median(got))

    def degrees(self):
        for line in self.cmd("s").splitlines():
            line = line.strip()
            if line.startswith("*"):
                continue
            fields = line.split()
            if len(fields) >= 3 and all(f.lstrip("-").isdigit() for f in fields[:3]):
                return [int(f) for f in fields[:3]]
        return None

    def hold(self):
        self.cmd("k 1", settle=1.5)


def require_av(arm, joint, what):
    value = arm.av_of(joint)
    if value is None:
        print(f"  ! joint {joint} position reading is not plausible "
              f"(railed pot or loose wiring) - cannot {what}")
    return value


# ------------------------------------------------------------------ status

def do_status(arm, _args):
    print("positions (deg) :", arm.degrees())
    print("AV unfiltered   :", arm.av())
    print("AV filtered     :", arm.av(filtered=True))
    print()
    print(arm.cmd("n", settle=1.5))
    print()
    print(arm.cmd("i", settle=2.0))


# ------------------------------------------------------------------- scale

def do_scale(arm, args):
    """ticksPerDegree, from open-loop moves of several sizes.

    Runs each size in both directions and averages, so gravity helping one way
    and hindering the other cancels instead of biasing the answer.
    """
    joint = args.joint
    av_per_degree = args.av_per_degree or CURRENT[joint]["avPerDegree"]
    arm.hold()

    # Measure from mid-travel and alternate direction, so the joint oscillates
    # about the centre instead of walking into an end stop. Hitting a limit
    # silently truncates a move and the ratio comes out meaningless -- the first
    # run of this tool reported ticks/AV from 8.3 to 19.4 for exactly that
    # reason, an 800-tick move having run out of travel after 55 AV counts.
    here = arm.degrees()
    if here is None:
        print("  ! no position reply")
        return
    centre = (JOINT_MIN_DEG + JOINT_MAX_DEG) // 2
    if abs(here[joint] - centre) > 15:
        print(f"  centring joint {joint}: {here[joint]} -> {centre} deg")
        wanted = [centre if j == joint else here[j] for j in range(3)]
        arm.cmd(f"p {wanted[0]} {wanted[1]} {wanted[2]}", settle=16.0)

    # Largest move that stays clear of both limits, as ticks.
    head_room = (JOINT_MAX_DEG - JOINT_MIN_DEG) / 2 - 15
    max_ticks = int(head_room * CURRENT[joint]["ticksPerDegree"]) - TICKS_PADDING
    sizes = [t for t in args.sizes if t <= max_ticks]
    if len(sizes) < len(args.sizes):
        dropped = [t for t in args.sizes if t > max_ticks]
        print(f"  dropping {dropped}: more travel than the joint has "
              f"(max {max_ticks} ticks from centre)")
    if not sizes:
        print("  ! no usable move sizes")
        return

    print(f"joint {joint}: measuring ticksPerDegree")
    print(f"  assuming avPerDegree = {av_per_degree} (use --av-per-degree to override,")
    print("  or run 'span' first -- ticksPerDegree is only as good as this)")
    print(f"  each commanded N ticks actually delivers N + {TICKS_PADDING}\n")

    print(f"  {'cmd ticks':>9} {'effective':>9} {'dir':>4} {'dAV':>6} {'ticks/AV':>9}")
    ratios = []
    for ticks in sizes:
        for direction in ("c", "w"):
            before = require_av(arm, joint, "measure")
            if before is None:
                return
            arm.cmd(f"{direction} {joint} {ticks}", settle=max(4.0, ticks / 120))
            after = require_av(arm, joint, "measure")
            if after is None:
                return

            delta = abs(after - before)
            effective = ticks + TICKS_PADDING
            if delta < 3:
                print(f"  {ticks:>9} {effective:>9} {direction:>4} {delta:>6} "
                      f"{'(too small, skipped)':>9}")
                continue
            ratio = effective / delta
            ratios.append(ratio)
            print(f"  {ticks:>9} {effective:>9} {direction:>4} {delta:>6} {ratio:>9.2f}")

    if len(ratios) < 2:
        print("\n  not enough usable measurements")
        return

    ticks_per_av = statistics.median(ratios)
    measured = ticks_per_av * av_per_degree
    now = CURRENT[joint]["ticksPerDegree"]
    print(f"\n  ticks per AV count : {ticks_per_av:.3f}  "
          f"(spread {min(ratios):.2f}..{max(ratios):.2f})")
    print(f"  ticksPerDegree     : {measured:.2f}   currently {now}   "
          f"{'no change needed' if abs(measured - now) / now < 0.05 else f'change to {measured:.1f}'}")


# -------------------------------------------------------------------- span

def do_span(arm, args):
    """Find the AV at both ends of travel, which gives avPerDegree directly.

    Creeps in small steps and stops as soon as the joint stops responding, so it
    finds the limit without driving into it repeatedly.
    """
    joint = args.joint
    arm.hold()
    print(f"joint {joint}: finding travel limits in {args.step} deg steps")
    print(f"  stops when a step moves less than {args.stall_av} AV counts")
    print(f"  treating the stops as {args.low_deg} and {args.high_deg} deg")
    if joint != 2 and (args.low_deg, args.high_deg) == (JOINT_MIN_DEG, JOINT_MAX_DEG):
        print()
        print("  ! Joints 0 and 1 travel BEYOND 90 and 270 mechanically, so their")
        print("    end stops are NOT those angles and this will be wrong. Only")
        print("    joint 2 stops at 90/270. Measure the stop angles externally and")
        print("    pass --low-deg/--high-deg, or use 'mark' and 'fit' instead.")
        return
    print()

    ends = {}
    for name, direction in (("low", "c"), ("high", "w")):
        last = require_av(arm, joint, "find limits")
        if last is None:
            return
        travelled = 0
        while travelled < args.max_travel:
            arm.cmd(f"{direction} {joint} {int(args.step * CURRENT[joint]['ticksPerDegree'])}",
                    settle=3.0)
            now = arm.av_of(joint)
            if now is None:
                print(f"  ! reading went implausible at the {name} end - stopping")
                return
            moved = abs(now - last)
            print(f"  {name:>4} end: AV {last:>4} -> {now:>4}  ({moved:+3d})")
            if moved < args.stall_av:
                ends[name] = now
                print(f"  {name} limit found at AV {now}\n")
                break
            last = now
            travelled += args.step
        else:
            print(f"  ! travelled {args.max_travel} deg without finding the {name} end\n")
            return

    lo, hi = min(ends.values()), max(ends.values())
    degrees = args.high_deg - args.low_deg
    av_per_degree = (hi - lo) / degrees
    now = CURRENT[joint]
    print(f"  AV span        : {lo} .. {hi}   ({hi - lo} counts over {degrees} deg)")
    print(f"  avPerDegree    : {av_per_degree:.3f}   currently {now['avPerDegree']}")
    at_min = lo + (JOINT_MIN_DEG - args.low_deg) * av_per_degree
    at_max = lo + (JOINT_MAX_DEG - args.low_deg) * av_per_degree
    print(f"  avAtMinDegree  : {at_min:.0f}  (AV at {JOINT_MIN_DEG} deg)"
          f"   currently {now['avAtMin']}")
    print(f"  avAtMaxDegree  : {at_max:.0f}  (AV at {JOINT_MAX_DEG} deg)"
          f"   currently {now['avAtMax']}")


# ---------------------------------------------------------------- accuracy

def do_accuracy(arm, args):
    """How close 'p' lands, and whether the error is a bias or scatter.

    A consistent signed error is a scale or deceleration problem and is fixable.
    Scatter around zero is backlash or noise and is not.
    """
    joints = [args.joint] if args.joint is not None else [0, 1, 2]
    arm.hold()
    print(f"commanding 'p' to each target, {args.repeats} time(s)\n")
    print(f"  {'joint':>5} {'from':>5} {'target':>7} {'landed':>7} {'error':>6}")

    errors = {j: [] for j in joints}
    for target in args.targets:
        for _ in range(args.repeats):
            before = arm.degrees()
            if before is None:
                print("  ! no position reply")
                return
            # Hold the joints we are not measuring at their current angle.
            wanted = [target if j in joints else before[j] for j in range(3)]
            arm.cmd(f"p {wanted[0]} {wanted[1]} {wanted[2]}", settle=args.settle)
            after = arm.degrees()
            if after is None:
                print("  ! no position reply")
                return
            for j in joints:
                err = after[j] - target
                errors[j].append(err)
                print(f"  {j:>5} {before[j]:>5} {target:>7} {after[j]:>7} {err:>+6}")

    print()
    deadband = END_AV_THRESHOLD / CURRENT[0]["avPerDegree"]
    print(f"  firmware stops within {END_AV_THRESHOLD} AV counts "
          f"= {deadband:.1f} deg, so errors below that are expected\n")
    for j in joints:
        vals = errors[j]
        if not vals:
            continue
        bias = statistics.mean(vals)
        scatter = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        verdict = ("consistent overshoot" if bias > deadband else
                   "consistent undershoot" if bias < -deadband else
                   "within the deadband")
        print(f"  joint {j}: bias {bias:+.1f} deg, scatter {scatter:.1f} deg  -> {verdict}")
        if abs(bias) > deadband and scatter < abs(bias):
            print("           a bias larger than the scatter is systematic: suspect "
                  "avPerDegree or the decel ramp, not backlash")


# ----------------------------------------------------------- free / hold

def do_free(arm, _args):
    """Release all joints so they can be positioned by hand.

    For calibration you want the joint at a KNOWN angle, which means putting it
    there yourself against whatever reference you are using. Commanding it with
    'p' would place it using the very constants you are trying to measure.

    Joints are limp afterwards and will fall under load -- support anything that
    can drop before running this.
    """
    print(arm.cmd("k 0", settle=1.5))
    print("all joints released - they are limp and can fall")


def do_hold(arm, _args):
    """Energise and hold. Use after positioning by hand, before 'mark'."""
    print(arm.cmd("k 1", settle=1.5))
    print("all joints holding")
    print("positions:", arm.degrees())


# ------------------------------------------------------------ mark / fit

MARKS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "arm_calib_marks.json")


def _load_marks():
    try:
        with open(MARKS_FILE) as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def _save_marks(marks):
    with open(MARKS_FILE, "w") as handle:
        json.dump(marks, handle, indent=2, sort_keys=True)


def do_mark(arm, args):
    """Record the current AV against an angle you measured externally.

    This is the only way to calibrate joints 0 and 1. Their mechanical stops lie
    OUTSIDE 90 and 270, so the stops cannot be used as known angles the way
    joint 2's can -- there is no reference in the machine, and it has to come
    from a protractor, an inclinometer, or a fixture.

    Position the joint, measure it, then record it:

        ./arm_calib.py mark 0 120
        ./arm_calib.py mark 0 180
        ./arm_calib.py mark 0 240
        ./arm_calib.py fit 0

    Two marks are enough; more, spread across the range, average out backlash
    and any non-linearity in the pot.
    """
    joint = args.joint
    value = require_av(arm, joint, "record a mark")
    if value is None:
        return

    marks = _load_marks()
    entries = marks.setdefault(str(joint), [])
    entries.append({"degrees": args.degrees, "av": value})
    _save_marks(marks)

    print(f"joint {joint}: recorded {args.degrees} deg = AV {value}   "
          f"({len(entries)} mark(s) now)")
    if len(entries) < 2:
        print("  need at least two at different angles before 'fit'")


def do_fit(arm, args):
    """Least-squares fit over the recorded marks -> avPerDegree, avAtMinDegree."""
    joint = args.joint
    entries = _load_marks().get(str(joint), [])
    if len(entries) < 2:
        print(f"joint {joint}: need at least two marks, have {len(entries)}")
        return

    degrees = [e["degrees"] for e in entries]
    avs = [e["av"] for e in entries]
    if len(set(degrees)) < 2:
        print("  all marks are at the same angle - nothing to fit")
        return

    slope = statistics.mean(
        [(avs[i] - avs[j]) / (degrees[i] - degrees[j])
         for i in range(len(entries)) for j in range(i + 1, len(entries))
         if degrees[i] != degrees[j]])
    intercept = statistics.mean([a - slope * d for a, d in zip(avs, degrees)])

    print(f"joint {joint}: {len(entries)} marks")
    print(f"  {'measured deg':>12} {'AV':>6} {'fitted AV':>10} {'residual':>9}")
    worst = 0.0
    for d, a in sorted(zip(degrees, avs)):
        fitted = slope * d + intercept
        worst = max(worst, abs(a - fitted))
        print(f"  {d:>12.1f} {a:>6} {fitted:>10.1f} {a - fitted:>+9.1f}")

    now = CURRENT[joint]
    at_min = slope * JOINT_MIN_DEG + intercept
    at_max = slope * JOINT_MAX_DEG + intercept
    print()
    print(f"  avPerDegree    : {slope:.3f}     currently {now['avPerDegree']}")
    print(f"  avAtMinDegree  : {at_min:.0f}        currently {now['avAtMin']}")
    print(f"  avAtMaxDegree  : {at_max:.0f}        currently {now['avAtMax']}")
    print(f"  worst residual : {worst:.1f} AV = {worst / slope:.1f} deg")
    if worst / slope > 2.0:
        print("  ! residuals this large mean the marks disagree: check the external")
        print("    measurements, or the pot is non-linear over that range")


def do_marks(_arm, args):
    marks = _load_marks()
    if args.clear is not None:
        marks.pop(str(args.clear), None)
        _save_marks(marks)
        print(f"cleared marks for joint {args.clear}")
        return
    if not marks:
        print("no marks recorded")
        return
    for joint in sorted(marks):
        print(f"joint {joint}:")
        for e in marks[joint]:
            print(f"   {e['degrees']:>7.1f} deg = AV {e['av']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=PORT)
    sub = parser.add_subparsers(dest="mode", required=True)

    sub.add_parser("status", help="positions, AV, currents, driver faults")

    p = sub.add_parser("scale", help="measure ticksPerDegree")
    p.add_argument("joint", type=int, choices=(0, 1, 2))
    p.add_argument("--sizes", type=int, nargs="+", default=[200, 400, 800])
    p.add_argument("--av-per-degree", type=float, default=None)

    p = sub.add_parser("span", help="find travel limits, gives avPerDegree")
    p.add_argument("joint", type=int, choices=(0, 1, 2))
    p.add_argument("--step", type=int, default=8, help="degrees per creep step")
    p.add_argument("--stall-av", type=int, default=6, help="AV change that counts as stopped")
    p.add_argument("--max-travel", type=int, default=200, help="give up after this many deg")
    p.add_argument("--low-deg", type=float, default=JOINT_MIN_DEG,
                   help="true angle of the low stop (only 90 on joint 2)")
    p.add_argument("--high-deg", type=float, default=JOINT_MAX_DEG,
                   help="true angle of the high stop (only 270 on joint 2)")

    sub.add_parser("free", help="release joints for positioning by hand")
    sub.add_parser("hold", help="energise and hold")

    p = sub.add_parser("mark", help="record current AV as a known angle")
    p.add_argument("joint", type=int, choices=(0, 1, 2))
    p.add_argument("degrees", type=float, help="angle you measured externally")

    p = sub.add_parser("fit", help="fit recorded marks -> avPerDegree")
    p.add_argument("joint", type=int, choices=(0, 1, 2))

    p = sub.add_parser("marks", help="list or clear recorded marks")
    p.add_argument("--clear", type=int, choices=(0, 1, 2), default=None)

    p = sub.add_parser("accuracy", help="how close 'p' lands")
    p.add_argument("joint", type=int, nargs="?", choices=(0, 1, 2), default=None)
    p.add_argument("--targets", type=int, nargs="+", default=[120, 180, 240, 180])
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--settle", type=float, default=14.0)

    args = parser.parse_args()

    arm = Arm(args.port)
    try:
        {"status": do_status, "scale": do_scale, "span": do_span,
         "accuracy": do_accuracy, "mark": do_mark, "fit": do_fit,
         "marks": do_marks, "free": do_free,
         "hold": do_hold}[args.mode](arm, args)
    finally:
        arm.close()


if __name__ == "__main__":
    sys.exit(main())
