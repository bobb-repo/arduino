# Exponential Ramp Implementation

## Overview
Replaced the old linear skip-interval table acceleration system with a smooth exponential ramp for better motor acceleration and deceleration. Also moved analog reads out of ISR and added progress monitoring.

## What Changed

### 1. Removed Old System
- **Removed**: `skipIntervalTable[] = {2,2,2,2,1,1,1,0}` - Fixed step table
- **Removed**: `tickSkipIntervalIndex`, `skipBuckets`, `skipIntervalChangeIncrement`, `ticksIntervalChangeTrigger` - Interval tracking variables

### 2. Added Exponential Ramp
- **Added**: `calculateExponentialDelay(int motor)` function at line ~300
- **Added**: Constants `MIN_TICK_DELAY` (0) and `MAX_TICK_DELAY` (8)
- **Optimized**: Uses integer-only math (no floating point) for ISR performance

### 3. Performance Optimizations
- **Moved**: `analogRead()` out of ISR into main loop (~line 1213)
- **Removed**: All `Serial.println()` debug statements from ISR
- **Result**: ISR now runs much faster, reducing jitter and improving step timing

### 4. Progress Monitoring
- **Added**: Motor stall detection - checks if analog value is changing
- **Added**: `progressCheckCounter` and `lastProgressCheckAv` to MOTOR struct
- **Feature**: Warns if motor not making progress (stalled/blocked)
- **Configurable**: Can auto-stop motor on stall (currently just warns)

## How It Works

### Acceleration Phase (MOTOR_ACC)
```
delay = MAX_TICK_DELAY × (1 - progress)²
```
- Starts at MAX_TICK_DELAY (8 ticks between steps) - slow
- Exponentially decreases to 0 (full speed)
- Most acceleration happens early in the ramp

### Steady State (MOTOR_STEADY)
```
delay = 0
```
- No delay between steps
- Maximum speed maintained

### Deceleration Phase (MOTOR_DEC)
```
delay = MAX_TICK_DELAY × progress²
```
- Starts at 0 (full speed)
- Exponentially increases to MAX_TICK_DELAY (slow)
- Smooth stop with most deceleration at the end

## Benefits

1. **Smoother Motion**: Exponential curve is more natural than linear steps
2. **Reduced Mechanical Stress**: Gradual acceleration/deceleration reduces jerking
3. **Simpler Code**: One function replaces complex interval-change logic
4. **Tunable**: Easy to adjust MAX_TICK_DELAY for different acceleration rates
5. **Adaptive**: Automatically scales to move distance

## Tuning Parameters

### MAX_TICK_DELAY (currently 8)
- **Increase** (e.g., 12) for:
  - Slower, more gentle acceleration
  - Heavy loads or long arms
  - Reduced mechanical stress

- **Decrease** (e.g., 4) for:
  - Faster acceleration
  - Lighter loads
  - Snappier response

### Acceleration Distance Calculation
```cpp
ticksFullSpeedTrigger = min(50, ticksLeft/8);
```
- Currently uses 1/8 of move distance (max 50 ticks)
- Increase denominator (e.g., /6) for longer acceleration
- Decrease denominator (e.g., /10) for shorter acceleration

## Debug Levels

The debug system has been improved with clear, informative messages:

- `z 0` - Debug off
- `z 1` - Basic debug (motor start/stop, transitions, progress warnings)
- `z 2` - Verbose (+ analog value monitoring every loop)
- `z 3` - Very verbose (+ progress check confirmations)
- `z 4` - Maximum (+ deceleration tick details)

### Debug Output Examples

```
*StartMotor 0 ticks:180 accel:22 decel@:136
*AS->Steady 0: 22
*Steady->DEC 0: 136
*STOP M0 [HIGH_AV_TARGET] ticks:42 av:845/850
*WARNING M1 NO PROGRESS av:234 delta:1 ticks:85
```

## Testing Recommendations

1. **Enable Debug Mode**: `z 1` command
   - Shows motor start, transitions, and stop reasons
   - Warns if motor stalls
   - Shows clear, readable status messages

2. **Test Short Moves**: `e 0 10` (rotate motor 0 by 10 degrees)
   - Should smoothly accelerate and decelerate
   - No harsh starts/stops

3. **Test Long Moves**: `e 0 90` (rotate motor 0 by 90 degrees)
   - Should reach full speed in middle
   - Smooth ramp up and down at ends

4. **Monitor Current Draw**:
   - Exponential ramp should show gradual current increase
   - Less current spike than old system

## Future Enhancements

If you need even smoother motion, consider:
1. **S-Curve (Jerk Limiting)**: Add cubic or sine-based ramps
2. **Per-Motor Tuning**: Different MAX_TICK_DELAY for each joint
3. **Load-Based Adjustment**: Adjust based on position feedback
4. **Coordinated Motion**: Synchronize multiple motors to finish together

## Troubleshooting

**Motors too slow to start:**
- Increase MAX_TICK_DELAY

**Motors jerk at start:**
- Decrease MAX_TICK_DELAY or increase ticksFullSpeedTrigger

**Motors don't reach full speed:**
- Decrease ticksFullSpeedTrigger or make longer moves

**Oscillation at end of move:**
- Ensure deceleration distance is adequate
- May need position-based stopping (already implemented via targetAv)

**"NO PROGRESS" warnings:**
- Motor may be stalled, blocked, or at mechanical limit
- Check for mechanical obstructions
- Verify motor is enabled and powered
- To auto-stop on stall, uncomment line ~1238: `motors[m].stopFlag = 5;`

**ISR Performance:**
- All slow operations (analogRead, Serial.println) moved out of ISR
- ISR now only does: tick counting, state checks, step pulse generation
- Main loop handles: analog reading, debug output, progress monitoring
