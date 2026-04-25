# GPS Navigation — Plain English Explanation

## The Problem

Your robot needs to drive from Point A to Point B using GPS. But GPS is noisy (wiggles around), the compass is slow to respond, and turning too hard overshoots the target. So we use four tricks to make it work better.

---

## Trick 1: Pure-Pursuit (Straight-Line Steering)

### What It Does
Instead of always aiming for the destination, the robot aims for a point **ahead on the straight line** between where it started and where it's going.

### Why It Matters
- **Without it**: GPS jitters side-to-side. The robot chases the noise and zigzags.
- **With it**: The robot stays on the line, ignoring temporary GPS wobbles.

### The Sensors Used
- **GPS receiver**: Tells you "you are here"
- **GPS receiver**: Tells you "destination is there"
- Your current position + the straight-line path = where to look ahead

### How It Works (Simple Version)
1. Draw a line from Start → Destination
2. Figure out where you are *relative to* that line
3. Look 2.5 meters ahead along that line
4. Steer toward that look-ahead point (not the destination)
5. If GPS jitters left/right, you stay on the line anyway

### Real-World Example
Imagine driving down a highway. You don't aim for the exit 10 miles away. You watch the line on the road 20 feet ahead and steer toward that. Same idea.

---

## Trick 2: Gyro-Aided Heading (Fast Compass)

### What It Does
Makes the compass respond instantly instead of slowly. Uses a spinning gyroscope to predict what the compass *will* see, then corrects it with the actual compass reading.

### Why It Matters
- **Without it**: Compass reading is slow and laggy (~1 second behind). Robot turns, compass catches up late.
- **With it**: Compass responds instantly. Robot knows its heading right now.

### The Sensors Used
- **Magnetometer (compass)**: Points north, but slow to update (averages 20 old readings)
- **Gyroscope**: Spins with the robot, gives instant rotation speed (rad/s)

### How It Works (Simple Version)
1. **Every 0.1 seconds:**
   - Gyro says "we spun 2° to the left in the last 0.1s"
   - Use that to predict the new compass heading
   - Read the actual compass
   - If actual compass is different from prediction, blend them 98% prediction + 2% compass

2. **Why this works:**
   - Gyro is fast but drifts (like a spinning ice skater who gradually slows)
   - Compass is slow but accurate (always points north in the end)
   - Blend them and you get fast + accurate

### Real-World Example
When you turn your head, you *feel* yourself spinning (that's the gyro). But your eyes see where you're looking (that's the compass). Both together tell you your heading perfectly.

---

## Trick 3: PD Heading Controller (Smooth Turns)

### What It Does
Controls how hard the robot turns. Uses two calculations:
- **P (Proportional)**: Turn harder the farther you are from the target heading
- **D (Derivative)**: Turn less hard if you're already spinning (damping)

### Why It Matters
- **Without it**: Robot overshoots the target heading, oscillates (zig-zag turning)
- **With it**: Robot smoothly approaches the target heading without overshoot

### The Sensors Used
- **Magnetometer**: Tells you current heading
- **Gyroscope**: Tells you how fast you're spinning right now

### How It Works (Simple Version)

**P Term (Proportional):**
```
If you need to turn 30° to face the goal:
  Turn command = 30° × (strength factor)
  
If you only need to turn 5° to face the goal:
  Turn command = 5° × (strength factor)
```
Bigger error = bigger turn. Simple.

**D Term (Derivative — the Damper):**
```
If the gyro says you're spinning at +5°/second (fast spin to the left):
  Reduce the turn command a little
  
If the gyro says you're barely spinning:
  No reduction, turn normally
```
This prevents overshoot. It's like a car suspension that dampens bouncing.

**Combined:**
```
Total turn = (P term) + (D term)
           = (error × Kp) + (spin rate × Kd)
```

### Real-World Example
Imagine trying to stop a spinning top. If you only look at how far off-angle it is, you'd overshoot. But if you also feel how fast it's spinning, you can gently slow it down and catch it at exactly the right angle.

---

## Trick 4: GPS Covariance Gating (Reject Bad Fixes)

### What It Does
Ignores GPS readings that are uncertain. GPS tells you not just where you are, but also *how confident* it is (±5 meters, ±10 meters, etc.). We reject uncertain readings.

### Why It Matters
- **Without it**: Bad GPS (multipath, reflections, poor signal) gives you wrong position, messes up navigation
- **With it**: Only use GPS when it's confident, ignore the rest

### The Sensors Used
- **GPS receiver**: Tells you position + confidence (standard deviation)

### How It Works (Simple Version)
1. GPS says: "You're at (-31.980457, 115.817590) with uncertainty of 1 meter"
   - **Accept it** — low uncertainty
   
2. GPS says: "You're at (-31.980400, 115.817600) with uncertainty of 8 meters"
   - **Reject it** — high uncertainty (we set the threshold at 5 meters)

3. Repeat every time GPS sends a new fix

### Real-World Example
You ask 10 people "which way is north?" Some point confidently, some guess. You listen to the confident ones and ignore the guessers.

---

## How They Work Together

```
START DRIVING TO WAYPOINT
         ↓
    [Pure-Pursuit]
    "Look ahead 2.5m on the straight line"
         ↓
    [GPS Covariance Gate]
    "Are we sure where we are? Use only good GPS"
         ↓
    Calculate steering direction
         ↓
    [Gyro-Aided Heading]
    "Fast compass + gyro = instant heading"
         ↓
    [PD Controller]
    "Turn smoothly without overshoot"
         ↓
    Publish turn command to robot
         ↓
    Repeat 10x per second
```

### What Each Sensor Contributes

| Sensor | Purpose | Update Rate |
|--------|---------|-------------|
| GPS | Position + confidence | 5-10 Hz (noisy) |
| Magnetometer | Compass heading (slow) | 100 Hz but averaged over 20 samples |
| Gyroscope | Spin rate (fast + accurate) | 100 Hz (instant) |

### The Result

- **Straighter paths**: Pure-pursuit keeps you on the line
- **Faster response**: Gyro-aided heading reacts instantly
- **Smooth turns**: PD controller avoids overshoot
- **Reliable**: Reject bad GPS, trust good GPS

---

## Tuning (If It Doesn't Work Well)

### If the robot zigzags:
- Increase `LOOKAHEAD_DIST_M` (look farther ahead)
- Decrease `ANGULAR_GAIN` (turn less aggressively)

### If the robot oscillates when turning:
- Increase `ANGULAR_D_GAIN` (more damping)

### If the robot drifts sideways:
- Decrease `LOOKAHEAD_DIST_M` (stay closer to line)

### If the robot jitters due to bad GPS:
- Decrease `GPS_MAX_STD_M` (only use confident GPS)
