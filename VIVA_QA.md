# Viva Q&A — Line Follower (IA621001 Automation and Robotics)

---

## HOW THE ROBOT FOLLOWS THE LINE

**Q: How does your robot follow the line?**

The robot uses 5 infrared line sensors on the bottom. Each sensor reads how much light reflects back — black absorbs light so gives a low reading, white reflects so gives a high reading. The library combines all 5 readings into a single position value between 0 and 4000, where 0 means the line is far left and 4000 means far right. The centre (where we want the line to be) is 2000. I calculate the error as `position - 2000`. If the error is positive the line is to the right, if negative it's to the left. I then use a PD controller to steer the motors to correct that error every loop.

---

**Q: What is a PD controller and why did you use it?**

PD stands for Proportional-Derivative. The proportional part (P) steers harder the further the robot is from the line — big error means big correction. The derivative part (D) looks at how fast the error is changing — if the error is growing quickly it adds extra correction to prevent overshoot. Together they give smooth, responsive steering. I didn't use the I (integral) term because on a line follower it tends to cause drift and isn't needed for this type of track.

In code:
```
pTerm = KP * error        // how far off centre
dTerm = KD * errorDelta   // how fast it's moving away
steering = pTerm + dTerm
```

---

**Q: How do the motors actually turn the robot?**

Steering is added to the left motor and subtracted from the right. So if the line is to the right (positive error), the left motor speeds up and the right slows down, turning the robot right. If the line is to the left, the opposite happens.

```
leftSpeed  = baseSpeed + steering
rightSpeed = baseSpeed - steering
```

---

## SENSOR FILTERING

**Q: Why do you filter the sensor readings?**

Raw sensor readings are noisy — they jump around slightly even when the robot is perfectly on the line. If I used raw values directly, the PD controller would react to that noise and cause jitter. I use an Exponential Moving Average (EMA) filter which blends the new reading with the previous filtered value:

```
filtered = 0.85 * newReading + 0.15 * previousFiltered
```

The 0.85 (alpha) means 85% of the new reading and 15% of history — so it's responsive but smoothed.

---

**Q: What happens on sharp corners — doesn't the filter slow the robot down?**

Yes, that's a problem on hairpin bends. So on sharp corners I bypass the filter completely and use the raw reading directly. I detect a sharp corner when the outermost sensor (sensor 0 or sensor 4) reads above a threshold — that means the line is right at the edge of the array, which only happens on tight bends. Bypassing the filter gives instant response exactly when it's needed most.

---

## SPEED CONTROL

**Q: Does the robot always run at the same speed?**

No — it uses adaptive speed. On straights the error is small so it runs at full speed (220). As the error grows (on curves) the speed reduces proportionally. This prevents the robot from flying off hairpins and zigzags. There's a minimum speed of 80 so it never stops completely on a curve.

```
if error is small  → speed = 220 (full cruise)
if error is large  → speed reduces down to minimum 80
```

---

## SMOOTH DEAD ZONE / HYSTERESIS

**Q: What is the smooth dead zone and why did you implement it?**

Early versions used a hard threshold — if error was below 200 apply zero correction, if above 200 apply full correction. The problem is the robot oscillates across that boundary constantly causing jitter. Error goes 199 → correction=0, then 201 → correction=JUMP, back and forth endlessly.

My solution is a smooth ramp between two thresholds (DEAD_INNER=150 and DEAD_OUTER=400). Below 150 there's zero correction. Above 400 there's full correction. Between them the correction ramps up linearly from 0% to 100%. There's no sharp boundary to oscillate across so jitter is eliminated. This is the hysteresis technique.

---

## LINE LOSS AND RECOVERY

**Q: What happens when the robot loses the line?**

Every loop I sum all 5 sensor readings. If the total is below a threshold (80) it means no sensor is seeing the line — the robot has derailed. I then run a 3-stage recovery:

**Stage 1 — Fast spin in last known direction:**
Before losing the line I track which way the robot was turning. If it was turning right, the line probably went right, so I spin clockwise at speed 180 for up to 3 seconds (about 4 full rotations), checking sensors every 10ms. If the line is found, stop and resume.

**Stage 2 — Fast spin opposite direction:**
If stage 1 fails, spin the other way for another 3 seconds.

**Stage 3 — Backup and spiral:**
If both spins fail, reverse briefly then do an expanding spiral arc. One wheel stays fast, the other starts slow and ramps up — this sweeps a wider area than a pure spin.

If all three stages fail, the robot stops and asks the user to press B to recalibrate.

---

**Q: Why spin rather than just drive forward to find the line?**

Driving forward when the line is lost would take the robot further away from where it derailed. Spinning in place keeps the robot near the last known position and sweeps 360° to find the line wherever it went. The direction bias (spinning toward where the robot was last turning) makes it faster in most cases.

---

## CALIBRATION

**Q: Why does the robot spin before it starts?**

The sensors need calibration because lighting conditions vary. During calibration the robot spins in 3 passes (right, left, right) so every sensor sees both the black line and the white surface. The library records the minimum and maximum reading for each sensor and uses those to normalise all future readings to a consistent 0–1000 range per sensor. Without calibration the position readings would be unreliable.

---

## CODE STRUCTURE

**Q: How is your code structured?**

The code is in one file (LineFollower.ino) split into 5 clear sections:
1. Constants — all tunable values in one place
2. Hardware objects — motors, sensors, display, buzzer
3. Sensor functions — calibration, reading, filtering, edge detection
4. Motor functions — PD steering, speed calculation, corner boost
5. Recovery functions — fast spin, spiral search
6. setup() and loop() — kept thin, just calls the functions above

The loop() itself has no complex logic — it just calls functions. All the real work is encapsulated in named methods which makes it easy to explain and understand.

---

**Q: What would you change to make it better?**

- Add the I term (integral) to correct for consistent drift on one side
- Use the IMU (gyroscope) to measure actual turn angle during recovery for more precise spinning
- Tune KP and KD more precisely for the specific track — currently they're conservative to avoid oscillation
- Add speed profiling — accelerate on straights and brake earlier before corners

---

## QUICK REFERENCE — KEY VALUES

| Constant | Value | What it does |
|---|---|---|
| LINE_CENTER | 2000 | Set point — target position |
| KP | 0.15 | Proportional gain |
| KD | 1.5 | Derivative gain |
| DEAD_INNER | 150 | Below this: no correction |
| DEAD_OUTER | 400 | Above this: full correction |
| SPEED_STRAIGHT | 220 | Cruise speed |
| SPEED_CURVE | 80 | Minimum speed on curves |
| EMA_ALPHA | 0.85 | Filter responsiveness |
| SPIN_MAX_MS | 3000 | Max recovery spin time |
