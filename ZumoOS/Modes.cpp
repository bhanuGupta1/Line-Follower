// ============================================================
//  Modes.cpp — Line following algorithm
//  IA621001 Automation and Robotics — Project 1
//
//  Implements ZumoOS V6: Smooth Dead Zone + Fast Spin Recovery
//
//  KEY DESIGN DECISIONS:
//
//  1. PD CONTROLLER
//     The sensor array returns a position 0–4000.
//     Set point = 2000 (centre). Error = position - 2000.
//     Proportional term (KP * error) steers toward the line.
//     Derivative term (KD * delta) damps oscillation.
//
//  2. SMOOTH DEAD ZONE (eliminates jitter)
//     Previous versions used a hard threshold:
//       error < 200 → correction = 0
//       error > 200 → correction = JUMP
//     This caused the robot to oscillate across the boundary.
//     V6 replaces this with a linear ramp from DEAD_INNER to
//     DEAD_OUTER, so correction fades in smoothly.
//
//  3. EMA FILTER (smooths noisy sensor readings)
//     Raw position is noisy. An Exponential Moving Average
//     filter blends the new reading with the previous filtered
//     value. On sharp corners the filter is bypassed so the
//     robot reacts instantly.
//
//  4. ADAPTIVE SPEED (slows on curves)
//     Base speed reduces as error grows, so the robot slows
//     automatically on hairpins and zigzags.
//
//  5. RECOVERY STRATEGY (3 stages)
//     Stage 1: Fast spin in last known direction (up to 1440°+)
//     Stage 2: Fast spin in opposite direction
//     Stage 3: Brief backup then expanding spiral arc
// ============================================================

#include "Modes.h"
#include "Hardware.h"
#include "Config.h"
#include "Utils.h"

// ------------------------------------------------------------
//  runMode() — dispatcher
//  Called with a Mode enum value to launch the correct mode.
// ------------------------------------------------------------
void runMode(Mode m) {
  switch (m) {
    case MODE_LINE_FOLLOW: modeLineFollow(); break;
    default: break;
  }
}

// ── PD controller state (persists across loop iterations) ────
static int   prevError   = 0;       // Error from previous loop — used for derivative
static float filteredPos = 2000.0f; // EMA-filtered sensor position
static int   lastDirection = 0;     // +1 = last turned right, -1 = left (for recovery)


// ============================================================
//  CALIBRATION
// ============================================================

// ------------------------------------------------------------
//  calibrate3Pass()
//  Spins the robot in 3 passes (right → left → right) so every
//  sensor sees both the black line and the white surface.
//  The Zumo library records min/max per sensor and uses these
//  to normalise all future readings to a 0–1000 range.
//
//  3 passes instead of 1 ensures the robot ends up roughly
//  where it started (centred on the line).
// ------------------------------------------------------------
static void calibrate3Pass() {
  display.clear();
  display.print("Calibrating");
  display.gotoXY(0, 1);
  display.print("[1/3] >>>");

  lineSensors.initFiveSensors(); // Tell the library we have 5 sensors

  // Pass 1 — sweep right (1/3 of total spins)
  motors.setSpeeds(CALIBRATION_SPEED, -CALIBRATION_SPEED);
  for (int i = 0; i < CALIBRATION_SPINS / 3; i++) {
    lineSensors.calibrate();       // Record one min/max sample per sensor
    delay(CALIBRATION_DELAY_MS);
  }

  // Pass 2 — sweep left (2/3 of total spins, crosses centre)
  display.gotoXY(0, 1);
  display.print("[2/3] <<<");
  motors.setSpeeds(-CALIBRATION_SPEED, CALIBRATION_SPEED);
  for (int i = 0; i < (CALIBRATION_SPINS * 2) / 3; i++) {
    lineSensors.calibrate();
    delay(CALIBRATION_DELAY_MS);
  }

  // Pass 3 — sweep back right to return to centre
  display.gotoXY(0, 1);
  display.print("[3/3] >>>");
  motors.setSpeeds(CALIBRATION_SPEED, -CALIBRATION_SPEED);
  for (int i = 0; i < CALIBRATION_SPINS / 3; i++) {
    lineSensors.calibrate();
    delay(CALIBRATION_DELAY_MS);
  }

  motors.setSpeeds(0, 0);
  playCalibrationDone();
  display.clear();
  display.print("Cal Done!");
  delay(500);
}


// ============================================================
//  SENSOR HELPERS
// ============================================================

// Returns the sum of all 5 sensor readings.
// Used to detect line loss — if total is very low, no sensor
// is seeing the line.
static unsigned int sensorSum() {
  unsigned int s = 0;
  for (int i = 0; i < 5; i++) s += lineValues[i];
  return s;
}

// True if total sensor sum is below LINE_LOST_THRESH.
// Means no sensor is detecting the line — robot has derailed.
static bool isLineLost() {
  return sensorSum() < LINE_LOST_THRESH;
}

// True if the leftmost sensor (index 0) is strongly activated.
// Indicates the robot is on a sharp left corner or hairpin.
static bool isOnLeftEdge() {
  return lineValues[0] > CORNER_SENSOR_THRESH;
}

// True if the rightmost sensor (index 4) is strongly activated.
// Indicates the robot is on a sharp right corner or hairpin.
static bool isOnRightEdge() {
  return lineValues[4] > CORNER_SENSOR_THRESH;
}

// True if 4 or more sensors are strongly activated.
// Indicates a T-junction or crossroads — robot drives straight through.
static bool isIntersection() {
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (lineValues[i] > INTERSECTION_THRESH) count++;
  }
  return count >= INTERSECTION_COUNT;
}


// ============================================================
//  SMOOTH DEAD ZONE SCALING
// ============================================================

// ------------------------------------------------------------
//  deadZoneScale()
//  Returns a multiplier between 0.0 and 1.0 based on error size.
//
//  This implements the hysteresis / smooth dead zone:
//    |error| < DEAD_INNER → 0.0 (no correction — silent zone)
//    |error| > DEAD_OUTER → 1.0 (full correction)
//    between              → linear ramp (smooth transition)
//
//  The PD output is multiplied by this scale, so correction
//  fades in gradually rather than snapping on at a threshold.
//  This eliminates the oscillation that causes jitter.
// ------------------------------------------------------------
static float deadZoneScale(int error) {
  int absErr = abs(error);
  if (absErr <= DEAD_INNER) return 0.0f;  // Inside dead zone — no correction
  if (absErr >= DEAD_OUTER) return 1.0f;  // Outside dead zone — full correction
  // Linear ramp between inner and outer boundaries
  return (float)(absErr - DEAD_INNER) / (float)(DEAD_OUTER - DEAD_INNER);
}


// ============================================================
//  STEERING — PD controller × smooth dead zone
// ============================================================

// ------------------------------------------------------------
//  computeSteering()
//  Calculates the steering correction to apply to motor speeds.
//
//  error      = current position error (position - set point)
//  errorDelta = change in error since last loop (for derivative)
//
//  Returns a signed float:
//    positive → steer right (add to left, subtract from right)
//    negative → steer left  (subtract from left, add to right)
// ------------------------------------------------------------
static float computeSteering(int error, int errorDelta) {
  float scale = deadZoneScale(error); // Get smooth dead zone multiplier

  // If inside the dead zone, return zero — no correction needed
  if (scale <= 0.0f) return 0.0f;

  float pTerm = LINE_KP * (float)error;       // Proportional: how far off centre
  float dTerm = LINE_KD * (float)errorDelta;  // Derivative: how fast error is changing

  // Multiply the full PD output by the smooth scale factor
  return (pTerm + dTerm) * scale;
}


// ============================================================
//  ADAPTIVE SPEED
// ============================================================

// ------------------------------------------------------------
//  computeSpeed()
//  Returns the base motor speed for this loop iteration.
//  On straights (small error) → full SPEED_STRAIGHT.
//  On curves (large error) → speed reduces proportionally.
//  Never drops below SPEED_CURVE to keep the robot moving.
// ------------------------------------------------------------
static int computeSpeed(int error) {
  int absErr = abs(error);

  // Below threshold — run at full cruise speed
  if (absErr < SPEED_ERROR_THRESH) return SPEED_STRAIGHT;

  // Above threshold — reduce speed proportionally
  float reduction = (float)(absErr - SPEED_ERROR_THRESH) * SPEED_REDUCE_RATE;
  int speed = SPEED_STRAIGHT - (int)reduction;

  // Never go below the minimum curve speed
  if (speed < SPEED_CURVE) speed = SPEED_CURVE;
  return speed;
}


// ============================================================
//  CORNER BOOST
// ============================================================

// ------------------------------------------------------------
//  applyCornerBoost()
//  When the outermost sensor is activated (sharp corner),
//  adds an extra differential push to help the robot turn.
//  Left edge → push right (reduce left, increase right).
//  Right edge → push left (increase left, reduce right).
// ------------------------------------------------------------
static void applyCornerBoost(int* leftSpeed, int* rightSpeed) {
  bool leftEdge  = isOnLeftEdge();
  bool rightEdge = isOnRightEdge();

  if (leftEdge && !rightEdge) {
    *leftSpeed  -= CORNER_BOOST;  // Slow left wheel
    *rightSpeed += CORNER_BOOST;  // Speed up right wheel → turns left
  }
  if (rightEdge && !leftEdge) {
    *leftSpeed  += CORNER_BOOST;  // Speed up left wheel → turns right
    *rightSpeed -= CORNER_BOOST;  // Slow right wheel
  }
}


// ============================================================
//  RECOVERY — FAST SPIN (Primary strategy)
// ============================================================

// ------------------------------------------------------------
//  fastSpinSearch()
//  Spins the robot in place at FAST_SPIN_SPEED, checking the
//  sensors every FAST_SPIN_DELAY_MS milliseconds.
//  Continues for up to FAST_SPIN_MAX_TIME ms (≈ 4 rotations).
//
//  direction > 0 → clockwise spin
//  direction < 0 → counterclockwise spin
//
//  Returns true immediately when the line is found.
//  Returns false if the time limit is reached without finding it.
// ------------------------------------------------------------
static bool fastSpinSearch(int direction) {
  // Set spin direction
  if (direction > 0) {
    motors.setSpeeds(FAST_SPIN_SPEED, -FAST_SPIN_SPEED);  // Clockwise
  } else {
    motors.setSpeeds(-FAST_SPIN_SPEED, FAST_SPIN_SPEED);  // Counterclockwise
  }

  unsigned long startTime = millis();

  while ((millis() - startTime) < FAST_SPIN_MAX_TIME) {
    lineSensors.readLine(lineValues); // Read sensors while spinning

    if (!isLineLost()) {
      // Line found — stop and report success
      motors.setSpeeds(0, 0);
      return true;
    }

    // Allow emergency stop during recovery
    if (buttonB.isPressed()) {
      motors.setSpeeds(0, 0);
      return false;
    }

    delay(FAST_SPIN_DELAY_MS); // Brief pause between sensor checks
  }

  motors.setSpeeds(0, 0);
  return false; // Time limit reached, line not found
}


// ============================================================
//  RECOVERY — SPIRAL SEARCH (Fallback strategy)
// ============================================================

// ------------------------------------------------------------
//  spiralSearch()
//  Creates an expanding arc by keeping one wheel (outer) at
//  constant speed while ramping the other (inner) up from slow.
//  As inner speed approaches outer, the arc gradually straightens,
//  sweeping a wider area than a pure spin.
//
//  direction > 0 → arc curves right
//  direction < 0 → arc curves left
// ------------------------------------------------------------
static bool spiralSearch(int direction) {
  int innerSpeed = SPIRAL_INNER_START; // Inner wheel starts slow
  int outerSpeed = SPIRAL_OUTER_START; // Outer wheel stays fast

  for (int step = 0; step < SPIRAL_MAX_STEPS; step++) {
    // Set motor speeds based on direction
    if (direction > 0) {
      motors.setSpeeds(outerSpeed, innerSpeed); // Right arc
    } else {
      motors.setSpeeds(innerSpeed, outerSpeed); // Left arc
    }

    delay(SPIRAL_STEP_MS);
    lineSensors.readLine(lineValues);

    if (!isLineLost()) {
      motors.setSpeeds(0, 0);
      return true; // Line found
    }

    if (buttonB.isPressed()) {
      motors.setSpeeds(0, 0);
      return false;
    }

    // Ramp up inner wheel speed — arc expands each step
    innerSpeed += SPIRAL_RAMP_RATE;
    if (innerSpeed > outerSpeed) innerSpeed = outerSpeed; // Cap at outer speed
  }

  motors.setSpeeds(0, 0);
  return false; // All steps exhausted, line not found
}


// ============================================================
//  RECOVERY — MAIN COORDINATOR
// ============================================================

// ------------------------------------------------------------
//  recoverLine()
//  Runs the 3-stage recovery sequence when the line is lost:
//
//  Stage 1: Fast spin in the last known turn direction.
//           If the robot was turning right when it lost the
//           line, the line is probably to the right — spin CW.
//
//  Stage 2: Fast spin in the opposite direction.
//           If stage 1 fails, try the other way.
//
//  Stage 3: Brief reverse then expanding spiral.
//           Backs up slightly to move away from the last
//           position, then spirals outward to sweep a wider
//           area than a spin alone.
//
//  Returns true as soon as the line is found.
//  Returns false only if all three stages are exhausted.
// ------------------------------------------------------------
static bool recoverLine() {
  motors.setSpeeds(0, 0);

  // Choose initial spin direction based on last known turn
  int searchDir = (lastDirection >= 0) ? 1 : -1;

  display.clear();
  display.print("LOST LINE");

  // ── Stage 1: spin in last known direction ─────────────────
  display.gotoXY(0, 1);
  display.print(searchDir > 0 ? "Spin CW..." : "Spin CCW...");

  if (fastSpinSearch(searchDir)) {
    playSuccessSound();
    return true;
  }

  // ── Stage 2: spin in opposite direction ───────────────────
  display.gotoXY(0, 1);
  display.print(searchDir > 0 ? "Spin CCW..." : "Spin CW...");

  if (fastSpinSearch(-searchDir)) {
    playSuccessSound();
    return true;
  }

  // ── Stage 3: back up then spiral ──────────────────────────
  display.gotoXY(0, 1);
  display.print("Spiral...");

  // Reverse briefly to move away from the lost position
  motors.setSpeeds(-SPIRAL_BACKUP_SPD, -SPIRAL_BACKUP_SPD);
  unsigned long t = millis();
  while ((millis() - t) < SPIRAL_BACKUP_MS) {
    lineSensors.readLine(lineValues);
    // Check if we reversed back onto the line
    if (!isLineLost()) {
      motors.setSpeeds(0, 0);
      playSuccessSound();
      return true;
    }
    delay(5);
  }
  motors.setSpeeds(0, 0);

  // Now spiral outward from the backup position
  if (spiralSearch(searchDir)) {
    playSuccessSound();
    return true;
  }

  // All strategies failed
  playErrorSound();
  return false;
}


// ============================================================
//  STATE RESET
// ============================================================

// ------------------------------------------------------------
//  resetState()
//  Clears PD state after a pause or recovery.
//  Prevents a stale prevError from causing a large derivative
//  spike on the first loop after resuming.
// ------------------------------------------------------------
static void resetState() {
  prevError   = 0;
  filteredPos = 2000.0f; // Reset filter to centre position
}


// ============================================================
//  MAIN LINE FOLLOW LOOP
// ============================================================

// ------------------------------------------------------------
//  modeLineFollow()
//  Entry point for line following mode.
//  Runs calibration first, then enters an infinite control loop.
//
//  Each iteration of the loop:
//    1. Check for emergency stop (Button B)
//    2. Read sensor position
//    3. Check for line loss → trigger recovery if needed
//    4. Check for intersection → drive straight through
//    5. Apply EMA filter to position (bypass on sharp corners)
//    6. Compute error from set point (2000)
//    7. Compute PD steering correction with smooth dead zone
//    8. Compute adaptive base speed
//    9. Apply corner boost if on sharp edge
//   10. Clamp and drive motors
//   11. Update display (rate-limited to avoid slowing the loop)
// ------------------------------------------------------------
void modeLineFollow() {

  // Run 3-pass calibration before starting
  calibrate3Pass();

  display.clear();
  display.print("V6 Smooth");
  display.gotoXY(0, 1);
  display.print("Starting...");
  delay(800);

  resetState();
  lastDirection = 0;
  unsigned long lastDisplayTime = 0;

  display.clear();
  display.print("Running...");

  while (true) {

    // ── Emergency stop ──────────────────────────────────────
    // Button B pauses the robot. Press again to resume.
    if (buttonB.isPressed()) {
      motors.setSpeeds(0, 0);
      display.clear();
      display.print("PAUSED");
      display.gotoXY(0, 1);
      display.print("B = Resume");
      while (buttonB.isPressed()) delay(10);          // Wait for release
      while (!buttonB.getSingleDebouncedPress()) delay(50); // Wait for press
      resetState(); // Clear stale PD state before resuming
      display.clear();
      continue;
    }

    // ── Read sensor position ────────────────────────────────
    // readLine() returns 0–4000 and fills lineValues[] array.
    int rawPos = lineSensors.readLine(lineValues);

    // ── Line lost → run recovery ────────────────────────────
    if (isLineLost()) {
      if (!recoverLine()) {
        // All recovery strategies failed — ask user to recalibrate
        display.clear();
        display.print("LOST LINE");
        display.gotoXY(0, 1);
        display.print("B=Recalib");
        while (!buttonB.getSingleDebouncedPress()) delay(50);
        calibrate3Pass();
        resetState();
        display.clear();
        display.print("Running...");
        continue;
      }
      // Recovery succeeded — re-read position and reset state
      rawPos = lineSensors.readLine(lineValues);
      resetState();
      filteredPos = (float)rawPos; // Seed filter with current position
    }

    // ── Intersection → drive straight through ───────────────
    // When 4+ sensors see the line, it's a junction — ignore it.
    if (isIntersection()) {
      motors.setSpeeds(INTERSECTION_BRAKE, INTERSECTION_BRAKE);
      delay(50);
      continue;
    }

    // ── EMA filter ──────────────────────────────────────────
    // On sharp corners, bypass the filter for instant response.
    // Otherwise, blend new reading with filtered history.
    bool onEdge = isOnLeftEdge() || isOnRightEdge();
    if (onEdge) {
      filteredPos = (float)rawPos; // Bypass filter — react instantly
    } else {
      // EMA: filtered = alpha * new + (1 - alpha) * previous
      filteredPos = (EMA_ALPHA * (float)rawPos) + ((1.0f - EMA_ALPHA) * filteredPos);
    }
    int position = (int)filteredPos;

    // ── Error from set point ────────────────────────────────
    // Set point = 2000 (centre of sensor array).
    // Positive error = line is to the right of centre.
    // Negative error = line is to the left of centre.
    int error = position - 2000;

    // Track which direction the robot is turning.
    // Used by recovery to spin toward where the line likely went.
    if (error > 80)       lastDirection =  1;  // Turning right
    else if (error < -80) lastDirection = -1;  // Turning left

    // ── PD steering with smooth dead zone ───────────────────
    int errorDelta = error - prevError; // Rate of change of error
    float steer = computeSteering(error, errorDelta);
    prevError = error; // Store for next iteration's derivative

    // ── Adaptive base speed ─────────────────────────────────
    int baseSpeed = computeSpeed(error);
    if (onEdge) baseSpeed = CORNER_SPEED; // Override speed on sharp corners

    // ── Motor speeds ────────────────────────────────────────
    // Steering is added to left and subtracted from right.
    // Positive steer → left faster than right → turns right.
    int leftSpeed  = baseSpeed + (int)steer;
    int rightSpeed = baseSpeed - (int)steer;

    // Apply extra differential boost on sharp corners
    if (onEdge) applyCornerBoost(&leftSpeed, &rightSpeed);

    // Clamp to safe motor range and drive
    leftSpeed  = clampSpeed(leftSpeed);
    rightSpeed = clampSpeed(rightSpeed);
    motors.setSpeeds(leftSpeed, rightSpeed);

    // ── Display telemetry (rate-limited) ────────────────────
    // Only update the display every DISPLAY_UPDATE_MS ms.
    // Writing to the OLED takes ~5ms — doing it every loop
    // would significantly slow the control loop.
    if ((millis() - lastDisplayTime) > DISPLAY_UPDATE_MS) {
      lastDisplayTime = millis();
      display.clear();
      if (onEdge) {
        display.print("CORNER");       // Show corner mode indicator
      } else {
        display.print("E:");
        display.print(error);          // Show current error value
      }
      display.print(" S:");
      display.print(baseSpeed);        // Show current base speed
      display.gotoXY(0, 1);
      display.print(leftSpeed);        // Show individual motor speeds
      display.print("  ");
      display.print(rightSpeed);
    }
  }
}
