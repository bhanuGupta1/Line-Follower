// ============================================================
//  LineFollower.ino — Zumo 32U4
//  IA621001 Automation and Robotics — Project 1
//
//  Single-file implementation.
//
//  Structure:
//    1. Constants & configuration
//    2. Hardware objects & globals
//    3. Sensor functions   — calibration, filtered reads
//    4. Motor functions    — PD steering, speed, corner boost
//    5. Recovery functions — spin search, spiral fallback
//    6. setup() / loop()
// ============================================================

#include <Wire.h>
#include <Zumo32U4.h>

// ============================================================
//  1. CONSTANTS & CONFIGURATION
//  All tunable values in one place — change here, affects all
// ============================================================

// Sensors
#define NUM_SENSORS          5

// Calibration
#define CALIBRATION_SPEED    120     // Motor speed during calibration spin
#define CALIBRATION_SPINS    100     // Total calibration samples
#define CALIBRATION_DELAY_MS 15      // Delay between samples (ms)

// PD Controller — set point is the centre of the sensor array
#define LINE_CENTER          2000    // Sensor range 0–4000, centre = 2000
#define KP                   0.15f   // Proportional gain
#define KD                   1.5f    // Derivative gain

// Smooth Dead Zone (hysteresis on steering correction)
//   Eliminates jitter caused by hard threshold boundaries.
//   |error| < DEAD_INNER → 0% correction  (silent zone)
//   |error| > DEAD_OUTER → 100% correction (full PD)
//   between              → linear ramp    (smooth transition)
#define DEAD_INNER           150
#define DEAD_OUTER           400

// Speed
#define SPEED_STRAIGHT       220     // Cruise speed on straights
#define SPEED_CURVE          80      // Minimum speed on tight curves
#define SPEED_REDUCE_START   500     // Error level to begin slowing
#define SPEED_REDUCE_RATE    0.10f   // Speed reduction per error unit above threshold

// Corner / sharp edge handling
#define CORNER_SENSOR_THRESH 500     // Outer sensor value that indicates sharp edge
#define CORNER_SPEED         80      // Speed override on sharp edge
#define CORNER_BOOST         100     // Extra differential push on corners

// EMA filter — smooths noisy position signal
//   Higher alpha = more responsive, less smoothing
#define EMA_ALPHA            0.85f

// Line loss
#define LINE_LOST_THRESH     80      // Total sensor sum below this = line lost

// Recovery: fast spin (primary strategy)
#define SPIN_SPEED           180     // Spin motor speed
#define SPIN_CHECK_MS        10      // Sensor check interval during spin (ms)
#define SPIN_MAX_MS          3000    // Max spin duration — approx 4 full rotations

// Recovery: spiral search (fallback strategy)
#define SPIRAL_INNER_START   30
#define SPIRAL_OUTER_START   180
#define SPIRAL_RAMP_RATE     3
#define SPIRAL_STEP_MS       25
#define SPIRAL_MAX_STEPS     90
#define SPIRAL_BACKUP_SPD    90
#define SPIRAL_BACKUP_MS     120

// Motor limits
#define MOTOR_MAX            300
#define MOTOR_MIN           -300

// Display refresh rate
#define DISPLAY_UPDATE_MS    300


// ============================================================
//  2. HARDWARE OBJECTS & GLOBALS
// ============================================================

Zumo32U4Motors      motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED        display;
Zumo32U4Buzzer      buzzer;
Zumo32U4ButtonB     buttonB;

unsigned int sensorValues[NUM_SENSORS];  // Raw calibrated sensor readings

// PD state
static int   prevError   = 0;
static float filteredPos = LINE_CENTER;
static int   lastDirection = 0;          // +1 = last turned right, -1 = left


// ============================================================
//  3. SENSOR FUNCTIONS
// ============================================================

// ------------------------------------------------------------
//  calibrateSensors()
//  3-pass sweep exposes sensors to both black line and white
//  surface so the library learns min/max per sensor.
// ------------------------------------------------------------
void calibrateSensors() {
  lineSensors.initFiveSensors();
  display.clear();
  display.print("Calibrating");

  // Pass 1 — sweep right
  display.gotoXY(0, 1);
  display.print("[1/3] >>>");
  motors.setSpeeds(CALIBRATION_SPEED, -CALIBRATION_SPEED);
  for (int i = 0; i < CALIBRATION_SPINS / 3; i++) {
    lineSensors.calibrate();
    delay(CALIBRATION_DELAY_MS);
  }

  // Pass 2 — sweep left (double to cross centre)
  display.gotoXY(0, 1);
  display.print("[2/3] <<<");
  motors.setSpeeds(-CALIBRATION_SPEED, CALIBRATION_SPEED);
  for (int i = 0; i < (CALIBRATION_SPINS * 2) / 3; i++) {
    lineSensors.calibrate();
    delay(CALIBRATION_DELAY_MS);
  }

  // Pass 3 — sweep back right to centre
  display.gotoXY(0, 1);
  display.print("[3/3] >>>");
  motors.setSpeeds(CALIBRATION_SPEED, -CALIBRATION_SPEED);
  for (int i = 0; i < CALIBRATION_SPINS / 3; i++) {
    lineSensors.calibrate();
    delay(CALIBRATION_DELAY_MS);
  }

  motors.setSpeeds(0, 0);
  buzzer.play("!L16 ceg>ce");
  display.clear();
  display.print("Cal Done!");
  delay(600);
}

// ------------------------------------------------------------
//  readLineSensors()
//  Returns calibrated weighted position 0 (far left) – 4000
//  (far right). Updates sensorValues[] as a side effect.
// ------------------------------------------------------------
int readLineSensors() {
  return (int)lineSensors.readLine(sensorValues);
}

// ------------------------------------------------------------
//  applyEMA()
//  Exponential Moving Average filter on position signal.
// ------------------------------------------------------------
float applyEMA(float current, float newVal) {
  return (EMA_ALPHA * newVal) + ((1.0f - EMA_ALPHA) * current);
}

// ------------------------------------------------------------
//  isLineLost() — total sensor sum below threshold
// ------------------------------------------------------------
bool isLineLost() {
  unsigned int total = 0;
  for (int i = 0; i < NUM_SENSORS; i++) total += sensorValues[i];
  return total < LINE_LOST_THRESH;
}

// ------------------------------------------------------------
//  Edge helpers — outermost sensors detect hairpins/corners
// ------------------------------------------------------------
bool isOnLeftEdge()  { return sensorValues[0]              > CORNER_SENSOR_THRESH; }
bool isOnRightEdge() { return sensorValues[NUM_SENSORS - 1] > CORNER_SENSOR_THRESH; }
bool isOnSharpEdge() { return isOnLeftEdge() || isOnRightEdge(); }


// ============================================================
//  4. MOTOR FUNCTIONS
// ============================================================

// ------------------------------------------------------------
//  clampSpeed() — enforce motor limits
// ------------------------------------------------------------
int clampSpeed(int speed) {
  if (speed > MOTOR_MAX) return MOTOR_MAX;
  if (speed < MOTOR_MIN) return MOTOR_MIN;
  return speed;
}

// ------------------------------------------------------------
//  stopMotors()
// ------------------------------------------------------------
void stopMotors() {
  motors.setSpeeds(0, 0);
}

// ------------------------------------------------------------
//  pauseUntilResume() — B button pause/resume
// ------------------------------------------------------------
void pauseUntilResume() {
  display.clear();
  display.print("PAUSED");
  display.gotoXY(0, 1);
  display.print("B = Resume");
  while (buttonB.isPressed()) delay(10);
  while (!buttonB.getSingleDebouncedPress()) delay(50);
  display.clear();
  display.print("Running...");
}

// ------------------------------------------------------------
//  computeSteering()
//
//  PD controller with smooth dead zone (hysteresis).
//
//  Problem with a hard threshold:
//    error=149 → correction=0
//    error=151 → correction=JUMP
//    Robot oscillates across the boundary → jitter
//
//  Solution — smooth ramp between DEAD_INNER and DEAD_OUTER:
//    scale=0.0 below DEAD_INNER (no correction)
//    scale=1.0 above DEAD_OUTER (full PD correction)
//    linear ramp between them   (smooth transition)
// ------------------------------------------------------------
float computeSteering(int error, int errorDelta) {
  int absErr = abs(error);

  float scale;
  if      (absErr <= DEAD_INNER) scale = 0.0f;
  else if (absErr >= DEAD_OUTER) scale = 1.0f;
  else scale = (float)(absErr - DEAD_INNER) / (float)(DEAD_OUTER - DEAD_INNER);

  if (scale <= 0.0f) return 0.0f;

  float pTerm = KP * (float)error;
  float dTerm = KD * (float)errorDelta;
  return (pTerm + dTerm) * scale;
}

// ------------------------------------------------------------
//  computeBaseSpeed()
//  Full speed on straights, reduces on curves to prevent
//  overshooting hairpins and zigzags.
// ------------------------------------------------------------
int computeBaseSpeed(int error) {
  int absErr = abs(error);
  if (absErr < SPEED_REDUCE_START) return SPEED_STRAIGHT;
  float reduction = (float)(absErr - SPEED_REDUCE_START) * SPEED_REDUCE_RATE;
  int speed = SPEED_STRAIGHT - (int)reduction;
  return (speed < SPEED_CURVE) ? SPEED_CURVE : speed;
}

// ------------------------------------------------------------
//  driveWithSteering()
//  Combines base speed + PD steering + corner boost, then
//  clamps and drives both motors.
// ------------------------------------------------------------
void driveWithSteering(int baseSpeed, float steering) {
  if (isOnSharpEdge()) {
    baseSpeed = CORNER_SPEED;
    steering += isOnLeftEdge() ? CORNER_BOOST : -CORNER_BOOST;
  }
  motors.setSpeeds(
    clampSpeed(baseSpeed + (int)steering),
    clampSpeed(baseSpeed - (int)steering)
  );
}

// ------------------------------------------------------------
//  updateLastDirection()
//  Tracks which way the robot was turning — used by recovery
//  to spin toward where the line most likely went.
// ------------------------------------------------------------
void updateLastDirection(int error) {
  if      (error >  80) lastDirection =  1;
  else if (error < -80) lastDirection = -1;
}


// ============================================================
//  5. RECOVERY FUNCTIONS
// ============================================================

// ------------------------------------------------------------
//  fastSpin()
//  Spins in place at SPIN_SPEED, checking sensors every
//  SPIN_CHECK_MS for up to SPIN_MAX_MS (≈ 4 full rotations).
// ------------------------------------------------------------
static bool fastSpin(int direction) {
  motors.setSpeeds(
    direction >= 0 ?  SPIN_SPEED : -SPIN_SPEED,
    direction >= 0 ? -SPIN_SPEED :  SPIN_SPEED
  );

  unsigned long start = millis();
  while ((millis() - start) < SPIN_MAX_MS) {
    readLineSensors();
    if (!isLineLost())          { stopMotors(); return true;  }
    if (buttonB.isPressed())    { stopMotors(); return false; }
    delay(SPIN_CHECK_MS);
  }
  stopMotors();
  return false;
}

// ------------------------------------------------------------
//  spiralSearch()
//  Expanding arc — outer wheel constant, inner wheel ramps up.
//  Sweeps a wider area than a pure spin.
// ------------------------------------------------------------
static bool spiralSearch(int direction) {
  int inner = SPIRAL_INNER_START;
  int outer = SPIRAL_OUTER_START;

  for (int step = 0; step < SPIRAL_MAX_STEPS; step++) {
    motors.setSpeeds(
      direction >= 0 ? outer : inner,
      direction >= 0 ? inner : outer
    );
    delay(SPIRAL_STEP_MS);
    readLineSensors();
    if (!isLineLost())       { stopMotors(); return true;  }
    if (buttonB.isPressed()) { stopMotors(); return false; }
    inner += SPIRAL_RAMP_RATE;
    if (inner > outer) inner = outer;
  }
  stopMotors();
  return false;
}

// ------------------------------------------------------------
//  recoverLine()
//  3-stage recovery:
//    1. Fast spin in last known direction
//    2. Fast spin in opposite direction
//    3. Brief backup + expanding spiral
//  Returns true when line found, false if all stages fail.
// ------------------------------------------------------------
bool recoverLine() {
  int dir = (lastDirection != 0) ? lastDirection : 1;

  display.clear();
  display.print("LOST LINE");

  // Stage 1
  display.gotoXY(0, 1);
  display.print(dir > 0 ? "Spin CW..." : "Spin CCW...");
  if (fastSpin(dir))  { buzzer.play("!L16 ece"); return true; }

  // Stage 2
  display.gotoXY(0, 1);
  display.print(dir > 0 ? "Spin CCW..." : "Spin CW...");
  if (fastSpin(-dir)) { buzzer.play("!L16 ece"); return true; }

  // Stage 3 — back up then spiral
  display.gotoXY(0, 1);
  display.print("Spiral...");
  motors.setSpeeds(-SPIRAL_BACKUP_SPD, -SPIRAL_BACKUP_SPD);
  unsigned long t = millis();
  while ((millis() - t) < SPIRAL_BACKUP_MS) {
    readLineSensors();
    if (!isLineLost()) { stopMotors(); buzzer.play("!L16 ece"); return true; }
    delay(5);
  }
  stopMotors();
  if (spiralSearch(dir)) { buzzer.play("!L16 ece"); return true; }

  buzzer.play("!L8 c<c");
  return false;
}


// ============================================================
//  6. SETUP & LOOP
// ============================================================

void setup() {
  Serial.begin(9600);

  display.clear();
  display.print("Line Follower");
  display.gotoXY(0, 1);
  display.print("Press B");
  buzzer.play("!L16 ceg>c");

  while (!buttonB.getSingleDebouncedPress()) delay(50);

  calibrateSensors();

  prevError   = 0;
  filteredPos = LINE_CENTER;

  display.clear();
  display.print("Running...");
  delay(500);
}

// loop() is intentionally thin — all logic lives in functions above
void loop() {

  // Emergency pause
  if (buttonB.isPressed()) {
    stopMotors();
    pauseUntilResume();
    prevError   = 0;
    filteredPos = LINE_CENTER;
    return;
  }

  // Read sensors
  int rawPos = readLineSensors();

  // Line lost — attempt recovery
  if (isLineLost()) {
    stopMotors();
    if (!recoverLine()) {
      display.clear();
      display.print("LOST LINE");
      display.gotoXY(0, 1);
      display.print("B=Recalib");
      while (!buttonB.getSingleDebouncedPress()) delay(50);
      calibrateSensors();
    }
    prevError   = 0;
    filteredPos = LINE_CENTER;
    return;
  }

  // EMA filter — bypass on sharp edge for instant response
  filteredPos = isOnSharpEdge()
    ? (float)rawPos
    : applyEMA(filteredPos, (float)rawPos);

  int position   = (int)filteredPos;
  int error      = position - LINE_CENTER;
  int errorDelta = error - prevError;
  prevError      = error;

  float steering = computeSteering(error, errorDelta);
  int   baseSpeed = computeBaseSpeed(error);

  driveWithSteering(baseSpeed, steering);
  updateLastDirection(error);
}
