// ============================================================
//  Utils.cpp — Utility functions
//
//  General-purpose helpers for movement, sensor reading,
//  diagnostics, and timing. These are not part of the core
//  line following algorithm but support testing and setup.
// ============================================================

#include "Utils.h"
#include "Hardware.h"
#include "Config.h"

// ------------------------------------------------------------
//  waitForButtonPress()
//  Blocks until any of the three buttons is pressed, then
//  waits for all buttons to be released before returning.
//  Prevents accidental double-triggers.
// ------------------------------------------------------------
void waitForButtonPress() {
  display.gotoXY(0, 3);
  display.print("Press any btn");
  // Wait for any button to be pressed
  while (!buttonA.isPressed() && !buttonB.isPressed() && !buttonC.isPressed()) delay(10);
  // Wait for all buttons to be released
  while (buttonA.isPressed() || buttonB.isPressed() || buttonC.isPressed()) delay(10);
}

// ------------------------------------------------------------
//  calibrateLineSensors()
//  Simple single-pass calibration — spins the robot once.
//  Less thorough than the 3-pass version in Modes.cpp but
//  useful for quick testing.
// ------------------------------------------------------------
void calibrateLineSensors() {
  display.clear();
  display.print("Calibrating");
  display.gotoXY(0, 1);
  display.print("Line Sensors");

  lineSensors.initFiveSensors(); // Tell library we have 5 sensors

  // Spin right while collecting calibration samples
  motors.setSpeeds(150, -150);
  for (int i = 0; i < 100; i++) {
    lineSensors.calibrate();
    delay(15);
  }
  motors.setSpeeds(0, 0);

  display.gotoXY(0, 2);
  display.print("Complete!");
  playSuccessSound();
  delay(500);
}

// ------------------------------------------------------------
//  calibrateProximitySensors()
//  Initialises the proximity sensor library.
//  No physical movement required — just sets up the driver.
// ------------------------------------------------------------
void calibrateProximitySensors() {
  display.clear();
  display.print("Calibrating");
  display.gotoXY(0, 1);
  display.print("Prox Sensors");
  prox.initThreeSensors(); // Front-left, front-right, and side sensors
  display.gotoXY(0, 2);
  display.print("Complete!");
  playSuccessSound();
  delay(500);
}

// ------------------------------------------------------------
//  showSensorReadings()
//  Displays all 5 line sensor values and 3 proximity readings
//  on the OLED. Useful for debugging sensor behaviour.
// ------------------------------------------------------------
void showSensorReadings() {
  lineSensors.read(lineValues); // Raw (uncalibrated) read
  prox.read();

  display.clear();
  // Row 0: left two sensors
  display.print("L:");
  display.print(lineValues[0]);
  display.print(" ");
  display.print(lineValues[1]);
  // Row 1: middle sensor
  display.gotoXY(0, 1);
  display.print("M:");
  display.print(lineValues[2]);
  // Row 2: right two sensors
  display.gotoXY(0, 2);
  display.print("R:");
  display.print(lineValues[3]);
  display.print(" ");
  display.print(lineValues[4]);
  // Row 3: proximity (left, front, right)
  display.gotoXY(0, 3);
  display.print("P:");
  display.print(prox.countsLeftWithLeftLeds());
  display.print(" ");
  display.print(prox.countsFrontWithLeftLeds());
  display.print(" ");
  display.print(prox.countsRightWithRightLeds());
}

// ------------------------------------------------------------
//  performSystemCheck()
//  Runs a quick hardware self-test:
//    - Drives motors forward then backward briefly
//    - Plays a buzzer tone
//  Useful for verifying hardware is working after assembly.
// ------------------------------------------------------------
void performSystemCheck() {
  display.clear();
  display.print("System Check");

  // Test motors — forward then backward
  display.gotoXY(0, 1);
  display.print("Motors...");
  motors.setSpeeds(100, 100);   // Both forward
  delay(500);
  motors.setSpeeds(-100, -100); // Both backward
  delay(500);
  motors.setSpeeds(0, 0);

  // Test buzzer
  display.gotoXY(0, 2);
  display.print("Buzzer...");
  buzzer.play("c32");
  delay(500);

  display.gotoXY(0, 3);
  display.print("Complete!");
  playSuccessSound();
  delay(1000);
}

// ------------------------------------------------------------
//  mapRange()
//  Re-maps a value from one range to another.
//  Equivalent to Arduino's map() but with explicit int types.
// ------------------------------------------------------------
int mapRange(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

// ------------------------------------------------------------
//  constrainFloat()
//  Clamps a float value between min and max.
//  Equivalent to Arduino's constrain() but for floats.
// ------------------------------------------------------------
float constrainFloat(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

// ------------------------------------------------------------
//  isTimeoutReached()
//  Returns true if more than timeoutMs milliseconds have
//  passed since startTime. Used for non-blocking timeouts.
// ------------------------------------------------------------
bool isTimeoutReached(unsigned long startTime, unsigned long timeoutMs) {
  return (millis() - startTime) > timeoutMs;
}

// ------------------------------------------------------------
//  delayWithButtonCheck()
//  Like delay() but exits early if Button B is pressed.
//  Useful for pauses that should be interruptible.
// ------------------------------------------------------------
void delayWithButtonCheck(unsigned long delayMs) {
  unsigned long startTime = millis();
  while (!isTimeoutReached(startTime, delayMs)) {
    if (buttonB.isPressed()) break;
    delay(10);
  }
}

// ------------------------------------------------------------
//  resetEncoders()
//  Zeroes both encoder counts. Call before any encoder-based
//  movement to get a clean starting reference.
// ------------------------------------------------------------
void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

// ------------------------------------------------------------
//  driveTicks()
//  Drives forward until both encoders reach the target tick
//  count. Button B can interrupt the move.
// ------------------------------------------------------------
void driveTicks(int ticks, int speed) {
  resetEncoders();
  while (encoders.getCountsLeft() < ticks && encoders.getCountsRight() < ticks) {
    motors.setSpeeds(speed, speed);
    if (buttonB.isPressed()) break;
  }
  motors.setSpeeds(0, 0);
  delay(200); // Brief settle time after stopping
}

// ------------------------------------------------------------
//  turnRight90()
//  Turns clockwise 90 degrees using encoder feedback.
//  Left wheel forward, right wheel backward.
// ------------------------------------------------------------
void turnRight90() {
  resetEncoders();
  while (abs(encoders.getCountsLeft()) < TURN_90_TICKS) {
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    if (buttonB.isPressed()) break;
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

// ------------------------------------------------------------
//  turnLeft90()
//  Turns counterclockwise 90 degrees using encoder feedback.
//  Right wheel forward, left wheel backward.
// ------------------------------------------------------------
void turnLeft90() {
  resetEncoders();
  while (abs(encoders.getCountsLeft()) < TURN_90_TICKS) {
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    if (buttonB.isPressed()) break;
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

// ------------------------------------------------------------
//  turn180()
//  Turns 180 degrees clockwise using encoder feedback.
// ------------------------------------------------------------
void turn180() {
  resetEncoders();
  while (abs(encoders.getCountsLeft()) < TURN_180_TICKS) {
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    if (buttonB.isPressed()) break;
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

// ------------------------------------------------------------
//  driveDistance()
//  Drives approximately the given distance in millimetres.
//  Conversion: 1 tick ≈ 0.15mm → multiply mm by ~7 for ticks.
//  Positive distance = forward, negative = backward.
// ------------------------------------------------------------
void driveDistance(int distance, int speed) {
  int ticks = abs(distance * 7); // Approximate mm-to-ticks conversion
  int dir = (distance >= 0) ? 1 : -1;
  resetEncoders();
  while (abs(encoders.getCountsLeft()) < ticks) {
    motors.setSpeeds(speed * dir, speed * dir);
    if (buttonB.isPressed()) break;
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

// ------------------------------------------------------------
//  turnAngle()
//  Turns any angle in degrees using encoder feedback.
//  Positive angle = clockwise, negative = counterclockwise.
//  Tick count is scaled from the 90-degree reference value.
// ------------------------------------------------------------
void turnAngle(int angle, int speed) {
  // Scale ticks proportionally from the 90-degree reference
  int ticks = (long)abs(angle) * TURN_90_TICKS / 90;
  int dir = (angle >= 0) ? 1 : -1; // Positive = clockwise
  resetEncoders();
  while (abs(encoders.getCountsLeft()) < ticks) {
    motors.setSpeeds(speed * dir, -speed * dir);
    if (buttonB.isPressed()) break;
  }
  motors.setSpeeds(0, 0);
  delay(200);
}
