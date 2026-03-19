// ============================================================
//  Hardware.cpp — Hardware object definitions and helpers
//
//  This file defines all the Zumo 32U4 hardware objects.
//  They are declared as extern in Hardware.h so every other
//  file can access them without creating duplicate instances.
// ============================================================

#include "Hardware.h"
#include "Config.h"

// ── Hardware object definitions ──────────────────────────────
// Each of these creates one instance of the Zumo library driver.
// Only defined here — all other files use the extern declarations.
Zumo32U4Motors           motors;      // Controls left and right motors
Zumo32U4Encoders         encoders;    // Reads wheel encoder tick counts
Zumo32U4LineSensors      lineSensors; // Reads the 5 IR line sensors
Zumo32U4ProximitySensors prox;        // Reads front and side proximity sensors
Zumo32U4OLED             display;     // Drives the OLED screen
Zumo32U4Buzzer           buzzer;      // Plays tones on the piezo buzzer
Zumo32U4ButtonA          buttonA;     // Left button
Zumo32U4ButtonB          buttonB;     // Middle button (used as main control)
Zumo32U4ButtonC          buttonC;     // Right button
Zumo32U4IMU              imu;         // Inertial measurement unit (accel + gyro)

// ── Shared sensor data ───────────────────────────────────────
unsigned int lineValues[5]; // Filled by lineSensors.readLine() each loop iteration
int lastError = 0;          // Stores previous error value for PD derivative term

// ------------------------------------------------------------
//  initializeHardware()
//  Called once in setup(). Shows a splash screen, starts I2C,
//  initialises the IMU, then clears the display ready to run.
// ------------------------------------------------------------
void initializeHardware() {
  // Show splash screen while hardware initialises
  display.clear();
  display.print("ZumoOS Pro");
  display.gotoXY(0, 1);
  display.print("Advanced LF");

  // Start I2C bus — required for IMU communication
  Wire.begin();

  // Initialise the IMU (accelerometer + gyroscope)
  // imu.init() returns true if the chip is found and responds
  if (imu.init()) {
    imu.enableDefault(); // Enable with default sensitivity settings
  }

  delay(1000);   // Hold splash screen for 1 second
  display.clear();
}

// ── Sound helpers ────────────────────────────────────────────
// Zumo buzzer uses a simple music notation string.
// ! = play immediately, L16 = sixteenth notes, > = octave up, < = octave down
// Letters are note names (c=C, e=E, g=G etc.)

// Ascending arpeggio — played on power-on to confirm the robot is alive
void playStartupSound()    { buzzer.play("!L16 ceg>c"); }

// Short two-note blip — played when changing modes in the menu
void playModeChangeSound() { buzzer.play("!L16 ce"); }

// Descending two-note — played when recovery fails and line cannot be found
void playErrorSound()      { buzzer.play("!L8 c<c"); }

// Three-note rising — played when line is successfully re-acquired after loss
void playSuccessSound()    { buzzer.play("!L16 ece"); }

// Longer ascending run — played at the end of sensor calibration
void playCalibrationDone() { buzzer.play("!L16 ceg>ce"); }

// ------------------------------------------------------------
//  clampSpeed()
//  Prevents motor speed values from exceeding the safe range.
//  All motor writes should go through this to avoid stalling.
// ------------------------------------------------------------
int clampSpeed(int speed) {
  if (speed > MOTOR_MAX) return MOTOR_MAX;
  if (speed < MOTOR_MIN) return MOTOR_MIN;
  return speed;
}

// ------------------------------------------------------------
//  stopMotors()
//  Immediately sets both motors to zero. Used for emergency
//  stops and before entering recovery mode.
// ------------------------------------------------------------
void stopMotors() {
  motors.setSpeeds(0, 0);
}

// ------------------------------------------------------------
//  setMotorSpeeds()
//  Convenience wrapper that clamps both speeds before driving.
//  Prefer this over calling motors.setSpeeds() directly.
// ------------------------------------------------------------
void setMotorSpeeds(int left, int right) {
  motors.setSpeeds(clampSpeed(left), clampSpeed(right));
}
