#pragma once

// ============================================================
//  Hardware.h — Hardware object declarations
//
//  All Zumo 32U4 hardware objects are defined once in
//  Hardware.cpp and declared here as extern so every other
//  file can use them without re-declaring.
// ============================================================

#include <Wire.h>
#include <Zumo32U4.h>

// ── Hardware objects ─────────────────────────────────────────
// These are the actual Zumo library driver objects.
// Defined in Hardware.cpp, shared via extern here.
extern Zumo32U4Motors           motors;      // Left and right motor control
extern Zumo32U4Encoders         encoders;    // Wheel encoder tick counters
extern Zumo32U4LineSensors      lineSensors; // 5-sensor IR line sensor array
extern Zumo32U4ProximitySensors prox;        // Front/side proximity sensors
extern Zumo32U4OLED             display;     // 128x64 OLED display
extern Zumo32U4Buzzer           buzzer;      // Piezo buzzer for sounds
extern Zumo32U4ButtonA          buttonA;     // Left button (A)
extern Zumo32U4ButtonB          buttonB;     // Middle button (B) — main control
extern Zumo32U4ButtonC          buttonC;     // Right button (C)
extern Zumo32U4IMU              imu;         // Accelerometer + gyroscope

// ── Shared sensor data ───────────────────────────────────────
// lineValues[] is filled by lineSensors.readLine() each loop.
// All 5 sensor readings are stored here for use across files.
extern unsigned int lineValues[5];
extern int lastError;   // Stores previous loop's error for derivative term

// ── Hardware initialisation ──────────────────────────────────
void initializeHardware();     // Initialise all hardware, show splash screen

// ── Sound helpers ────────────────────────────────────────────
// Short buzzer melodies to give audio feedback for key events.
void playStartupSound();       // Played once on power-on
void playModeChangeSound();    // Played when switching modes in menu
void playErrorSound();         // Played when recovery fails
void playSuccessSound();       // Played when line is re-acquired
void playCalibrationDone();    // Played at end of calibration

// ── Motor helpers ────────────────────────────────────────────
int  clampSpeed(int speed);                  // Clamp to MOTOR_MIN / MOTOR_MAX
void stopMotors();                           // Set both motors to 0
void setMotorSpeeds(int left, int right);    // Set speeds with clamping applied
