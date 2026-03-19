#pragma once

// ============================================================
//  Utils.h — Utility function declarations
//
//  General-purpose helper functions for movement, sensor
//  reading, and system diagnostics.
// ============================================================

#include <Arduino.h>

// ── Input ────────────────────────────────────────────────────
void waitForButtonPress();       // Block until any button is pressed and released

// ── Sensor calibration ───────────────────────────────────────
void calibrateLineSensors();     // Simple single-pass line sensor calibration
void calibrateProximitySensors();// Initialise proximity sensors

// ── Diagnostics ──────────────────────────────────────────────
void showSensorReadings();       // Display all sensor values on OLED
void performSystemCheck();       // Run motors and buzzer to verify hardware

// ── Math helpers ─────────────────────────────────────────────
int   mapRange(int value, int fromLow, int fromHigh, int toLow, int toHigh);
float constrainFloat(float value, float min, float max);

// ── Timing helpers ───────────────────────────────────────────
bool isTimeoutReached(unsigned long startTime, unsigned long timeoutMs);
void delayWithButtonCheck(unsigned long delayMs); // Delay that exits early on Button B

// ── Encoder-based movement ───────────────────────────────────
void resetEncoders();                        // Zero both encoder counts
void driveTicks(int ticks, int speed);       // Drive forward a set number of ticks
void driveDistance(int distance, int speed); // Drive a distance in mm (approximate)
void turnRight90();                          // Turn 90 degrees clockwise
void turnLeft90();                           // Turn 90 degrees counterclockwise
void turn180();                              // Turn 180 degrees
void turnAngle(int angle, int speed);        // Turn any angle in degrees
