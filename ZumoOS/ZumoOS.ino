// ============================================================
//  ZumoOS.ino — Main entry point
//  IA621001 Automation and Robotics — Project 1
//  Zumo 32U4 Line Follower
//
//  This file contains only setup() and loop().
//  All logic is split across:
//    Config.h    — tunable constants
//    Hardware.h/cpp — hardware objects and sound helpers
//    Modes.h/cpp — line following algorithm
//    Menu.h/cpp  — display menu
//    Utils.h/cpp — movement and sensor utilities
// ============================================================

#include "Hardware.h"
#include "Menu.h"
#include "Modes.h"
#include "Config.h"

// ------------------------------------------------------------
//  setup()
//  Runs once on power-on or reset.
//  Initialises all hardware, plays a startup sound, then waits
//  briefly before entering the main loop.
// ------------------------------------------------------------
void setup() {
  Serial.begin(9600);       // Open serial port for debug output
  initializeHardware();     // Set up motors, sensors, IMU, display
  playStartupSound();       // Play ascending tone to confirm power-on
  delay(500);               // Short pause before showing the menu
}

// ------------------------------------------------------------
//  loop()
//  Runs repeatedly after setup().
//  Shows the home screen and waits for Button B to be pressed,
//  then launches the line following mode.
//  When line following exits (it doesn't normally), loop()
//  returns here and shows the home screen again.
// ------------------------------------------------------------
void loop() {
  // Show home screen on the OLED display
  display.clear();
  display.print("ZumoOS Pro");
  display.gotoXY(0, 1);
  display.print("Advanced LF");
  display.gotoXY(0, 3);
  display.print("Press B");         // Prompt user to press Button B

  // Wait here until Button B is pressed (debounced single press)
  while (!buttonB.getSingleDebouncedPress()) {
    delay(50);
  }

  // Launch line following — this runs its own internal loop
  // and only returns if something goes very wrong
  modeLineFollow();
}
