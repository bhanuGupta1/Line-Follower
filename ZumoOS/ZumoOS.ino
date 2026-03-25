// ============================================================
//  ZumoOS.ino — Obstacle Avoidance Robot
//  Zumo 32U4
//
//  Boots straight into obstacle avoidance.
//  No menu, no line following.
//  Press B to start, press B again to pause/resume.
// ============================================================

#include "Hardware.h"
#include "Modes.h"
#include "Config.h"

void setup() {
  Serial.begin(9600);
  initializeHardware();
  playStartupSound();
  delay(500);
}

void loop() {
  display.clear();
  display.print("Obstacle Bot");
  display.gotoXY(0, 1);
  display.print("Press B Start");

  while (!buttonB.getSingleDebouncedPress()) delay(50);

  modeObstacleAvoid(); // runs its own internal loop
}
