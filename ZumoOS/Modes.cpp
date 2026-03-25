// ============================================================
//  Modes.cpp — Obstacle Avoidance
//  Zumo 32U4
//
//  HOW IT WORKS:
//  The robot drives forward continuously. The front proximity
//  sensors detect objects ahead. Depending on what is seen:
//
//    FORWARD  — nothing detected, drive straight
//    AVOIDING — obstacle on one side, steer away while moving
//    BACKUP   — obstacle too close on both sides, reverse
//               then turn toward the clearer side
//
//  Proximity sensor layout:
//    countsFrontWithLeftLeds()  — front-left IR sensor
//    countsFrontWithRightLeds() — front-right IR sensor
//    countsLeftWithLeftLeds()   — left side sensor
//    countsRightWithRightLeds() — right side sensor
//  Each returns 0-6. Higher count = object is closer.
// ============================================================

#include "Modes.h"
#include "Hardware.h"
#include "Config.h"

static int clamp(int v) {
  if (v > MOTOR_MAX) return MOTOR_MAX;
  if (v < MOTOR_MIN) return MOTOR_MIN;
  return v;
}

struct ProxData {
  int frontLeft;
  int frontRight;
  int left;
  int right;
};

static ProxData readAllProx() {
  prox.read();
  ProxData d;
  d.frontLeft  = prox.countsFrontWithLeftLeds();
  d.frontRight = prox.countsFrontWithRightLeds();
  d.left       = prox.countsLeftWithLeftLeds();
  d.right      = prox.countsRightWithRightLeds();
  return d;
}

void modeObstacleAvoid() {
  prox.initThreeSensors();

  display.clear();
  display.print("Obstacle Bot");
  display.gotoXY(0, 1);
  display.print("Running...");

  unsigned long lastDisplayTime = 0;

  while (true) {

    if (buttonB.isPressed()) {
      motors.setSpeeds(0, 0);
      display.clear();
      display.print("PAUSED");
      display.gotoXY(0, 1);
      display.print("B = Resume");
      while (buttonB.isPressed()) delay(10);
      while (!buttonB.getSingleDebouncedPress()) delay(50);
      display.clear();
      display.print("Running...");
      continue;
    }

    ProxData p = readAllProx();

    bool obstacleAhead = (p.frontLeft  >= OBS_DETECT_THRESH ||
                          p.frontRight >= OBS_DETECT_THRESH);
    bool tooClose      = (p.frontLeft  >= OBS_CLOSE_THRESH &&
                          p.frontRight >= OBS_CLOSE_THRESH);

    if (tooClose) {
      // BACKUP: too close on both sides, reverse then turn
      display.clear();
      display.print("TOO CLOSE");
      display.gotoXY(0, 1);
      display.print("Reversing...");

      motors.setSpeeds(-OBS_BACKUP_SPEED, -OBS_BACKUP_SPEED);
      delay(OBS_BACKUP_MS);
      motors.setSpeeds(0, 0);
      delay(50);

      // Turn toward side with more space (lower count = more open)
      if (p.left <= p.right) {
        motors.setSpeeds(-OBS_TURN_SPEED, OBS_TURN_SPEED); // turn left
      } else {
        motors.setSpeeds(OBS_TURN_SPEED, -OBS_TURN_SPEED); // turn right
      }
      delay(OBS_TURN_MS);
      motors.setSpeeds(0, 0);
      delay(50);

    } else if (obstacleAhead) {
      // AVOIDING: steer away while still moving forward
      // frontLeft > frontRight = obstacle on left = steer right
      // frontRight > frontLeft = obstacle on right = steer left
      if (p.frontLeft >= p.frontRight) {
        motors.setSpeeds(clamp(OBS_DRIVE_SPEED + OBS_STEER_SPEED),
                         clamp(OBS_DRIVE_SPEED - OBS_STEER_SPEED));
      } else {
        motors.setSpeeds(clamp(OBS_DRIVE_SPEED - OBS_STEER_SPEED),
                         clamp(OBS_DRIVE_SPEED + OBS_STEER_SPEED));
      }

      if ((millis() - lastDisplayTime) > DISPLAY_UPDATE_MS) {
        lastDisplayTime = millis();
        display.clear();
        display.print("AVOIDING");
        display.gotoXY(0, 1);
        display.print("L:"); display.print(p.frontLeft);
        display.print(" R:"); display.print(p.frontRight);
      }

    } else {
      // FORWARD: path clear, drive straight
      motors.setSpeeds(OBS_DRIVE_SPEED, OBS_DRIVE_SPEED);

      if ((millis() - lastDisplayTime) > DISPLAY_UPDATE_MS) {
        lastDisplayTime = millis();
        display.clear();
        display.print("FORWARD");
        display.gotoXY(0, 1);
        display.print("L:"); display.print(p.frontLeft);
        display.print(" R:"); display.print(p.frontRight);
      }
    }
  }
}
