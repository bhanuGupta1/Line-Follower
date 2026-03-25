#pragma once

// ============================================================
//  Config.h — Obstacle Avoidance Configuration
//  All tunable constants in one place.
// ============================================================

// ── Motor limits ─────────────────────────────────────────────
#define MOTOR_MAX            300   // Maximum forward motor speed
#define MOTOR_MIN           -300   // Maximum reverse motor speed

// ── Display ──────────────────────────────────────────────────
// OLED is only refreshed every DISPLAY_UPDATE_MS to avoid
// slowing the main control loop (each write takes ~5ms).
#define DISPLAY_UPDATE_MS    300

// ── Encoder & turns ──────────────────────────────────────────
// Used by Utils.cpp for encoder-based turning.
// 1 encoder tick ≈ 0.15mm travel on the Zumo 32U4.
#define TURN_90_TICKS        640   // Ticks for a 90-degree turn
#define TURN_180_TICKS       1280  // Ticks for a 180-degree turn
#define TURN_SPEED           250   // Motor speed during turns

// ============================================================
//  OBSTACLE AVOIDANCE
//
//  The robot drives forward and uses the front proximity
//  sensors to detect objects. When something is detected:
//    - If one side is clearer → steer away while still moving
//    - If both sides blocked  → reverse then turn
//
//  Proximity sensor returns 0–6 counts. Higher = closer.
//
//  OBS_DETECT_THRESH: count that starts avoidance steering
//  OBS_CLOSE_THRESH:  count that forces a backup (too close)
//  OBS_DRIVE_SPEED:   normal cruise speed
//  OBS_STEER_SPEED:   motor differential during steering
//  OBS_BACKUP_SPEED:  reverse speed when too close
//  OBS_BACKUP_MS:     how long to reverse (ms)
//  OBS_TURN_SPEED:    turn speed after backup
//  OBS_TURN_MS:       how long to turn after backup (ms)
// ============================================================
#define OBS_DETECT_THRESH  2     // Proximity count = obstacle detected (~20cm)
#define OBS_CLOSE_THRESH   4     // Proximity count = too close, must back up
#define OBS_DRIVE_SPEED    180   // Normal forward cruise speed
#define OBS_STEER_SPEED    120   // Motor differential when steering around obstacle
#define OBS_BACKUP_SPEED   120   // Reverse speed when too close
#define OBS_BACKUP_MS      300   // How long to reverse (ms)
#define OBS_TURN_SPEED     150   // Turn speed after backup
#define OBS_TURN_MS        400   // How long to turn after backup (ms)
