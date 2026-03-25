#pragma once

// ============================================================
//  Config.h — Central configuration file
//  IA621001 Automation and Robotics — Project 1
//
//  All tunable constants live here. Changing a value here
//  affects every file that includes Config.h — no need to
//  hunt through multiple files to tune behaviour.
// ============================================================


// ============================================================
//  ENCODER & MOVEMENT
//  Used by Utils.cpp for encoder-based driving and turning.
//  1 encoder tick ≈ 0.15mm of travel on the Zumo 32U4.
// ============================================================
#define ENCODER_SPEED       250    // Motor speed used during encoder moves
#define TURN_90_TICKS       640    // Encoder ticks for a 90-degree turn
#define TURN_180_TICKS      1280   // Encoder ticks for a 180-degree turn
#define SQUARE_SIDE_TICKS   900    // Encoder ticks for one side of a square path
#define TURN_SPEED          250    // Motor speed used during turns


// ============================================================
//  UI / TIMING
// ============================================================
#define MENU_TIMEOUT_MS     10000  // How long menu waits before auto-selecting (ms)
#define BUTTON_DEBOUNCE_MS  50     // Minimum time between button presses (ms)


// ============================================================
//  CALIBRATION
//  The robot spins in place during calibration so each sensor
//  sees both the black line and the white surface.
//  The Zumo library records min/max per sensor and uses these
//  to normalise readings to 0–1000 range.
// ============================================================
#define CALIBRATION_SPINS      100   // Total number of calibration samples taken
#define CALIBRATION_DELAY_MS   15    // Delay between each sample (ms)
#define CALIBRATION_SPEED      120   // Motor speed during calibration spin


// ============================================================
//  PD CONTROLLER GAINS
//
//  The line position is read as 0 (far left) to 4000 (far right).
//  The set point (target) is 2000 — the centre of the array.
//  Error = position - 2000.
//
//  KP (proportional): how hard to steer for a given error.
//    Too high → oscillation / jitter
//    Too low  → sluggish, misses curves
//
//  KD (derivative): damps oscillation by reacting to how fast
//  the error is changing (error delta between loops).
//    Too high → twitchy, over-reacts to noise
//    Too low  → doesn't damp oscillation enough
// ============================================================
#define LINE_KP             0.15f  // Proportional gain
#define LINE_KD             1.5f   // Derivative gain


// ============================================================
//  SMOOTH DEAD ZONE (Hysteresis on steering correction)
//
//  WHY: A hard threshold causes jitter.
//    With threshold=200: error=199 → correction=0
//                        error=201 → correction=JUMP
//    The robot oscillates back and forth across this boundary.
//
//  FIX: Replace the hard threshold with a smooth ramp.
//    |error| < DEAD_INNER → scale = 0.0  (no correction at all)
//    |error| > DEAD_OUTER → scale = 1.0  (full PD correction)
//    between the two      → linear ramp  (smooth transition)
//
//  The PD output is multiplied by this scale factor, so
//  correction fades in gradually instead of snapping on.
//  This eliminates the oscillation boundary entirely.
// ============================================================
#define DEAD_INNER          150    // Below this error: zero correction applied
#define DEAD_OUTER          400    // Above this error: full PD correction applied
                                   // Between 150–400: smooth linear ramp


// ============================================================
//  ADAPTIVE SPEED
//
//  The robot runs at full SPEED_STRAIGHT on straights.
//  When error exceeds SPEED_ERROR_THRESH, speed is reduced
//  proportionally — this slows the robot on curves so it
//  doesn't overshoot hairpins or zigzags.
//  Speed never drops below SPEED_CURVE.
// ============================================================
#define SPEED_STRAIGHT      220    // Cruise speed on straight sections
#define SPEED_CURVE         80     // Minimum speed on tight curves
#define SPEED_ERROR_THRESH  500    // Error level at which slowing begins
#define SPEED_REDUCE_RATE   0.10f  // Speed reduction per unit of error above threshold


// ============================================================
//  CORNER / SHARP EDGE HANDLING
//
//  When the outermost sensor (sensor 0 or sensor 4) reads
//  above CORNER_SENSOR_THRESH, the robot is on a sharp corner
//  or hairpin bend.
//
//  In this case:
//    - Speed is overridden to CORNER_SPEED (slow)
//    - CORNER_BOOST is added as extra differential to push
//      the robot around the bend more aggressively
//    - The EMA filter is bypassed for instant response
// ============================================================
#define CORNER_SENSOR_THRESH 500   // Outer sensor value that triggers corner mode
#define CORNER_BOOST         100   // Extra differential motor push on corners
#define CORNER_SPEED         80    // Speed override when on a sharp corner


// ============================================================
//  EMA SENSOR FILTER (Exponential Moving Average)
//
//  Raw sensor position is noisy. The EMA filter smooths it:
//    filtered = alpha * new_reading + (1 - alpha) * previous
//
//  Higher alpha (closer to 1.0):
//    + More responsive to real position changes
//    - Less smoothing, more noise passes through
//
//  Lower alpha (closer to 0.0):
//    + Smoother signal, less noise
//    - More lag, slower to react to sharp turns
//
//  0.85 is a good balance — responsive but not jittery.
//  On sharp corners the filter is bypassed entirely for
//  instant response (see CORNER_SENSOR_THRESH above).
// ============================================================
#define EMA_ALPHA           0.85f  // Filter coefficient (0.0 = max smooth, 1.0 = raw)


// ============================================================
//  LINE LOSS DETECTION
//
//  The sum of all 5 sensor readings is checked each loop.
//  If the total falls below LINE_LOST_THRESH, no sensor is
//  seeing the line — the robot has derailed.
//  Recovery is then triggered automatically.
// ============================================================
#define LINE_LOST_THRESH    80     // Total sensor sum below this = line is lost


// ============================================================
//  FAST SPIN RECOVERY (Primary recovery strategy)
//
//  When the line is lost, the robot spins in place at
//  FAST_SPIN_SPEED, checking sensors every FAST_SPIN_DELAY_MS.
//  It spins up to FAST_SPIN_MAX_TIME ms — approximately
//  4 full rotations (1440°+) — before giving up.
//
//  Direction is chosen based on which way the robot was
//  turning when it lost the line (lastDirection).
//  If that fails, it tries the opposite direction.
// ============================================================
#define FAST_SPIN_SPEED      180   // Motor speed during recovery spin
#define FAST_SPIN_DELAY_MS   10    // How often to check sensors during spin (ms)
#define FAST_SPIN_MAX_TIME   3000  // Maximum spin duration before giving up (ms)


// ============================================================
//  SPIRAL SEARCH (Fallback recovery strategy)
//
//  If both spin directions fail, the robot backs up slightly
//  then performs an expanding spiral arc.
//  One wheel (outer) stays at SPIRAL_OUTER_START speed.
//  The other wheel (inner) starts at SPIRAL_INNER_START and
//  ramps up by SPIRAL_RAMP_RATE each step.
//  As inner approaches outer, the arc straightens out,
//  sweeping a wider area than a pure spin.
// ============================================================
#define SPIRAL_INNER_START   30    // Inner wheel starting speed
#define SPIRAL_OUTER_START   180   // Outer wheel speed (constant)
#define SPIRAL_RAMP_RATE     3     // How much inner speed increases per step
#define SPIRAL_STEP_MS       25    // Duration of each spiral step (ms)
#define SPIRAL_MAX_STEPS     90    // Maximum steps before giving up
#define SPIRAL_BACKUP_MS     120   // How long to reverse before spiralling (ms)
#define SPIRAL_BACKUP_SPD    90    // Reverse speed during backup


// ============================================================
//  INTERSECTION HANDLING
//
//  When 4 or more sensors read above INTERSECTION_THRESH,
//  the robot is at a T-junction or crossroads.
//  It drives straight through at INTERSECTION_BRAKE speed
//  rather than trying to follow the cross line.
// ============================================================
#define INTERSECTION_THRESH  700   // Sensor value that counts as "on line" at junction
#define INTERSECTION_COUNT   4     // How many sensors must trigger to detect intersection
#define INTERSECTION_BRAKE   130   // Speed to drive straight through intersection


// ============================================================
//  MOTOR LIMITS
//  All motor speed values are clamped to this range.
//  Zumo 32U4 motors accept -400 to +400 but we cap at ±300
//  to avoid stalling and to keep behaviour predictable.
// ============================================================
#define MOTOR_MAX            300   // Maximum forward motor speed
#define MOTOR_MIN           -300   // Maximum reverse motor speed


// ============================================================
//  DISPLAY
//  The OLED is only updated every DISPLAY_UPDATE_MS milliseconds.
//  Updating every loop iteration would slow the control loop
//  significantly — the OLED write takes ~5ms.
// ============================================================
#define DISPLAY_UPDATE_MS    300   // Minimum time between display refreshes (ms)


// ============================================================
//  OBSTACLE AVOIDANCE
//
//  OBS_DETECT_THRESH: proximity count that triggers avoidance
//    Zumo proximity returns 0–6. 2 = object within ~20cm.
//
//  OBS_CLOSE_THRESH: count that triggers backup (too close)
//    4 = object very close, need to reverse before turning.
//
//  OBS_DRIVE_SPEED:  normal forward speed
//  OBS_STEER_SPEED:  how much to differentiate motors when steering
//  OBS_TURN_SPEED:   speed during backup turn
//  OBS_BACKUP_SPEED: reverse speed when too close
//  OBS_BACKUP_MS:    how long to reverse (ms)
//  OBS_TURN_MS:      how long to turn after backup (ms)
// ============================================================
#define OBS_DETECT_THRESH  2     // Proximity count = obstacle detected
#define OBS_CLOSE_THRESH   4     // Proximity count = too close, must back up
#define OBS_DRIVE_SPEED    180   // Normal forward cruise speed
#define OBS_STEER_SPEED    120   // Motor differential when steering around obstacle
#define OBS_TURN_SPEED     150   // Turn speed after backup
#define OBS_BACKUP_SPEED   120   // Reverse speed when too close
#define OBS_BACKUP_MS      300   // How long to reverse (ms)
#define OBS_TURN_MS        400   // How long to turn after backup (ms)
