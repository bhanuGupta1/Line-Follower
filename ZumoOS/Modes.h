#pragma once

// ============================================================
//  Modes.h — Mode enum and function declarations
//
//  Defines the available operating modes as an enum.
//  Add new modes here and implement them in Modes.cpp.
// ============================================================

// Enum of all available robot modes.
// MODE_COUNT must always be last — it gives the total count
// and is used for menu wrapping logic.
enum Mode {
  MODE_LINE_FOLLOW,   // PD line following with recovery
  MODE_MAZE,          // Left-hand wall following maze solver
  MODE_COUNT          // Always last — used as array size / wrap value
};

// Dispatcher — calls the correct mode function for a given Mode value
void runMode(Mode m);

// Main line following mode — runs its own internal loop
void modeLineFollow();

// Left-hand wall following maze solver
void modeMazeSolve();
