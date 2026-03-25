// ============================================================
//  Menu.cpp — Mode selection menu
//
//  Displays a scrollable list of modes on the OLED.
//  Button A cycles forward, Button C cycles backward,
//  Button B confirms and launches the selected mode.
// ============================================================

#include "Menu.h"
#include "Hardware.h"
#include "Config.h"

// Array of human-readable mode names, indexed by the Mode enum.
// Must stay in sync with the Mode enum in Modes.h.
static const char* modeNames[] = {
  "Line Follow",    // MODE_LINE_FOLLOW
  "Obstacle Avoid"  // MODE_OBSTACLE
};

// ------------------------------------------------------------
//  getModeName()
//  Returns the display string for a given Mode enum value.
//  Returns "Unknown" if the mode value is out of range.
// ------------------------------------------------------------
const char* getModeName(Mode mode) {
  if (mode >= 0 && mode < MODE_COUNT) return modeNames[mode];
  return "Unknown";
}

// ------------------------------------------------------------
//  showMenu()
//  Draws the menu header line on the display.
//  Called once when entering the menu, and again after
//  returning from a mode.
// ------------------------------------------------------------
void showMenu() {
  display.clear();
  display.print("A/C:Chg B:Run");  // Button hint on top row
}

// ------------------------------------------------------------
//  selectMode()
//  Blocking loop that handles menu navigation.
//  - Button A: cycle forward through modes
//  - Button C: cycle backward through modes
//  - Button B: confirm selection and return chosen mode
//
//  Uses a static variable so the last selected mode is
//  remembered if the user returns to the menu.
// ------------------------------------------------------------
Mode selectMode() {
  static Mode currentMode = MODE_LINE_FOLLOW; // Remembers last selection

  showMenu();
  display.gotoXY(0, 1);
  display.print(getModeName(currentMode));  // Show current mode on row 2
  delay(100);

  Mode lastDisplayedMode = currentMode;

  while (true) {
    // Only redraw the mode name if it has changed — avoids flicker
    if (currentMode != lastDisplayedMode) {
      display.gotoXY(0, 1);
      display.print("                ");   // Clear the row first
      display.gotoXY(0, 1);
      display.print(getModeName(currentMode));
      lastDisplayedMode = currentMode;
    }

    // Button A — cycle forward (wraps around)
    if (buttonA.getSingleDebouncedPress()) {
      currentMode = (Mode)((currentMode + 1) % MODE_COUNT);
      playModeChangeSound();
    }

    // Button C — cycle backward (wraps around)
    if (buttonC.getSingleDebouncedPress()) {
      currentMode = (Mode)((currentMode - 1 + MODE_COUNT) % MODE_COUNT);
      playModeChangeSound();
    }

    // Button B — confirm selection
    if (buttonB.getSingleDebouncedPress()) {
      displayModeInfo(currentMode);  // Show brief confirmation screen
      return currentMode;
    }

    delay(50); // Small delay to avoid hammering button reads
  }
}

// ------------------------------------------------------------
//  displayModeInfo()
//  Shows a brief confirmation screen before the mode starts.
//  Gives the user a moment to position the robot.
// ------------------------------------------------------------
void displayModeInfo(Mode mode) {
  display.clear();
  display.print("Starting:");
  display.gotoXY(0, 1);
  display.print(getModeName(mode));
  display.gotoXY(0, 2);
  display.print("Press B=Stop");  // Remind user how to pause
  delay(1000);                    // Hold for 1 second before starting
}
