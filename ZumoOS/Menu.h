#pragma once

// ============================================================
//  Menu.h — Mode selection menu declarations
//
//  Provides functions for displaying and navigating the
//  mode selection menu on the OLED display.
// ============================================================

#include "Modes.h"

// Returns the display name string for a given Mode enum value
const char* getModeName(Mode mode);

// Draws the menu header on the display
void showMenu();

// Blocking menu loop — waits for button input and returns
// the selected Mode when Button B is pressed
Mode selectMode();

// Shows a brief "Starting: <mode name>" confirmation screen
// before the selected mode begins
void displayModeInfo(Mode mode);
