#pragma once
#include <cstdint>
#include "layout.h"

// ---------------------------------------------------------- dimmer overlay --
static constexpr uint8_t BRIGHTNESS_MIN = 8; // slider floor: never fully dark
static constexpr uint8_t BRIGHTNESS_MAX = 255;

// Brightness overlay, opened from the LIGHT chip.
extern bool dimmerOpen;
extern bool dimmerDirty;
extern bool dimmerDragging; // slider grabbed, follows X until the lift

Rect dimmerPanelRect();
Rect dimmerTrackRect();
Rect dimmerOffRect();
Rect dimmerDoneRect();

// Grab area for the slider, padded well past the drawn track since the knob
// riding on it is taller than the track itself.
Rect dimmerTrackHitRect();

// True when the level actually moved, so a drag that hasn't crossed into the
// next step doesn't repaint the overlay or re-issue the panel command.
bool setBrightnessFromTouch(int tx);

void drawDimmer();
