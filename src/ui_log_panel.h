#pragma once
#include "layout.h"
#include "render_snapshot.h"

// ---------------------------------------------------------- ui log panel --

Rect nmeaFilterChipRect();
// Padded touch target -- the chip itself is only 26px tall.
Rect nmeaFilterHitRect();

// Drawn from drawLogPanel() (per frame, so it can animate) as well as from
// layout.cpp's relayout() (so it survives a full repaint).
void drawFilterChip();

void drawLogPanel(const RenderSnapshot &s);
