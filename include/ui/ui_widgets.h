#pragma once
#include <cstdint>
#include "core/layout.h"
#include "core/display.h" // for lgfx::IFont, via M5Unified.h

// ------------------------------------------------------------- ui widgets --
// Generic drawing primitives: no domain knowledge, no top-bar-specific sizing.

int drawHeroValue(int x, int y, int w, const char *label, const char *value, uint8_t bigSize = 2);
void drawMiniStat(int x, int y, const char *label, const char *value);

// Rounded pill with a leading text (no dot) -- used for fix/sat count badges.
int drawBadge(int x, int y, const char *text, uint16_t bg, uint16_t fg);

void drawCardFrame(const Rect &r, const char *label, uint16_t accent);

// Press feedback is a colour swap rather than an inset/shrink: an inset would
// leave a ring of the previous fill behind unless the surrounding background
// were also repainted, and that background differs per call site.
void drawChip(const Rect &c, const char *label, uint16_t bg, uint16_t fg, bool pressed,
              const lgfx::IFont *font = &fonts::Font2);

// Ring-buffer line chart. `count`/`head` follow the same convention as the
// raw-log ring buffer: `head` is the next write index, the most recent
// sample is at (head-1).
void drawSparkline(int x, int y, int w, int h, const float *values, int count, uint32_t head,
                    uint16_t color, float maxVal);
