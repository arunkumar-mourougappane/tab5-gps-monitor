#pragma once
#include "model/render_snapshot.h"

// ---------------------------------------------------------- ui sky panel --

// Builds the pre-rendered radar background sprite. Call once from setup(),
// after computeLayout() -- the sprite's size is derived from skyCard.
void buildRadarSprite();

void drawSatPanel(const RenderSnapshot &s);

// Finger-sized hit target against the last-drawn dot positions; -1 if none
// qualifies. Used by app_input's touch dispatch.
int hitTestSkyDot(int px, int py);

// The tooltip for whichever dot was last tapped. skyDots[] stays private to
// this module -- app_input sets the tooltip by hitTestSkyDot() index rather
// than reaching into the dot array itself.
void setSatTooltip(int dotIndex);
void clearSatTooltip();
bool satTooltipActive();

// True while the tooltip is up and hasn't hit its 4s self-expiry yet -- a
// read-only check (unlike satTooltipActive(), drawSatTooltip() is what
// actually flips active off once expired). Used by sigSatPanel() to fold
// the expiry into its own repaint trigger without a draw call.
bool satTooltipVisible(uint32_t nowMs);
SatInfo satTooltipInfo(); // valid only when satTooltipVisible()
