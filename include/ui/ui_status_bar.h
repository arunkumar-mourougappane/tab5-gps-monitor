#pragma once
#include <cstdint>
#include "core/config.h"
#include "core/layout.h"

// ------------------------------------------------------------ ui status bar --
// The top-bar's dynamic content: LIGHT/SLEEP chips and the right-aligned
// badge row. Redrawn on its own poll interval, independent of drawStaticChrome().

// Sized against two hard limits. Vertically the bar is TOPBAR_H (56px), so a
// badge cannot exceed that minus breathing room at each edge; 36 leaves 10px
// above and below. Horizontally the row is right-aligned and chains leftward,
// and drawStatusPill() repaints the chips before the badges -- so a row long
// enough to reach the SLEEP chip's right edge (518) would paint over it.
static constexpr int BADGE_H = 36;
static constexpr int BADGE_PAD_L = 16;
static constexpr int BADGE_PAD_R = 16;
static constexpr int BADGE_ICON_GAP = 11;
static constexpr int BADGE_GAP = 11; // spacing between adjacent badges
static constexpr int BADGE_DOT_R = 6;
// The classic font ramp has no size between Font2 (16px) and Font4 (26px) --
// Font0 is 8px and the next step up is 26 -- so the badge text stays at Font2
// while the pill around it grew. Single knob for the whole top bar: the
// badges and the LIGHT/SLEEP chips take their face from here.
#define BADGE_FONT (&fonts::Font2)

// Action chips live on the LEFT of the top bar, at fixed positions clear of the
// title. Deliberately not chained onto the right-hand badge row: those pills
// resize with their labels, which would shift these buttons' hit rects around
// under the user's finger.
static constexpr int TOPBAR_CTRL_X = 300;
static constexpr int TOPBAR_CTRL_W = 104;

Rect lightBtnRect();
Rect sleepBtnRect();

// Touch targets, deliberately larger than the chips drawn inside them. A
// fingertip covers roughly 40px on this panel and these chips are 36 tall.
Rect lightHitRect();
Rect sleepHitRect();

#if ENABLE_WIFI_NMEA
// Unlike LIGHT/SLEEP, the AP badge has no fixed position: it's the rightmost
// pill in a row that chains leftward and resizes with its neighbours' label
// widths (battery %, SD/NO SD). drawStatusPill() already computes its
// x-position while chaining the row; apBadgeRect() returns what the last
// draw call placed it at, the same way lightBtnRect() returns a constant.
Rect apBadgeRect();
Rect apBadgeHitRect();
#endif

// Exposed so sigStatusBar() (render_pipeline) can hash the same cached values
// drawStatusPill() draws from, without a second I2C poll.
extern int32_t battLevel;   // 0-100, negative if unknown
extern bool battCharging;
void refreshBattery();      // rate-limited to 1Hz internally

void drawStatusPill(uint32_t snapLastSentenceMs);
