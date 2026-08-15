#pragma once

// ----------------------------------------------------------------- layout --

struct Rect { int x, y, w, h; };

extern int SCREEN_W;
extern int SCREEN_H;
static constexpr int MARGIN = 16;
static constexpr int CARD_RADIUS = 14;
static constexpr int TOPBAR_H = 56;
static constexpr int LOGCARD_H = 190;
static constexpr int CARD_HEADER_H = 36;

extern Rect fixCard, skyCard, logCard;

// Owned by the log panel; computeLayout() needs it to decide whether the log
// card takes the whole content area.
extern bool logExpanded;

Rect innerOf(const Rect &r);
bool pointInRect(int px, int py, const Rect &r);

// ------------------------------------------------------------- sky plot --
// The plot is a north-up polar projection with a fixed instrument bezel around
// it: 10 degree minor ticks, 30 degree major ticks, cardinals, a white index at
// north and the elevation rings labelled. All of that is static, so it lives in
// the pre-rendered radarBg sprite (ui_sky_panel.cpp) and costs nothing per frame.
//
// The bezel is paid for out of the circle: the sprite can be at most the card
// interior's height (366px), so the ring costs SKY_BEZEL_PAD of radius. Both
// the sprite builder and drawSatPanel() derive their geometry from skyGeom() --
// they used to compute it separately, and a bezel makes that divergence
// visible rather than harmless.
static constexpr int SKY_BEZEL_PAD = 35;

struct SkyGeom { int radius, size, cx, cy; };

SkyGeom skyGeom(const Rect &skyInner);

void computeLayout();

// Set whenever relayout() repaints the whole canvas, so the next frame pushes
// full-screen instead of only the per-panel dirty rects.
// Starts true so the first frame paints every panel and pushes full-screen,
// rather than relying on initial signature values differing from zero.
extern bool relayoutPending;

// Re-lays out and repaints chrome after a touch toggles view/expand state.
void relayout();
