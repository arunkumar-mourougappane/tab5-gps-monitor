// Reads NMEA sentences from an M5Stack GPS/BDS Unit v1.1 (AT6668/ATGM336H)
// connected to Tab5's Port A (Grove), logs the decoded fix to USB serial, and
// renders a touch-interactive dashboard (position/trip card, satellite sky
// plot + signal list, raw NMEA log) on the Tab5's screen.
//
// Port A is a HY2.0-4P Grove connector: Black=GND, Red=5V, Yellow=G53, White=G54.
// By Grove/M5Stack convention Yellow carries the unit's RX line and White its TX
// line, so from the Tab5's side: G53 transmits to the GPS, G54 receives from it.
// This wasn't confirmed in M5Stack's Tab5 pinout doc -- swap GPS_RX_PIN/GPS_TX_PIN
// below if no sentences show up.
//
// TinyGPSPlus only parses GGA/RMC-family fields (location, altitude, speed,
// course, date/time, satellite count, HDOP). The satellite table (per-PRN
// elevation/azimuth/SNR) comes from GSV, and the 2D/3D fix mode + PDOP/VDOP
// come from GSA -- neither is exposed by TinyGPSPlus, so both are parsed here
// directly off the raw sentence stream.
//
// GPS ingestion runs on its own FreeRTOS task, pinned to the core opposite
// the Arduino loop task, so a slow display redraw can never stall draining
// the UART. The default RX ring buffer (256B) only absorbs ~22ms of silence
// at 115200 baud -- comfortably less than a full-panel redraw was taking in
// a single-loop design, which is the most likely cause of dropped/garbled
// sentences ("choppy" updates). Shared state is protected by a mutex; the
// render side only holds it long enough to copy a snapshot, then draws
// unlocked so the GPS task is never blocked by slow screen I/O.
//
// Touch: tap the position card to flip between the live readout and a trip
// summary (distance/max speed/HDOP trend); tap a satellite dot in the sky
// plot for its detail; tap the NMEA card to expand it full-height. Touch
// state is only ever read/written from loop() (the render side), so it
// needs no locking against gpsTask.

#include <M5Unified.h>
// Not reached by the normal M5GFX include chain; needed for the DCS brightness
// workaround in panelSetBrightness() below.
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>
#include <TinyGPSPlus.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "config.h"
#include "display.h"
#include "sd_logger.h"
#include "wifi_nmea.h"

#include "gps_task.h" // GPSSerial, gps, startGpsTask()
#include "render_snapshot.h" // RenderSnapshot, captureSnapshot(), stateMutex

// Definition matching the extern in display.h.
M5Canvas canvas(&M5.Display);

// Pre-rendered sky-plot background (rings + N/S/E/W labels): built once in
// setup() and blitted each tick instead of redrawing a filled circle, three
// ring outlines, two crosshair lines, and four text labels from scratch --
// that redraw was one of the most expensive things happening every 200ms.
M5Canvas radarBg(&canvas);
static int radarBgSize = 0;

#include "theme.h"
#include "layout.h"

#include "gps_model.h"
#include "trip_stats.h"
#include "nmea_parser.h"

// Fixed size so the hit target matches the drawn chip regardless of label, and
// so the chip can't resize under a changing label the way the status badges did.
static constexpr int FILTER_CHIP_W = 92;
static constexpr int FILTER_CHIP_H = 26;

static Rect nmeaFilterChipRect() {
  return {logCard.x + logCard.w - 18 - FILTER_CHIP_W, logCard.y + (CARD_HEADER_H - FILTER_CHIP_H) / 2,
          FILTER_CHIP_W, FILTER_CHIP_H};
}

// The chip is only 26px tall, well under a fingertip. Its target is padded to
// 44 and reaches slightly into the card body; that region belongs to the log
// card, which is why the filter is hit-tested before it -- a near miss used to
// expand the whole card instead, which is a worse outcome than doing nothing.
static Rect nmeaFilterHitRect() {
  Rect c = nmeaFilterChipRect();
  return {c.x - 16, c.y - 5, c.w + 24, c.h + 18};
}

// logSentence()/handleRawSentence() now live in nmea_parser.cpp.

// formatFix()/gpsTaskFn()/startGpsTask() now live in gps_task.cpp.

// RenderSnapshot / captureSnapshot() now live in render_snapshot.h/.cpp.

// ---------------------------------------------------------- touch/UI state --
// Only ever touched from loop() (render side) -- no locking needed.

enum class PositionView : uint8_t { LIVE, TRIP };
static PositionView positionView = PositionView::LIVE;
// logExpanded is defined in layout.cpp (computeLayout() needs it); declared
// via layout.h's extern.

struct SkyDot { int x, y; SatInfo info; };
static SkyDot skyDots[MAX_SATS];
static int skyDotCount = 0;

struct SatTooltip { bool active = false; SatInfo info; uint32_t shownAtMs = 0; };
static SatTooltip satTooltip;

// Screen-dark states. Both are woken by a tap anywhere; the difference is that
// sleep also powers the radio down and puts the panel itself to sleep rather
// than only zeroing the backlight.
//
// Neither stops GPS ingestion or SD logging: for a logger, "screen off" means
// keep recording in your pocket, not stop working.
static bool backlightOff = false;
static bool asleep = false;
static uint8_t savedBrightness = 128;

// Which chip the finger is currently down on, so it can render pressed. Held
// across frames (not just on release) so the highlight tracks the finger.
enum class PressTarget : uint8_t { NONE, LIGHT, SLEEP, FILTER, DIM_OFF, DIM_DONE };
static PressTarget pressTarget = PressTarget::NONE;

// Which chip the press *started* on. A button fires on release of the press
// that began inside it, rather than on a fresh hit test of the lift position:
// re-testing the lift meant a finger that drifted a few pixels off the chip
// lit the highlight and then did nothing at all, which reads as a button that
// only works sometimes.
static PressTarget capturedTarget = PressTarget::NONE;

// Touch sampling. Hoisted out of loop() so it can also run between the slices
// of a screen push -- a full-canvas blit is the longest thing the render side
// does, and while it runs nothing polls the panel. A quick tap that started
// and ended inside one of those windows was simply never seen: the driver
// only reads the hardware when we call it, so there is no event left to
// recover afterwards.
//
// getDetail(i) resolves through _touch_raw[i].id, and _touch_raw is only
// refreshed for the points the current scan reports. On release the driver
// reports zero points, so those entries hold stale data from an earlier scan
// -- and unlike getTouchPointRaw(), getDetail() does not clamp the index
// against the live point count. Reading a second slot that way is therefore
// not deterministic: the stale id can alias back to slot 0 or point at an
// unrelated detail slot.
//
// Instead: sample coordinates only while a contact is definitely down (raw
// data valid then), and detect the lift ourselves as the absence of a press.
// That is unambiguous, fires exactly once per physical touch, and is immune
// both to the ghost second contact this panel emits and to the state machine
// escalating a tap to hold/flick/drag.
static bool touchWasDown = false;
static bool touchPressing = false;
static bool touchPressEdge = false;   // consumed by loop()
static bool touchReleaseEdge = false; // consumed by loop()
static int touchLastX = -1, touchLastY = -1; // current contact, for drags
static int touchDownX = -1, touchDownY = -1; // where the press began, for hit tests

static void sampleTouch() {
  M5.update();
  auto touch = M5.Touch.getDetail(0);
  if (touch.isPressed()) {
    if (!touchWasDown) {
      touchPressEdge = true;
      touchDownX = touch.x;
      touchDownY = touch.y;
    }
    touchWasDown = true;
    touchPressing = true;
    touchLastX = touch.x;
    touchLastY = touch.y;
  } else {
    touchPressing = false;
    if (touchWasDown) {
      touchWasDown = false;
      touchReleaseEdge = true;
    }
  }
}

// Brightness overlay, opened from the LIGHT chip.
static bool dimmerOpen = false;
static bool dimmerDirty = false;
static bool dimmerDragging = false; // slider grabbed, follows X until the lift
static constexpr uint8_t BRIGHTNESS_MIN = 8; // slider floor: never fully dark
static constexpr uint8_t BRIGHTNESS_MAX = 255;

// Finger-sized hit target. The dots are drawn at only 4-9px radius, but a
// fingertip covers ~40px on this panel -- matching the visual size made them
// effectively untappable, and a miss silently does nothing because the sky
// area isn't covered by any other tap target.
//
// Picks the *nearest* dot rather than the first in array order that happens
// to qualify: satellites cluster near the zenith, and since the tooltip
// reports one specific satellite's details, grabbing an arbitrary neighbour
// would show data for a satellite the user didn't aim at.
static constexpr int SKY_DOT_HIT_RADIUS = 34;

static int hitTestSkyDot(int px, int py) {
  int best = -1;
  int bestDistSq = SKY_DOT_HIT_RADIUS * SKY_DOT_HIT_RADIUS;
  for (int i = 0; i < skyDotCount; i++) {
    int dx = px - skyDots[i].x, dy = py - skyDots[i].y;
    int distSq = dx * dx + dy * dy;
    if (distSq <= bestDistSq) {
      bestDistSq = distSq;
      best = i;
    }
  }
  return best;
}

// ------------------------------------------------------------- rendering --

// Draws label above value, auto-shrinking the value if it would overflow `w`.
// Returns the y position just below the drawn value.
static int drawHeroValue(int x, int y, int w, const char *label, const char *value, uint8_t bigSize = 2) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(label);
  y += d.fontHeight() + 4;

  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setTextSize(bigSize);
  if (d.textWidth(value) > w) d.setTextSize(1);
  d.setCursor(x, y);
  d.print(value);
  y += d.fontHeight() + 2;
  return y;
}

static void drawMiniStat(int x, int y, const char *label, const char *value) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(label);
  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(x, y + 20);
  d.print(value);
}

// Rounded pill with a leading text (no dot) -- used for fix/sat count badges.
static int drawBadge(int x, int y, const char *text, uint16_t bg, uint16_t fg) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  int h = 30;
  int w = d.textWidth(text) + 28;
  d.fillRoundRect(x, y, w, h, h / 2, bg);
  d.setTextColor(fg, bg);
  d.setCursor(x + 14, y + (h - d.fontHeight()) / 2);
  d.print(text);
  return w;
}

static void drawCardFrame(const Rect &r, const char *label, uint16_t accent) {
  auto &d = canvas;
  d.fillRoundRect(r.x, r.y, r.w, r.h, CARD_RADIUS, COLOR_CARD_BG);
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(r.x + 18, r.y + 11);
  d.print(label);
  d.drawFastHLine(r.x + 18, r.y + CARD_HEADER_H, r.w - 36, accent);
}

// Header text doubles as the touch-affordance hint, and only needs to be
// repainted when a view toggles -- so drawStaticChrome() is called again on
// every touch transition rather than every render tick.
void drawFilterChip(); // defined below; called from layout.cpp's relayout()

void drawStaticChrome() { // called from layout.cpp's relayout()
  auto &d = canvas;
  d.fillScreen(COLOR_BG);

  d.fillRect(0, 0, SCREEN_W, TOPBAR_H, COLOR_TOPBAR_BG);
  d.drawFastHLine(0, TOPBAR_H, SCREEN_W, COLOR_DIVIDER);
  d.setFont(&fonts::Font4);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_TOPBAR_BG);
  d.setCursor(MARGIN, (TOPBAR_H - d.fontHeight()) / 2);
  d.print("TAB5 GPS MONITOR");

  if (!logExpanded) {
    drawCardFrame(fixCard, positionView == PositionView::LIVE ? "POSITION  (tap: trip stats)" : "TRIP STATS  (tap: live)",
                  COLOR_ACCENT);
    drawCardFrame(skyCard, "SATELLITES  (tap a dot for detail)", COLOR_ACCENT);
  }
  drawCardFrame(logCard, logExpanded ? "NMEA STREAM  (tap to collapse)" : "NMEA STREAM  (tap to expand)",
                COLOR_ACCENT_GREEN);

  drawFilterChip(); // sits in the log card header, at its right end
}

// Shared top-bar badge geometry. Every pill is laid out as
//   [PAD_L][icon][ICON_GAP][text][PAD_R]
// and its width is always derived from the string that is actually drawn.
// Keeping these in one place is deliberate: the battery badge previously
// sized itself from "100%" while printing "100% CHG", so the label overran
// the pill and collided with the badge to its right.
//
// Sized against two hard limits. Vertically the bar is TOPBAR_H (56px), so a
// badge cannot exceed that minus breathing room at each edge; 36 leaves 10px
// above and below. Horizontally the row is right-aligned and chains leftward,
// and drawStatusPill() repaints the chips before the badges -- so a row long
// enough to reach the SLEEP chip's right edge (518) would paint over it.
//
// This is a deliberate midpoint: the pills were 32px and briefly 42px, which
// read as oversized on the panel.
static constexpr int BADGE_H = 36;
static constexpr int BADGE_PAD_L = 16;
static constexpr int BADGE_PAD_R = 16;
static constexpr int BADGE_ICON_GAP = 11;
static constexpr int BADGE_GAP = 11; // spacing between adjacent badges
static constexpr int BADGE_DOT_R = 6;
// The classic font ramp has no size between Font2 (16px) and Font4 (26px) --
// Font0 is 8px and the next step up is 26 -- so the badge text stays at Font2
// while the pill around it grew. Getting an intermediate size means bringing
// in a GFX face (FreeSans9pt7b, DejaVu18), which would put a second typeface
// in a bar that is otherwise all Font2/Font4.
//
// Single knob for the whole top bar: the badges and the LIGHT/SLEEP chips
// take their face from here. Everything else drawn through drawChip() keeps
// its own default -- the NMEA filter chip is only 26px tall.
#define BADGE_FONT (&fonts::Font2)

// Action chips live on the LEFT of the top bar, at fixed positions clear of the
// title. Deliberately not chained onto the right-hand badge row: those pills
// resize with their labels, which would shift these buttons' hit rects around
// under the user's finger.
static constexpr int TOPBAR_CTRL_X = 300;
static constexpr int TOPBAR_CTRL_W = 104;

static Rect lightBtnRect() {
  return {TOPBAR_CTRL_X, (TOPBAR_H - BADGE_H) / 2, TOPBAR_CTRL_W, BADGE_H};
}

static Rect sleepBtnRect() {
  return {TOPBAR_CTRL_X + TOPBAR_CTRL_W + BADGE_GAP, (TOPBAR_H - BADGE_H) / 2, TOPBAR_CTRL_W, BADGE_H};
}

// Touch targets, deliberately larger than the chips drawn inside them. A
// fingertip covers roughly 40px on this panel and these chips are 36 tall, so
// aiming at one and landing a few pixels high or low was a miss. Nothing else
// in the top bar is tappable, so both claim its full height; horizontally they
// take 5px of the 11px gap each, which keeps them from overlapping each other.
static Rect lightHitRect() {
  Rect c = lightBtnRect();
  return {c.x - 5, 0, c.w + 10, TOPBAR_H};
}

static Rect sleepHitRect() {
  Rect c = sleepBtnRect();
  return {c.x - 5, 0, c.w + 10, TOPBAR_H};
}

// Press feedback is a colour swap rather than an inset/shrink: an inset would
// leave a ring of the previous fill behind unless the surrounding background
// were also repainted, and that background differs per call site.
static void drawChip(const Rect &c, const char *label, uint16_t bg, uint16_t fg, bool pressed,
                     const lgfx::IFont *font = &fonts::Font2) {
  auto &d = canvas;
  uint16_t fill = pressed ? COLOR_ACCENT : bg;
  uint16_t text = pressed ? COLOR_BG : fg;
  d.fillRoundRect(c.x, c.y, c.w, c.h, c.h / 2, fill);
  d.setFont(font);
  d.setTextSize(1);
  d.setTextColor(text, fill);
  d.setCursor(c.x + (c.w - d.textWidth(label)) / 2, c.y + (c.h - d.fontHeight()) / 2);
  d.print(label);
}

// ------------------------------------------------ panel brightness via DCS --
// M5.Display.setBrightness() is a no-op on Tab5: Panel_Device::setBrightness()
// forwards to a Light instance and Tab5's init path never attaches one, while
// Panel_DSI/ST7123/ST7121 implement no brightness of their own and neither IO
// expander carries a backlight pin. The one remaining route is the panel's own
// DSI command channel: DCS 0x51 (set_display_brightness), which only takes
// effect once 0x53 (write_control_display) has enabled the brightness block.
//
// write_params() is protected, so it's reached through a derived type that adds
// no members and therefore shares the base layout -- the same reinterpret_cast
// idiom M5GFX itself uses in getPanel(). The type is never instantiated.
//
// SPECULATIVE: this only works if the ST7121/ST7123 actually drives the
// backlight. If the panel ignores 0x51 the level won't change, and only the
// blank-to-black path will be visible. Verify on hardware before relying on it.
struct DsiBrightnessAccess : public lgfx::Panel_DSI {
  bool writeDcs(uint32_t cmd, const uint8_t *data, size_t len) { return write_params(cmd, data, len); }
};

static bool panelSetBrightness(uint8_t level) {
  auto *p = M5.Display.getPanel();
  if (p == nullptr) return false;
  auto *acc = reinterpret_cast<DsiBrightnessAccess *>(p);
  uint8_t ctrl = 0x2C; // BCTRL | DD | BL -- enable the brightness control block
  acc->writeDcs(0x53, &ctrl, 1);
  return acc->writeDcs(0x51, &level, 1);
}

// ---------------------------------------------------------- dimmer overlay --

static Rect dimmerPanelRect() {
  constexpr int w = 560, h = 210;
  return {(SCREEN_W - w) / 2, (SCREEN_H - h) / 2, w, h};
}

static Rect dimmerTrackRect() {
  Rect p = dimmerPanelRect();
  return {p.x + 30, p.y + 82, p.w - 60, 26};
}

static Rect dimmerOffRect() {
  Rect p = dimmerPanelRect();
  return {p.x + 30, p.y + p.h - 62, 150, 44};
}

static Rect dimmerDoneRect() {
  Rect p = dimmerPanelRect();
  return {p.x + p.w - 30 - 150, p.y + p.h - 62, 150, 44};
}

// Grab area for the slider. The track is drawn 26px tall but the knob riding
// on it is 34, and a horizontal drag never stays inside a 26px band -- so the
// area that can start and hold a drag is padded well past both. It stops short
// of OFF/DONE at y+148 and of the percentage readout above.
static Rect dimmerTrackHitRect() {
  Rect t = dimmerTrackRect();
  return {t.x - 10, t.y - 18, t.w + 20, t.h + 36};
}

// True when the level actually moved, so a drag that hasn't crossed into the
// next step doesn't repaint the overlay or re-issue the panel command.
static bool setBrightnessFromTouch(int tx) {
  Rect t = dimmerTrackRect();
  int rel = constrain(tx - t.x, 0, t.w);
  uint8_t level = (uint8_t)(BRIGHTNESS_MIN + (long)(BRIGHTNESS_MAX - BRIGHTNESS_MIN) * rel / t.w);
  if (level == savedBrightness) return false;
  savedBrightness = level;
  M5.Display.setBrightness(savedBrightness); // no-op today; harmless if M5GFX gains a Light
  panelSetBrightness(savedBrightness);
  return true;
}

static void drawDimmer() {
  auto &d = canvas;
  Rect p = dimmerPanelRect();

  d.fillRoundRect(p.x, p.y, p.w, p.h, CARD_RADIUS, COLOR_CARD_BG);
  d.drawRoundRect(p.x, p.y, p.w, p.h, CARD_RADIUS, COLOR_ACCENT);

  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(p.x + 30, p.y + 22);
  d.print("BRIGHTNESS");

  int pct = (savedBrightness - BRIGHTNESS_MIN) * 100 / (BRIGHTNESS_MAX - BRIGHTNESS_MIN);
  char pctBuf[8];
  snprintf(pctBuf, sizeof(pctBuf), "%d%%", pct);
  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(p.x + p.w - 30 - d.textWidth(pctBuf), p.y + 16);
  d.print(pctBuf);

  Rect t = dimmerTrackRect();
  d.fillRoundRect(t.x, t.y, t.w, t.h, t.h / 2, COLOR_BG);
  int fillW = max(t.h, t.w * pct / 100);
  d.fillRoundRect(t.x, t.y, fillW, t.h, t.h / 2, COLOR_ACCENT);
  // Knob, clamped so it stays fully inside the track at both extremes.
  int knobX = constrain(t.x + fillW, t.x + t.h / 2, t.x + t.w - t.h / 2);
  d.fillCircle(knobX, t.y + t.h / 2, t.h / 2 + 4, COLOR_TEXT_PRIMARY);

  drawChip(dimmerOffRect(), "OFF", COLOR_TOPBAR_BG, COLOR_TEXT_PRIMARY, pressTarget == PressTarget::DIM_OFF);
  drawChip(dimmerDoneRect(), "DONE", COLOR_TOPBAR_BG, COLOR_TEXT_PRIMARY, pressTarget == PressTarget::DIM_DONE);
}

// Drawn from drawLogPanel() (per frame, so it can animate) as well as from
// drawStaticChrome() (so it survives a relayout). Its fixed size means the
// self-contained fill fully covers the previous state -- no clear needed.
void drawFilterChip() {
  bool filtering = (NmeaFilter)nmeaFilter != NmeaFilter::ALL;
  drawChip(nmeaFilterChipRect(), nmeaFilterLabel(), filtering ? COLOR_ACCENT_GREEN : COLOR_TOPBAR_BG,
           filtering ? COLOR_BG : COLOR_TEXT_SECONDARY, pressTarget == PressTarget::FILTER);
}

static PressTarget hitTestChips(int x, int y) {
  if (dimmerOpen) {
    if (pointInRect(x, y, dimmerOffRect())) return PressTarget::DIM_OFF;
    if (pointInRect(x, y, dimmerDoneRect())) return PressTarget::DIM_DONE;
    return PressTarget::NONE;
  }
  if (pointInRect(x, y, lightHitRect())) return PressTarget::LIGHT;
  if (pointInRect(x, y, sleepHitRect())) return PressTarget::SLEEP;
  if (pointInRect(x, y, nmeaFilterHitRect())) return PressTarget::FILTER;
  return PressTarget::NONE;
}

// Right-aligned "dot + label" pill, returning its left edge so callers can
// chain the next badge leftward from it.
static int drawDotBadge(int rightEdgeX, int py, const char *text, uint16_t dotColor) {
  auto &d = canvas;
  d.setFont(BADGE_FONT);
  d.setTextSize(1);

  int iconW = BADGE_DOT_R * 2;
  int textX = BADGE_PAD_L + iconW + BADGE_ICON_GAP;
  int pillW = textX + d.textWidth(text) + BADGE_PAD_R;
  int px = rightEdgeX - pillW;

  d.fillRoundRect(px, py, pillW, BADGE_H, BADGE_H / 2, COLOR_CARD_BG);
  d.fillCircle(px + BADGE_PAD_L + BADGE_DOT_R, py + BADGE_H / 2, BADGE_DOT_R, dotColor);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(px + textX, py + (BADGE_H - d.fontHeight()) / 2);
  d.print(text);
  return px;
}

// M5.Power reads go over I2C to the PMIC. Both the status-bar signature and
// the badge itself want them, and the signature is sampled every 200ms -- so
// an uncached pair costs ten I2C round-trips a second for a value that moves
// on the order of minutes. Refreshed at 1Hz and shared by both callers.
static int32_t battLevel = -1; // 0-100, negative if unknown
static bool battCharging = false;
static uint32_t battReadMs = 0;

static void refreshBattery() {
  uint32_t now = millis();
  if (battReadMs != 0 && now - battReadMs < 1000) return;
  battReadMs = now;
  battLevel = M5.Power.getBatteryLevel();
  battCharging = M5.Power.isCharging() == m5::Power_Class::is_charging_t::is_charging;
}

static int drawBatteryBadge(int rightEdgeX, int py) {
  auto &d = canvas;
  refreshBattery();
  int32_t level = battLevel;
  bool charging = battCharging;

  char text[16];
  if (level >= 0) snprintf(text, sizeof(text), "%d%%%s", (int)level, charging ? " CHG" : "");
  else snprintf(text, sizeof(text), "--%s", charging ? " CHG" : "");

  d.setFont(BADGE_FONT);
  d.setTextSize(1);

  // Icon scaled with the pill.
  constexpr int bw = 26, bh = 16, nubW = 4;
  int iconW = bw + nubW;
  int textX = BADGE_PAD_L + iconW + BADGE_ICON_GAP;
  int pillW = textX + d.textWidth(text) + BADGE_PAD_R; // measured on the drawn string
  int px = rightEdgeX - pillW;

  uint16_t barColor = level < 0 ? COLOR_STATUS_NONE : level < 20 ? COLOR_STATUS_BAD : level < 40 ? COLOR_STATUS_WARN : COLOR_STATUS_GOOD;

  d.fillRoundRect(px, py, pillW, BADGE_H, BADGE_H / 2, COLOR_CARD_BG);

  int bx = px + BADGE_PAD_L, by = py + BADGE_H / 2 - bh / 2;
  d.drawRoundRect(bx, by, bw, bh, 3, COLOR_TEXT_SECONDARY);
  d.fillRect(bx + bw, by + 4, nubW, bh - 8, COLOR_TEXT_SECONDARY); // terminal nub
  if (level >= 0) {
    int fillW = max(2, (bw - 4) * constrain((int)level, 0, 100) / 100);
    d.fillRoundRect(bx + 2, by + 2, fillW, bh - 4, 2, barColor);
  }

  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(px + textX, py + (BADGE_H - d.fontHeight()) / 2);
  d.print(text);
  return px;
}

static void drawStatusPill(uint32_t snapLastSentenceMs) {
  auto &d = canvas;

  // Every badge here is right-aligned and chains leftward off the one before
  // it, and each pill's width comes from its own label -- "100%" vs
  // "100% CHG", "SD" vs "NO SD", "RECEIVING" vs "NO DATA", "100%" vs "99%".
  // When any label changes width the whole row shifts, but each badge only
  // repaints its own current footprint, leaving the previous pill's rounded
  // cap and trailing glyphs behind. Unlike the cards, the top-bar background
  // is painted only once (drawStaticChrome at setup), so nothing ever cleans
  // those up. Wipe the strip first. The badge row is far narrower than half
  // the screen, so clearing the right half stays well clear of the title.
  d.fillRect(TOPBAR_CTRL_X, 0, SCREEN_W - TOPBAR_CTRL_X, TOPBAR_H, COLOR_TOPBAR_BG);

  drawChip(lightBtnRect(), "LIGHT", COLOR_CARD_BG, COLOR_TEXT_SECONDARY, pressTarget == PressTarget::LIGHT,
           BADGE_FONT);
  drawChip(sleepBtnRect(), "SLEEP", COLOR_CARD_BG, COLOR_TEXT_SECONDARY, pressTarget == PressTarget::SLEEP,
           BADGE_FONT);

  bool haveData = snapLastSentenceMs != 0 && millis() - snapLastSentenceMs < 5000;
  int py = (TOPBAR_H - BADGE_H) / 2;

  // Right-aligned, each badge chaining leftward off the previous one's left edge.
  int px = drawDotBadge(SCREEN_W - MARGIN, py, haveData ? "RECEIVING" : "NO DATA",
                        haveData ? COLOR_STATUS_GOOD : COLOR_STATUS_BAD);
  int bpx = drawBatteryBadge(px - BADGE_GAP, py);
#if ENABLE_WIFI_NMEA
  char apBuf[12];
  snprintf(apBuf, sizeof(apBuf), "AP %d", nmeaClientCount);
  bpx = drawDotBadge(bpx - BADGE_GAP, py, apBuf,
                     nmeaClientCount > 0 ? COLOR_STATUS_GOOD : COLOR_TEXT_SECONDARY);
#endif
  drawDotBadge(bpx - BADGE_GAP, py, sdReady ? "SD" : "NO SD",
               sdReady ? COLOR_STATUS_GOOD : COLOR_STATUS_NONE);
}

static void drawSparkline(int x, int y, int w, int h, const float *values, int count, uint32_t head,
                          uint16_t color, float maxVal); // defined below, with the trip view

// Medium-weight value+quality-badge / value+caption block, used for the
// accuracy and time readouts -- a step down from the lat/lon hero values but
// well above a plain caption line.
static int drawAccuracyBlock(int x, int y, int w, const RenderSnapshot &s) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print("ACCURACY (HDOP)");
  y += d.fontHeight() + 4;

  char val[12];
  if (s.hdopValid) snprintf(val, sizeof(val), "%.1f", s.hdop);
  else snprintf(val, sizeof(val), "--");
  uint16_t qColor = COLOR_STATUS_NONE;
  const char *qLabel = s.hdopValid ? hdopQuality(s.hdop, qColor) : "NO FIX";

  d.setFont(&fonts::Font4);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(val);
  int valW = d.textWidth(val);
  int heroH = d.fontHeight();

  int bh = 24;
  d.setFont(&fonts::Font2);
  int bw = d.textWidth(qLabel) + 20;
  int bx = x + valW + 14;
  int by = y + (heroH - bh) / 2;
  d.fillRoundRect(bx, by, bw, bh, bh / 2, qColor);
  d.setTextColor(COLOR_BG, qColor);
  d.setCursor(bx + 10, by + (bh - d.fontHeight()) / 2);
  d.print(qLabel);

  // The HDOP trend sits directly under its own value, in the space the
  // PDOP/VDOP caption used to hold. A single number says nothing about whether
  // the fix is settling or degrading, which is the question the accuracy block
  // exists to answer; PDOP/VDOP are still in the stream and on the SD log for
  // anyone who wants the full trio.
  //
  // 18px tall so the block's baseline still lines up with the date line the
  // time block ends on, and the same fixed 0-5 scale the trip view uses -- a
  // trace that autoscaled would make a steady poor fix look identical to a
  // steady good one.
  y += heroH + 4;
  constexpr int TREND_H = 18;
  drawSparkline(x, y, w, TREND_H, s.hdopHistory, s.hdopHistCount, s.hdopHistHead, COLOR_ACCENT, 5.0f);
  y += TREND_H + 2;
  return y;
}

static int drawTimeBlock(int x, int y, int w, const RenderSnapshot &s) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print("TIME (UTC)");
  y += d.fontHeight() + 4;

  char timeBuf[10], dateBuf[16];
  if (s.timeValid) snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d", s.hour, s.minute, s.second);
  else snprintf(timeBuf, sizeof(timeBuf), "--:--:--");
  if (s.dateValid) snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d", s.year, s.month, s.day);
  else snprintf(dateBuf, sizeof(dateBuf), "----------");

  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setTextSize(1);
  d.setCursor(x, y);
  d.print(timeBuf);
  y += d.fontHeight() + 4;

  d.setFont(&fonts::Font2);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(dateBuf);
  y += d.fontHeight() + 2;
  return y;
}

// Ring-buffer line chart. `count`/`head` follow the same convention as the
// raw-log ring buffer: `head` is the next write index, the most recent
// sample is at (head-1).
static void drawSparkline(int x, int y, int w, int h, const float *values, int count, uint32_t head,
                           uint16_t color, float maxVal) {
  auto &d = canvas;
  d.fillRect(x, y, w, h, COLOR_BG);
  if (count < 2) return;

  int prevX = 0, prevY = 0;
  for (int i = 0; i < count; i++) {
    int idx = (int)((head - count + i) % SPARK_LEN);
    if (idx < 0) idx += SPARK_LEN;
    float norm = maxVal > 0 ? constrain(values[idx] / maxVal, 0.0f, 1.0f) : 0;
    int px = x + (int)((float)i / (count - 1) * w);
    int py = y + h - (int)(norm * h);
    if (i > 0) d.drawLine(prevX, prevY, px, py, color);
    prevX = px;
    prevY = py;
  }
}

static int drawTripView(int x, int y, int w, const RenderSnapshot &s) {
  auto &d = canvas;
  int colW = w / 2;

  uint32_t elapsedMs = s.firstFixAbsMs ? (millis() - s.firstFixAbsMs) : 0;
  double avgSpeed = elapsedMs > 2000 ? s.tripDistanceKm / (elapsedMs / 3600000.0) : 0;

  char distBuf[20], maxBuf[20], avgBuf[20];
  snprintf(distBuf, sizeof(distBuf), "%.2f km", s.tripDistanceKm);
  snprintf(maxBuf, sizeof(maxBuf), "%.1f km/h", s.maxSpeedKmph);
  snprintf(avgBuf, sizeof(avgBuf), "%.1f km/h", avgSpeed);

  int y1 = drawHeroValue(x, y, colW - 10, "DISTANCE", distBuf, 1);
  int y2 = drawHeroValue(x + colW, y, colW - 10, "MAX SPEED", maxBuf, 1);
  y = max(y1, y2) + 10;

  char tripBuf[12];
  uint32_t secs = elapsedMs / 1000;
  snprintf(tripBuf, sizeof(tripBuf), "%02u:%02u:%02u", (unsigned)(secs / 3600), (unsigned)((secs / 60) % 60),
           (unsigned)(secs % 60));
  int y3 = drawHeroValue(x, y, colW - 10, "TRIP TIME", tripBuf, 1);
  int y4 = drawHeroValue(x + colW, y, colW - 10, "AVG SPEED", avgBuf, 1);
  y = max(y3, y4) + 8;

  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  char ttffBuf[36];
  if (s.timeToFirstFixMs > 0) snprintf(ttffBuf, sizeof(ttffBuf), "TIME TO FIRST FIX: %.1fs", s.timeToFirstFixMs / 1000.0f);
  else snprintf(ttffBuf, sizeof(ttffBuf), "TIME TO FIRST FIX: --");
  d.setCursor(x, y);
  d.print(ttffBuf);
  y += d.fontHeight() + 10;

  // Speed over the session. Autoscaled to the trip's own maximum rather than a
  // fixed ceiling -- unlike HDOP, there is no absolute scale that suits both a
  // walk and a motorway, and MAX SPEED is printed directly above, so the top of
  // the trace is already labelled.
  float speedScale = max(10.0f, (float)s.maxSpeedKmph);
  d.setCursor(x, y);
  d.print("SPEED");
  y += d.fontHeight() + 4;
  int sparkH = 36;
  drawSparkline(x, y, w, sparkH, s.speedHistory, s.speedHistCount, s.speedHistHead, COLOR_ACCENT, speedScale);
  y += sparkH + 14;

  // How much of the trip actually had a usable fix. For a logger this is the
  // question the recording is only as good as, and nothing on the live view
  // can answer it -- that shows the fix you have now, not the one you had for
  // the last twenty minutes.
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print("FIX QUALITY");
  y += d.fontHeight() + 4;

  uint32_t total = s.fixSecs[0] + s.fixSecs[1] + s.fixSecs[2];
  int barH = 16;
  d.fillRoundRect(x, y, w, barH, barH / 2, COLOR_BG);
  if (total > 0) {
    // Painted worst-first so the eye lands on the good segment's length.
    const uint16_t segColor[3] = {COLOR_STATUS_BAD, COLOR_STATUS_WARN, COLOR_STATUS_GOOD};
    int segX = x;
    for (int i = 0; i < 3; i++) {
      int segW = (int)((uint64_t)w * s.fixSecs[i] / total);
      if (segW > 0) d.fillRect(segX, y, segW, barH, segColor[i]);
      segX += segW;
    }
  }
  y += barH + 8;

  // Legend doubles as the readout: colour plus the actual time at each mode.
  const char *segLabel[3] = {"NONE", "2D", "3D"};
  const uint16_t segColor[3] = {COLOR_STATUS_BAD, COLOR_STATUS_WARN, COLOR_STATUS_GOOD};
  int colStep = w / 3;
  for (int i = 0; i < 3; i++) {
    int lx = x + colStep * i;
    d.fillRect(lx, y + 4, 10, 10, segColor[i]);
    char buf[24];
    uint32_t secs = s.fixSecs[i];
    snprintf(buf, sizeof(buf), "%s %u:%02u", segLabel[i], (unsigned)(secs / 60), (unsigned)(secs % 60));
    d.setTextColor(secs > 0 ? COLOR_TEXT_PRIMARY : COLOR_STATUS_NONE, COLOR_CARD_BG);
    d.setCursor(lx + 16, y);
    d.print(buf);
  }
  y += d.fontHeight() + 2;
  return y;
}

static void drawFixPanel(const RenderSnapshot &s) {
  auto &d = canvas;
  Rect r = innerOf(fixCard);
  d.fillRect(r.x, r.y, r.w, r.h, COLOR_CARD_BG);

  int y = r.y;
  const char *fixStr = s.fixType == 3 ? "3D FIX" : s.fixType == 2 ? "2D FIX" : "NO FIX";
  uint16_t fixColor = s.fixType == 3 ? COLOR_STATUS_GOOD : s.fixType == 2 ? COLOR_STATUS_WARN : COLOR_STATUS_BAD;
  drawBadge(r.x, y, fixStr, fixColor, COLOR_BG);

  char satsBuf[24];
  snprintf(satsBuf, sizeof(satsBuf), "%d/%d SATS", s.satsValid ? s.satsUsed : 0, s.visibleSats);
  d.setFont(&fonts::Font2);
  int satsW = d.textWidth(satsBuf) + 28;
  drawBadge(r.x + r.w - satsW, y, satsBuf, COLOR_TOPBAR_BG, COLOR_TEXT_PRIMARY);
  y += 30 + 22;

  if (positionView == PositionView::TRIP) {
    drawTripView(r.x, y, r.w, s);
    return;
  }

  char latBuf[16], lonBuf[16], altBuf[16], spdBuf[16], crsBuf[16];
  if (s.locValid) {
    snprintf(latBuf, sizeof(latBuf), "%.6f", s.lat);
    snprintf(lonBuf, sizeof(lonBuf), "%.6f", s.lon);
  } else {
    strlcpy(latBuf, "---", sizeof(latBuf));
    strlcpy(lonBuf, "---", sizeof(lonBuf));
  }
  if (s.altValid) snprintf(altBuf, sizeof(altBuf), "%.0fm", s.alt);
  else strlcpy(altBuf, "---", sizeof(altBuf));
  if (s.spdValid) snprintf(spdBuf, sizeof(spdBuf), "%.1fkm/h", s.spd);
  else strlcpy(spdBuf, "---", sizeof(spdBuf));
  if (s.crsValid) snprintf(crsBuf, sizeof(crsBuf), "%.0f%c", s.crs, (char)0xB0);
  else strlcpy(crsBuf, "---", sizeof(crsBuf));

  y = drawHeroValue(r.x, y, r.w, "LATITUDE", latBuf);
  y += 12;
  y = drawHeroValue(r.x, y, r.w, "LONGITUDE", lonBuf);
  y += 20;

  int colW = r.w / 3;
  drawMiniStat(r.x, y, "ALT", altBuf);
  drawMiniStat(r.x + colW, y, "SPEED", spdBuf);
  drawMiniStat(r.x + colW * 2, y, "CRS", crsBuf);
  y += 58;

  int colW2 = r.w / 2;
  int yAcc = drawAccuracyBlock(r.x, y, colW2 - 10, s);
  int yTime = drawTimeBlock(r.x + colW2, y, colW2 - 10, s);
  (void)max(yAcc, yTime);
}

// Polar sky plot: center = zenith (elevation 90), edge = horizon (elevation
// 0). Also draws a heading needle (course-over-ground) when moving, and
// records each dot's screen position/data into skyDots[] for touch hit-testing.
static void drawSkyPlot(const SatInfo arr[MAX_SATS], int cx, int cy, int radius, bool showNeedle, float courseDeg) {
  auto &d = canvas;
  // Blit the pre-rendered rings/labels instead of redrawing them -- this
  // also resets the area to a clean background, erasing last tick's dots.
  radarBg.pushSprite(&canvas, cx - radarBgSize / 2, cy - radarBgSize / 2);

  // Course rides the bezel as a chevron instead of a needle through the middle.
  // A needle looks right on an aircraft HSI, where it points across empty
  // gauge -- here it lies on top of the readout. Measured against a simulated
  // sky, a solid arrow covers about 1.5 of 14 satellites at any moment and a
  // compass-style needle about 2.6; the chevron covers none of them, because
  // nothing enters the plot at all.
  if (showNeedle) {
    float a = radians(courseDeg);
    float aL = radians(courseDeg - 5.0f), aR = radians(courseDeg + 5.0f);
    int tipX = cx + (int)((radius + 20) * sinf(a));
    int tipY = cy - (int)((radius + 20) * cosf(a));
    int lx = cx + (int)((radius + 3) * sinf(aL));
    int ly = cy - (int)((radius + 3) * cosf(aL));
    int rx = cx + (int)((radius + 3) * sinf(aR));
    int ry = cy - (int)((radius + 3) * cosf(aR));
    d.fillTriangle(tipX, tipY, lx, ly, rx, ry, COLOR_ACCENT);
    // A short arc anchors the chevron to the rim so it still reads as a bearing
    // when it happens to sit between two ticks. LGFX arc angles start at +X and
    // run clockwise, hence the -90.
    d.fillArc(cx, cy, radius + 1, radius + 4, courseDeg - 96.0f, courseDeg - 84.0f, COLOR_ACCENT);
  } else {
    // Below ~1km/h course-over-ground is noise, so there is no bearing to draw.
    // Say so, rather than removing the marker and leaving the instrument
    // looking broken.
    d.setFont(&fonts::Font2);
    d.setTextSize(1);
    d.setTextColor(COLOR_STATUS_NONE, COLOR_BG);
    const char *msg = "NO COURSE";
    d.setCursor(cx - d.textWidth(msg) / 2, cy + (int)(radius * 0.72f));
    d.print(msg);
  }

  skyDotCount = 0;
  for (int i = 0; i < MAX_SATS; i++) {
    const SatInfo &s = arr[i];
    if (!s.used || s.elevation < 0 || s.azimuth < 0) continue;
    float az = radians((float)s.azimuth);
    float rr = radius * (90 - s.elevation) / 90.0f;
    int x = cx + (int)(rr * sinf(az));
    int y = cy - (int)(rr * cosf(az));
    int dotR = s.snr < 0 ? 4 : map(constrain(s.snr, 0, 55), 0, 55, 4, 9);
    d.fillCircle(x, y, dotR, snrColor(s.snr));
    if (skyDotCount < MAX_SATS) skyDots[skyDotCount++] = {x, y, s};
  }
}

static void drawSatList(const SatInfo arr[MAX_SATS], int x, int y, int w, int h) {
  auto &d = canvas;
  d.fillRect(x, y, w, h, COLOR_CARD_BG);
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print("SIGNAL");
  int rowY = y + d.fontHeight() + 10;

  int order[MAX_SATS];
  int countUsed = sortedSatOrder(arr, order);

  int rowH = 24;
  int maxRows = (y + h - rowY) / rowH;
  int labelW = 56;
  int barX = x + labelW;
  int barW = w - labelW - 34;
  if (barW < 10) barW = 10;

  for (int k = 0; k < countUsed && k < maxRows; k++) {
    const SatInfo &s = arr[order[k]];
    char buf[12];
    snprintf(buf, sizeof(buf), "%s%d", s.talker, s.prn);
    d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
    d.setCursor(x, rowY);
    d.print(buf);

    int bw = s.snr < 0 ? 2 : map(constrain(s.snr, 0, 55), 0, 55, 2, barW);
    d.fillRoundRect(barX, rowY + 2, barW, 12, 6, COLOR_BG);
    d.fillRoundRect(barX, rowY + 2, bw, 12, 6, snrColor(s.snr));

    char snrBuf[8];
    if (s.snr >= 0) snprintf(snrBuf, sizeof(snrBuf), "%d", s.snr);
    else snprintf(snrBuf, sizeof(snrBuf), "-");
    d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
    d.setCursor(barX + barW + 6, rowY);
    d.print(snrBuf);

    rowY += rowH;
  }
}

// Small floating detail card for whichever satellite dot was last tapped;
// self-expires 4s after the tap.
static void drawSatTooltip(const Rect &r) {
  if (!satTooltip.active) return;
  if (millis() - satTooltip.shownAtMs > 4000) {
    satTooltip.active = false;
    return;
  }

  auto &d = canvas;
  const SatInfo &s = satTooltip.info;
  char snrBuf[8];
  if (s.snr >= 0) snprintf(snrBuf, sizeof(snrBuf), "%d", s.snr);
  else strlcpy(snrBuf, "-", sizeof(snrBuf));
  char buf[48];
  snprintf(buf, sizeof(buf), "%s%d  EL %d  AZ %d  SNR %s", s.talker, s.prn, s.elevation, s.azimuth, snrBuf);

  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  int tw = d.textWidth(buf);
  int boxW = tw + 24, boxH = 30;
  int bx = r.x + (r.w - boxW) / 2;
  int by = r.y + r.h - boxH - 6;
  uint16_t bg = snrColor(s.snr);

  d.fillRoundRect(bx, by, boxW, boxH, boxH / 2, bg);
  d.setTextColor(COLOR_BG, bg);
  d.setCursor(bx + 12, by + (boxH - d.fontHeight()) / 2);
  d.print(buf);
}

static void drawSatPanel(const RenderSnapshot &s) {
  Rect r = innerOf(skyCard);
  // The radar blit and drawSatList() each clear their own region, so a
  // full-panel clear here would be redundant -- except drawSatTooltip()
  // floats near the bottom, centered across the full width, and can fall
  // outside both of those regions. Clear just its footprint so a stale or
  // expired tooltip box can never linger.
  canvas.fillRect(r.x, r.y + r.h - 40, r.w, 40, COLOR_CARD_BG);

  SkyGeom g = skyGeom(r);
  bool showNeedle = s.crsValid && s.spdValid && s.spd > 1.0; // course-over-ground is noisy near-stationary
  drawSkyPlot(s.sats, g.cx, g.cy, g.radius, showNeedle, (float)s.crs);

  int listX = g.cx + g.size / 2 + 4;
  int listW = r.x + r.w - listX;
  drawSatList(s.sats, listX, r.y, listW, r.h);

  drawSatTooltip(r);
}

static void drawLogPanel(const RenderSnapshot &s) {
  auto &d = canvas;
  Rect r = innerOf(logCard);
  d.fillRect(r.x, r.y, r.w, r.h, COLOR_CARD_BG);

  drawFilterChip(); // above innerOf(), but inside the logCard dirty rect

  // Expanded is the "read the sentences" view, so it gets larger type and the
  // full card width. The width matters: a long GSV sentence at Font4 runs to
  // roughly 980px, which would clip against the ~968px left column that the
  // sentence-type counters leave behind. Those counters stay in the collapsed
  // view, where the smaller font leaves width to spare.
  bool big = logExpanded;
  d.setFont(big ? &fonts::Font4 : &fonts::Font2);
  d.setTextSize(1);

  int rightW = big ? 0 : 220;
  int leftW = big ? r.w : r.w - rightW - 24;

  d.setTextColor(COLOR_ACCENT_GREEN, COLOR_CARD_BG);
  int rowH = d.fontHeight() + (big ? 6 : 5);
  int maxRows = r.h / rowH;
  int shown = min<uint32_t>(min<uint32_t>(s.logHead, LOG_LINES), (uint32_t)maxRows);

  // Oldest at the top, newest on the last row. Bottom-anchored so the newest
  // sentence sits on the final row even before the buffer has filled, instead
  // of the block hugging the top with a gap beneath it.
  int y = r.y + (maxRows - shown) * rowH;
  for (int i = 0; i < shown; i++) {
    int idx = (s.logHead - shown + i) % LOG_LINES;
    d.setCursor(r.x, y);
    d.print(s.logLines[idx]);
    y += rowH;
  }

  if (big) return; // no counter column in the expanded view

  int dividerX = r.x + leftW + 12;
  d.drawFastVLine(dividerX, r.y, r.h, COLOR_DIVIDER);

  // Right: cumulative sentence-type counts -- gives the wide log card a
  // purpose beyond the raw text, which rarely fills its own width.
  int cx0 = dividerX + 12;
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(cx0, r.y);
  d.print("SENTENCE TYPES");
  int cy = r.y + d.fontHeight() + 8;
  int colW = rightW / 2 - 6;
  int rowH2 = d.fontHeight() + 8;
  for (int i = 0; i < NUM_SENTENCE_TYPES; i++) {
    int col = i % 2, row = i / 2;
    int cxp = cx0 + col * (colW + 12);
    int cyp = cy + row * rowH2;
    char buf[16];
    snprintf(buf, sizeof(buf), "%-3s %lu", SENTENCE_TYPES[i], (unsigned long)s.typeCounts[i]);
    d.setTextColor(s.typeCounts[i] > 0 ? COLOR_TEXT_PRIMARY : COLOR_STATUS_NONE, COLOR_CARD_BG);
    d.setCursor(cxp, cyp);
    d.print(buf);
  }
}

// -------------------------------------------------- change detection/dirty --
// Each panel gets a cheap signature of exactly the values it renders. A panel
// is only redrawn when its own signature changes, so panels update at their
// own natural rates -- the fix/satellite cards effectively at the GPS's 1Hz,
// the NMEA log as sentences land, the badge row only when a badge's value
// changes -- rather than all of them on a shared timer.
//
// Fields are hashed individually rather than memcmp'd over a struct, so
// padding bytes can't produce spurious mismatches.

template <typename T>
static uint32_t hashVal(uint32_t h, const T &v) {
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&v);
  for (size_t i = 0; i < sizeof(T); i++) {
    h ^= p[i];
    h *= 16777619u;
  }
  return h;
}

static uint32_t hashStr(uint32_t h, const char *s) {
  while (*s) {
    h ^= (uint8_t)*s++;
    h *= 16777619u;
  }
  return h;
}

static constexpr uint32_t FNV_SEED = 2166136261u;

static uint32_t sigStatusBar(const RenderSnapshot &s) {
  uint32_t h = FNV_SEED;
  bool haveData = s.lastSentenceMs != 0 && millis() - s.lastSentenceMs < 5000;
  refreshBattery(); // rate-limited: the badge below reads the same cached pair
  h = hashVal(h, haveData);
  h = hashVal(h, battLevel);
  h = hashVal(h, battCharging);
  h = hashVal(h, sdReady);
#if ENABLE_WIFI_NMEA
  int clients = nmeaClientCount;
  h = hashVal(h, clients);
#endif
  bool pressLight = pressTarget == PressTarget::LIGHT;
  bool pressSleep = pressTarget == PressTarget::SLEEP;
  h = hashVal(h, pressLight);
  h = hashVal(h, pressSleep);
  return h;
}

static uint32_t sigFixPanel(const RenderSnapshot &s) {
  uint32_t h = FNV_SEED;
  uint8_t view = (uint8_t)positionView;
  h = hashVal(h, view);
  h = hashVal(h, s.fixType);
  h = hashVal(h, s.satsValid); h = hashVal(h, s.satsUsed); h = hashVal(h, s.visibleSats);

  if (view == (uint8_t)PositionView::TRIP) {
    h = hashVal(h, s.tripDistanceKm);
    h = hashVal(h, s.maxSpeedKmph);
    h = hashVal(h, s.timeToFirstFixMs);
    h = hashVal(h, s.speedHistHead); // the trend the trip face draws now
    for (int i = 0; i < 3; i++) h = hashVal(h, s.fixSecs[i]);
    // The trip clock ticks visibly, so fold in whole elapsed seconds.
    uint32_t elapsedSec = s.firstFixAbsMs ? (millis() - s.firstFixAbsMs) / 1000 : 0;
    h = hashVal(h, elapsedSec);
  } else {
    h = hashVal(h, s.locValid); h = hashVal(h, s.lat); h = hashVal(h, s.lon);
    h = hashVal(h, s.altValid); h = hashVal(h, s.alt);
    h = hashVal(h, s.spdValid); h = hashVal(h, s.spd);
    h = hashVal(h, s.crsValid); h = hashVal(h, s.crs);
    h = hashVal(h, s.hdopValid); h = hashVal(h, s.hdop);
    // The live view draws the HDOP trend now, not the DOP trio -- so the graph
    // has to be able to trigger its own repaint, and a PDOP that jitters
    // without anything visible changing must not.
    h = hashVal(h, s.hdopHistHead);
    h = hashVal(h, s.dateValid); h = hashVal(h, s.timeValid);
    h = hashVal(h, s.year); h = hashVal(h, s.month); h = hashVal(h, s.day);
    h = hashVal(h, s.hour); h = hashVal(h, s.minute); h = hashVal(h, s.second);
  }
  return h;
}

static uint32_t sigSatPanel(const RenderSnapshot &s) {
  uint32_t h = FNV_SEED;
  for (int i = 0; i < MAX_SATS; i++) {
    const SatInfo &sat = s.sats[i];
    if (!sat.used) continue;
    h = hashStr(h, sat.talker);
    h = hashVal(h, sat.prn);
    h = hashVal(h, sat.elevation);
    h = hashVal(h, sat.azimuth);
    h = hashVal(h, sat.snr);
  }
  bool needle = s.crsValid && s.spdValid && s.spd > 1.0;
  h = hashVal(h, needle);
  if (needle) h = hashVal(h, s.crs);

  // Fold in tooltip visibility so its 4s expiry triggers a repaint on its own.
  bool tipVisible = satTooltip.active && (millis() - satTooltip.shownAtMs <= 4000);
  h = hashVal(h, tipVisible);
  if (tipVisible) {
    h = hashStr(h, satTooltip.info.talker);
    h = hashVal(h, satTooltip.info.prn);
  }
  return h;
}

static uint32_t sigLogPanel(const RenderSnapshot &s) {
  uint32_t h = FNV_SEED;
  h = hashVal(h, s.logHead); // sentence-type counts advance in lockstep with this
  h = hashVal(h, logExpanded);
  h = hashVal(h, nmeaFilter); // the chip label lives in this panel
  bool pressFilter = pressTarget == PressTarget::FILTER;
  h = hashVal(h, pressFilter);
  return h;
}

// Regions whose canvas pixels changed this frame, pushed individually so a
// push costs only the area that actually moved rather than the full screen.
static constexpr int MAX_DIRTY = 4;
static Rect dirtyRects[MAX_DIRTY];
static int dirtyCount = 0;

static void markDirty(const Rect &r) {
  if (r.w <= 0 || r.h <= 0) return;
  if (dirtyCount < MAX_DIRTY) dirtyRects[dirtyCount++] = r;
}

// pushImage() clips by adjusting the copied rect and source offset rather than
// masking, so a clipped pushSprite genuinely transfers fewer pixels. Note the
// canvas lives in PSRAM, which disables DMA on the blit -- making the transfer
// CPU-bound and the saving proportional to the area skipped.
// Pushed in horizontal bands with a touch sample between them. A full-canvas
// push moves 1280x720x2 bytes out of PSRAM with no DMA, and for its whole
// duration nothing polls the panel -- so a tap that began and ended inside it
// was lost outright. Banding costs a few extra clip setups and bounds the
// blind window to one slice instead of the whole blit. It is invisible: the
// canvas is already fully composed, so the bands carry finished pixels.
static constexpr int PUSH_SLICE_H = 120;

static void pushRegion(const Rect &r) {
  int bottom = r.y + r.h;
  for (int y = r.y; y < bottom; y += PUSH_SLICE_H) {
    int h = min(PUSH_SLICE_H, bottom - y);
    M5.Display.setClipRect(r.x, y, r.w, h);
    canvas.pushSprite(&M5.Display, 0, 0);
    sampleTouch();
  }
  M5.Display.clearClipRect();
}

static void pushDirty(bool full) {
  if (full) {
    pushRegion({0, 0, SCREEN_W, SCREEN_H});
    return;
  }
  for (int i = 0; i < dirtyCount; i++) {
    pushRegion(dirtyRects[i]);
  }
}

// ------------------------------------------------------------------ setup --
// computeLayout()/relayout() now live in layout.cpp.

// ------------------------------------------------------------ power states --

static void rememberBrightness() {
  uint8_t b = M5.Display.getBrightness();
  if (b > 0) savedBrightness = b; // never latch 0, or wake would restore to dark
}

// NOTE: M5.Display.setBrightness() is a no-op on Tab5 with M5GFX 0.2.26 --
// Panel_Device::setBrightness() forwards to a Light instance, and Tab5's init
// path never attaches one (Panel_DSI/ST7123/ST7121 implement no brightness
// either, and neither IO expander carries a backlight pin). The calls are kept
// so this does the right thing if a future M5GFX wires one up, but the visible
// effect comes from blanking the canvas to black and pushing that.
static void blankScreen() {
  canvas.fillScreen(0);
  canvas.pushSprite(&M5.Display, 0, 0);
}

// The backlight really does go off at 0: M5GFX attaches no Light instance for
// Tab5, so M5.Display.setBrightness() does nothing and the DCS 0x51 write in
// panelSetBrightness() is the only control there is. It works -- confirmed on
// hardware by the panel visibly glowing when this was briefly set to
// BRIGHTNESS_MIN instead.
//
// That brief detour was an attempt to explain a sleeping screen that would not
// wake on a tap, on the theory that the ST7123 -- one controller for both the
// display and the digitiser -- might stop reporting touch with its display
// block idled. It is a real possibility: M5GFX's own Tab5 timing block warns
// that shrinking the vertical front porch "will cause the touch panel to stop
// working". But the same commit also fixed the wake path itself, which needed
// a press AND a release both seen through a 20ms sampling window, and that is
// the likelier reason a tap did nothing.
//
// So: dark again, with the wake fixes kept. If tapping a sleeping screen stops
// working, the "touch while dark" line on the serial monitor says which half is
// at fault -- and the targeted next step is clearing the BL bit in DCS 0x53
// (ctrl 0x28 rather than 0x2C), which switches the backlight off without
// touching the brightness value at all.
static void enterBacklightOff() {
  rememberBrightness();
  backlightOff = true;
  M5.Display.setBrightness(0);
  panelSetBrightness(0);
  blankScreen();
}

// Deliberately zeroes the backlight rather than calling M5.Display.sleep().
// Tab5's newer panels use an integrated display+touch controller (ST7123 /
// ST7121), so putting the panel to sleep risks taking the touch digitiser with
// it -- which would make tap-to-wake impossible. The backlight is the dominant
// consumer anyway, and the radio going down is the other half of the saving.
static void enterSleep() {
  rememberBrightness();
  asleep = true;
  wifiEnabled = false; // wifiTask tears the AP down and powers the radio off
  M5.Display.setBrightness(0);
  panelSetBrightness(0); // see enterBacklightOff() for why this is 0 again
  blankScreen();
}

static void wakeDisplay() {
  if (asleep) {
    wifiEnabled = true; // wifiTask brings the AP back up
    asleep = false;
  }
  backlightOff = false;
  M5.Display.setBrightness(savedBrightness);
  panelSetBrightness(savedBrightness);
  relayout(); // force a full repaint and full push
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  if (M5.Display.height() > M5.Display.width()) {
    M5.Display.setRotation((M5.Display.getRotation() + 1) % 4);
  }
  SCREEN_W = M5.Display.width();
  SCREEN_H = M5.Display.height();

  computeLayout();

  canvas.setColorDepth(16);
  canvas.setPsram(true);
  if (!canvas.createSprite(SCREEN_W, SCREEN_H)) {
    Serial.println("ERROR: failed to allocate PSRAM canvas -- check BOARD_HAS_PSRAM build flag");
  }

  // Build the sky-plot radar background once, matching the geometry
  // drawSatPanel() computes from skyCard (fixed for the life of the program
  // -- logExpanded zeroes skyCard out, but never resizes it otherwise).
  {
    SkyGeom g = skyGeom(innerOf(skyCard));
    int radius = g.radius;
    radarBgSize = g.size;

    radarBg.setColorDepth(16);
    radarBg.setPsram(true);
    radarBg.createSprite(radarBgSize, radarBgSize);
    radarBg.fillSprite(COLOR_CARD_BG);

    int bcx = radarBgSize / 2, bcy = radarBgSize / 2;
    radarBg.fillCircle(bcx, bcy, radius, COLOR_BG);
    radarBg.drawCircle(bcx, bcy, radius, COLOR_DIVIDER);
    radarBg.drawCircle(bcx, bcy, radius * 2 / 3, COLOR_DIVIDER);
    radarBg.drawCircle(bcx, bcy, radius / 3, COLOR_DIVIDER);
    radarBg.drawFastHLine(bcx - radius, bcy, radius * 2, COLOR_DIVIDER);
    radarBg.drawFastVLine(bcx, bcy - radius, radius * 2, COLOR_DIVIDER);

    // Tick ring: every 10 degrees, longer and brighter every 30. This is what
    // makes an azimuth readable off the plot -- four cardinal letters alone
    // leave you interpolating across a 90 degree gap.
    for (int a = 0; a < 360; a += 10) {
      bool major = (a % 30) == 0;
      float rad_a = radians((float)a);
      int r0 = radius + 3, r1 = radius + (major ? 15 : 8);
      int x0 = bcx + (int)(r0 * sinf(rad_a)), y0 = bcy - (int)(r0 * cosf(rad_a));
      int x1 = bcx + (int)(r1 * sinf(rad_a)), y1 = bcy - (int)(r1 * cosf(rad_a));
      if (major) {
        radarBg.drawWideLine(x0, y0, x1, y1, 2.0f, COLOR_TEXT_SECONDARY);
      } else {
        radarBg.drawLine(x0, y0, x1, y1, COLOR_DIVIDER);
      }
    }

    // Fixed index at north, in COLOR_TEXT_PRIMARY. White means "reference that
    // never moves" here; the live course chevron is COLOR_ACCENT so the two can
    // never be confused, and neither can be mistaken for a satellite, which is
    // always one of the three signal colours.
    radarBg.fillTriangle(bcx, bcy - radius - 2, bcx - 7, bcy - radius - 16, bcx + 7, bcy - radius - 16,
                         COLOR_TEXT_PRIMARY);

    // Elevation rings mean nothing unlabelled. Inner ring is 60 degrees, middle
    // 30 -- rr = radius * (90 - el) / 90, so the labels sit where each ring
    // crosses the vertical axis.
    radarBg.setFont(&fonts::Font0);
    radarBg.setTextSize(1);
    radarBg.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
    radarBg.setCursor(bcx + 4, bcy - radius / 3 - 3);
    radarBg.print("60");
    radarBg.setCursor(bcx + 4, bcy - radius * 2 / 3 - 3);
    radarBg.print("30");

    // Cardinals move out past the tick ring and step up to Font2: they were
    // Font0, the smallest type anywhere on the panel.
    radarBg.setFont(&fonts::Font2);
    radarBg.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
    auto label = [&](const char *s, int lx, int ly) {
      int tw = radarBg.textWidth(s), th = radarBg.fontHeight();
      radarBg.setCursor(lx - tw / 2, ly - th / 2);
      radarBg.print(s);
    };
    int lr = radius + 30;
    label("N", bcx, bcy - lr);
    label("S", bcx, bcy + lr);
    label("E", bcx + lr, bcy);
    label("W", bcx - lr, bcy);
  }

  drawStaticChrome();
  canvas.pushSprite(&M5.Display, 0, 0);

  Serial.begin(115200);
  GPSSerial.setRxBufferSize(2048); // headroom for the mutex-guarded parse burst
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Serial.println("Tab5 GPS logger starting...");

  initSdLogging();

  stateMutex = xSemaphoreCreateMutex();
#if ENABLE_WIFI_NMEA
  initWifiQueue();
#endif
  startGpsTask();
#if ENABLE_WIFI_NMEA
  startWifiTask();
#endif
}

void loop() {
  M5.update();

  // Touch edge detection, done here rather than via wasClicked()/wasReleased().
  //
  // getDetail(i) resolves through _touch_raw[i].id, and _touch_raw is only
  // refreshed for the points the current scan reports. On release the driver
  // reports zero points, so those entries hold stale data from an earlier
  // scan -- and unlike getTouchPointRaw(), getDetail() does not clamp the
  // index against the live point count. Reading a second slot that way is
  // therefore not deterministic: the stale id can alias back to slot 0 or
  // point at an unrelated detail slot.
  //
  // Instead: sample coordinates only while a contact is definitely down (raw
  // data valid then), and detect the lift ourselves as the absence of a
  // press. That is unambiguous, fires exactly once per physical touch, and
  // is immune both to the ghost second contact this panel emits and to the
  // state machine escalating a tap to hold/flick/drag.
  sampleTouch();
  bool pressing = touchPressing;
  bool pressEdge = touchPressEdge;
  bool released = touchReleaseEdge;
  touchPressEdge = false;
  touchReleaseEdge = false;

  // Everything below hit-tests the touch-DOWN position, not the lift position.
  // A fingertip rolls as it leaves the glass, so the last sample before a lift
  // can sit several pixels from where the user actually aimed -- and it is the
  // aim that should decide.
  if (pressEdge) capturedTarget = hitTestChips(touchDownX, touchDownY);

  // While the screen is dark, a tap only restores it and is consumed -- you
  // can't aim at a control you can't see, so no other target should fire.
  // Note GPS ingestion, SD logging and trip stats all keep running here; only
  // the display work is skipped.
  if (asleep || backlightOff) {
    // Wake on the press, not the lift. Waiting for a full press-release cycle
    // meant both edges had to be observed through a 20ms sampling window; the
    // press alone is unambiguous, and there is nothing else a tap could mean
    // while the screen is dark.
    if (pressEdge || released) {
      capturedTarget = PressTarget::NONE; // the waking tap commits nothing else
      wakeDisplay();
      return;
    }
    // Escape hatch: if the panel stops reporting touch while dark, the power
    // button still gets the display back.
    if (M5.BtnPWR.wasClicked()) {
      capturedTarget = PressTarget::NONE;
      wakeDisplay();
      return;
    }
    // Whether the digitiser reports anything at all while dark is the one
    // thing that cannot be reasoned about off-device, so say so on the wire.
    static uint32_t lastDarkLogMs = 0;
    if (touchPressing && millis() - lastDarkLogMs > 1000) {
      lastDarkLogMs = millis();
      Serial.printf("touch while dark: %d,%d\n", touchLastX, touchLastY);
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // nothing is visible, so ease off the CPU
    return;
  }

  // Highlight whichever chip the finger is currently over, and drop it on lift.
  // The highlight shows the captured chip for the whole press, rather than
  // re-testing the moving contact. That keeps it honest: what is lit is what
  // will fire on release, including when the finger has drifted off the chip.
  PressTarget prevPress = pressTarget;
  if (pressing) {
    pressTarget = capturedTarget;
  } else if (released) {
    pressTarget = PressTarget::NONE;
  }
  bool pressChanged = pressTarget != prevPress;

  // The brightness overlay is modal: it owns touch and drawing while open, so
  // the panels underneath can't paint over it.
  if (dimmerOpen) {
    // A press that starts on the slider grabs it, and keeps it until the lift.
    // Only X matters from then on: testing the live position against the track
    // each sample meant any vertical drift mid-slide stopped the value dead,
    // which reads as the slider sticking.
    if (pressEdge && pointInRect(touchDownX, touchDownY, dimmerTrackHitRect())) {
      dimmerDragging = true;
    }
    if (dimmerDragging && pressing) {
      if (setBrightnessFromTouch(touchLastX)) dimmerDirty = true;
    }
    if (released) dimmerDragging = false;
    if (pressChanged) dimmerDirty = true;

    if (released) {
      PressTarget hit = capturedTarget;
      capturedTarget = PressTarget::NONE;
      if (hit == PressTarget::DIM_OFF) {
        dimmerOpen = false;
        enterBacklightOff(); // keeps the level just chosen for the next wake
        return;
      }
      if (hit == PressTarget::DIM_DONE || !pointInRect(touchDownX, touchDownY, dimmerPanelRect())) {
        dimmerOpen = false;
        relayout(); // repaint whatever the overlay was covering
        return;
      }
      dimmerDirty = true;
    }

    if (dimmerDirty) {
      dimmerDirty = false;
      drawDimmer();
      dirtyCount = 0;
      markDirty(dimmerPanelRect());
      pushDirty(false);
    }
    return;
  }

  // Each branch marks only the panel(s) it actually affects. A satellite tap
  // only needs the sat panel redrawn; the fix/log cards are untouched by it.
  // A view/expand toggle calls relayout(), which repaints the whole canvas'
  // static chrome, so all three panel *contents* need repainting afterward
  // regardless of which card was tapped.
  bool wantFix = false, wantSat = false, wantLog = false;
  {
    {
      // The touch-DOWN position, not the lift position: a fingertip rolls as
      // it leaves the glass, and the aim is what should decide.
      int tx = touchDownX, ty = touchDownY;
      if (released && tx >= 0 && ty >= 0) {
        // Chips commit on the press that began inside them, so a small drift
        // before the lift no longer swallows the tap.
        PressTarget chip = capturedTarget;
        capturedTarget = PressTarget::NONE;

        // Scoping the hit test to skyCard keeps the enlarged pick radius from
        // reaching across the gap into a neighbouring card.
        bool inSky = !logExpanded && pointInRect(tx, ty, skyCard);
        int hit = inSky ? hitTestSkyDot(tx, ty) : -1;
        if (chip == PressTarget::LIGHT) {
          dimmerOpen = true;
          dimmerDirty = true;
          return; // the modal block draws it on the next pass
        } else if (chip == PressTarget::SLEEP) {
          enterSleep();
          return;
        } else if (hit >= 0) {
          satTooltip = {true, skyDots[hit].info, millis()};
          wantSat = true;
        } else if (inSky) {
          // Tapped the sky plot but not a satellite: close any open tooltip so
          // the tap still produces visible feedback instead of reading as dead.
          if (satTooltip.active) {
            satTooltip.active = false;
            wantSat = true;
          }
        } else if (!logExpanded && pointInRect(tx, ty, fixCard)) {
          positionView = positionView == PositionView::LIVE ? PositionView::TRIP : PositionView::LIVE;
          relayout();
          wantFix = wantSat = wantLog = true;
        } else if (chip == PressTarget::FILTER) {
          // Checked before the logCard branch -- the chip sits inside it.
          // Clearing the ring under the lock gives immediate feedback and
          // avoids showing a mix of pre- and post-filter sentences.
          xSemaphoreTake(stateMutex, portMAX_DELAY);
          nmeaFilter = (nmeaFilter + 1) % NMEA_FILTER_COUNT;
          rawLogHead = 0;
          for (auto &l : rawLog) l[0] = '\0';
          xSemaphoreGive(stateMutex);
          relayout();
          wantFix = wantSat = wantLog = true;
        } else if (pointInRect(tx, ty, logCard)) {
          logExpanded = !logExpanded;
          relayout();
          wantFix = wantSat = wantLog = true;
        }
      }
    }
  }
  bool touchAction = wantFix || wantSat || wantLog;

  // relayout() repaints the whole canvas (background + card frames), so those
  // frames need a full-screen push rather than per-panel dirty rects.
  bool fullRepaint = relayoutPending;
  relayoutPending = false;

  static uint32_t lastPollMs = 0;
  static RenderSnapshot snap;
  static bool haveSnapshot = false;
  uint32_t now = millis();

  // Polling only samples signatures -- it does not imply any drawing. Panels
  // whose values are unchanged cost nothing here, so this rate sets how
  // quickly a change is *noticed*, not how often anything is repainted.
  bool poll = now - lastPollMs >= 200;
  // pressChanged bypasses the poll interval so press highlights track the
  // finger instead of lagging up to a full poll behind it.
  if (!poll && !touchAction && !fullRepaint && !pressChanged) return;
  if (poll) lastPollMs = now;

  // Only re-snapshot on a poll. A touch changes UI state, not GPS state, and
  // captureSnapshot() copies MAX_SATS + LOG_LINES Strings while holding
  // stateMutex -- doing that per tap added heap churn to the tap path and
  // blocked gpsTask from draining the UART for its duration.
  if (poll || !haveSnapshot) {
    captureSnapshot(snap);
    haveSnapshot = true;
  }

  static uint32_t lastSigStatus = 0, lastSigFix = 0, lastSigSat = 0, lastSigLog = 0;
  uint32_t nSigStatus = sigStatusBar(snap);
  uint32_t nSigFix = sigFixPanel(snap);
  uint32_t nSigSat = sigSatPanel(snap);
  uint32_t nSigLog = sigLogPanel(snap);

  bool doStatus = fullRepaint || nSigStatus != lastSigStatus;
  bool doFix = !logExpanded && (fullRepaint || wantFix || nSigFix != lastSigFix);
  bool doSat = !logExpanded && (fullRepaint || wantSat || nSigSat != lastSigSat);
  bool doLog = fullRepaint || wantLog || nSigLog != lastSigLog;

  lastSigStatus = nSigStatus;
  lastSigFix = nSigFix;
  lastSigSat = nSigSat;
  lastSigLog = nSigLog;

  if (!doStatus && !doFix && !doSat && !doLog) return;

  dirtyCount = 0;
  if (doStatus) {
    drawStatusPill(snap.lastSentenceMs);
    markDirty({TOPBAR_CTRL_X, 0, SCREEN_W - TOPBAR_CTRL_X, TOPBAR_H});
  }
  if (doFix) {
    drawFixPanel(snap);
    markDirty(fixCard);
  }
  if (doSat) {
    drawSatPanel(snap);
    markDirty(skyCard);
  }
  if (doLog) {
    drawLogPanel(snap);
    markDirty(logCard);
  }

  pushDirty(fullRepaint);
}
