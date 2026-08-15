#include "ui_dimmer.h"
#include "display.h"
#include "theme.h"
#include "ui_widgets.h"
#include "power.h"

// PressTarget is still defined in main.cpp.
enum class PressTarget : uint8_t { NONE, LIGHT, SLEEP, FILTER, DIM_OFF, DIM_DONE };
extern PressTarget pressTarget;

bool dimmerOpen = false;
bool dimmerDirty = false;
bool dimmerDragging = false;

Rect dimmerPanelRect() {
  constexpr int w = 560, h = 210;
  return {(SCREEN_W - w) / 2, (SCREEN_H - h) / 2, w, h};
}

Rect dimmerTrackRect() {
  Rect p = dimmerPanelRect();
  return {p.x + 30, p.y + 82, p.w - 60, 26};
}

Rect dimmerOffRect() {
  Rect p = dimmerPanelRect();
  return {p.x + 30, p.y + p.h - 62, 150, 44};
}

Rect dimmerDoneRect() {
  Rect p = dimmerPanelRect();
  return {p.x + p.w - 30 - 150, p.y + p.h - 62, 150, 44};
}

// The track is drawn 26px tall but the knob riding on it is 34, and a
// horizontal drag never stays inside a 26px band -- so the area that can
// start and hold a drag is padded well past both. It stops short of OFF/DONE
// at y+148 and of the percentage readout above.
Rect dimmerTrackHitRect() {
  Rect t = dimmerTrackRect();
  return {t.x - 10, t.y - 18, t.w + 20, t.h + 36};
}

bool setBrightnessFromTouch(int tx) {
  Rect t = dimmerTrackRect();
  int rel = constrain(tx - t.x, 0, t.w);
  uint8_t level = (uint8_t)(BRIGHTNESS_MIN + (long)(BRIGHTNESS_MAX - BRIGHTNESS_MIN) * rel / t.w);
  if (level == savedBrightness) return false;
  savedBrightness = level;
  M5.Display.setBrightness(savedBrightness); // no-op today; harmless if M5GFX gains a Light
  panelSetBrightness(savedBrightness);
  return true;
}

void drawDimmer() {
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
