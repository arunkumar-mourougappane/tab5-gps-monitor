#include <Arduino.h> // min/max
#include "core/layout.h"

// Forward-declared rather than included from ui_chrome.h/ui_log_panel.h: those
// modules don't exist as separate translation units yet in this extraction --
// both functions are still defined (non-static, for external linkage) in
// main.cpp until their turn comes.
void drawStaticChrome();
void drawFilterChip();

bool logExpanded = false;

int SCREEN_W = 1280;
int SCREEN_H = 720;

Rect fixCard, skyCard, logCard;

Rect innerOf(const Rect &r) {
  return {r.x + 18, r.y + CARD_HEADER_H + 8, r.w - 36, r.h - CARD_HEADER_H - 24};
}

bool pointInRect(int px, int py, const Rect &r) {
  return px >= r.x && px < r.x + r.w && py >= r.y && py < r.y + r.h;
}

SkyGeom skyGeom(const Rect &skyInner) {
  int radius = min(skyInner.h, (int)(skyInner.w * 0.46f)) / 2 - 25;
  int size = (radius + SKY_BEZEL_PAD) * 2;
  return {radius, size, skyInner.x + size / 2, skyInner.y + skyInner.h / 2};
}

void computeLayout() {
  int contentTop = TOPBAR_H + MARGIN;

  if (logExpanded) {
    logCard = {MARGIN, contentTop, SCREEN_W - 2 * MARGIN, SCREEN_H - contentTop - MARGIN};
    fixCard = {0, 0, 0, 0};
    skyCard = {0, 0, 0, 0};
    return;
  }

  logCard = {MARGIN, SCREEN_H - MARGIN - LOGCARD_H, SCREEN_W - 2 * MARGIN, LOGCARD_H};
  int row1Bottom = logCard.y - MARGIN;
  int row1H = row1Bottom - contentTop;

  int fixW = (int)((SCREEN_W - 3 * MARGIN) * 0.36f);
  fixCard = {MARGIN, contentTop, fixW, row1H};
  int skyX = MARGIN * 2 + fixW;
  skyCard = {skyX, contentTop, SCREEN_W - skyX - MARGIN, row1H};
}

bool relayoutPending = true;

void relayout() {
  computeLayout();
  drawStaticChrome();
  // The filter chip is drawn here rather than from inside drawStaticChrome()
  // itself: chrome is "whole-screen background," and reaching into the log
  // panel's own chip from there would be the wrong direction of coupling.
  // Same two draws, same order, as before the split.
  drawFilterChip();
  relayoutPending = true;
}
