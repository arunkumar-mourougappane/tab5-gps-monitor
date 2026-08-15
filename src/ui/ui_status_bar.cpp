#include "ui/ui_status_bar.h"
#include "core/display.h"
#include "core/theme.h"
#include "ui/ui_widgets.h"
#include "io/sd_logger.h"
#include "io/wifi_nmea.h"

// PressTarget is still defined in main.cpp; forward-declared here since only
// its LIGHT/SLEEP/FILTER members are read (for the pressed-state colour swap).
enum class PressTarget : uint8_t { NONE, LIGHT, SLEEP, FILTER, DIM_OFF, DIM_DONE };
extern PressTarget pressTarget;

Rect lightBtnRect() {
  return {TOPBAR_CTRL_X, (TOPBAR_H - BADGE_H) / 2, TOPBAR_CTRL_W, BADGE_H};
}

Rect sleepBtnRect() {
  return {TOPBAR_CTRL_X + TOPBAR_CTRL_W + BADGE_GAP, (TOPBAR_H - BADGE_H) / 2, TOPBAR_CTRL_W, BADGE_H};
}

Rect lightHitRect() {
  Rect c = lightBtnRect();
  return {c.x - 5, 0, c.w + 10, TOPBAR_H};
}

Rect sleepHitRect() {
  Rect c = sleepBtnRect();
  return {c.x - 5, 0, c.w + 10, TOPBAR_H};
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
int32_t battLevel = -1;
bool battCharging = false;
static uint32_t battReadMs = 0;

void refreshBattery() {
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

void drawStatusPill(uint32_t snapLastSentenceMs) {
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
