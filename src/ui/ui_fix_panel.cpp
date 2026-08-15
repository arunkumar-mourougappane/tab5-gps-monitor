#include "ui/ui_fix_panel.h"
#include <cstring>
#include "core/display.h"
#include "core/theme.h"
#include "core/layout.h"
#include "ui/ui_widgets.h"
#include "model/gps_model.h"

PositionView positionView = PositionView::LIVE; // read by ui_chrome.cpp

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

void drawFixPanel(const RenderSnapshot &s) {
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
