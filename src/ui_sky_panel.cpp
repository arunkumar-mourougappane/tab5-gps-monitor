#include "ui_sky_panel.h"
#include <cstring>
#include "display.h"
#include "theme.h"
#include "layout.h"
#include "gps_model.h"

// Pre-rendered sky-plot background (rings + N/S/E/W labels + bezel): built
// once in setup() and blitted each tick instead of redrawing it from scratch
// -- that redraw was one of the most expensive things happening every 200ms.
static M5Canvas radarBg(&canvas);
static int radarBgSize = 0;

void buildRadarSprite() {
  // Matches the geometry drawSatPanel() computes from skyCard (fixed for the
  // life of the program -- logExpanded zeroes skyCard out, but never resizes
  // it otherwise).
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

struct SkyDot { int x, y; SatInfo info; };
static SkyDot skyDots[MAX_SATS];
static int skyDotCount = 0;

// Picks the *nearest* dot rather than the first in array order that happens
// to qualify: satellites cluster near the zenith, and since the tooltip
// reports one specific satellite's details, grabbing an arbitrary neighbour
// would show data for a satellite the user didn't aim at.
static constexpr int SKY_DOT_HIT_RADIUS = 34;

int hitTestSkyDot(int px, int py) {
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

struct SatTooltip { bool active = false; SatInfo info; uint32_t shownAtMs = 0; };
static SatTooltip satTooltip;

void setSatTooltip(int dotIndex) {
  satTooltip = {true, skyDots[dotIndex].info, (uint32_t)millis()};
}

void clearSatTooltip() { satTooltip.active = false; }

bool satTooltipActive() { return satTooltip.active; }

bool satTooltipVisible(uint32_t nowMs) {
  return satTooltip.active && (nowMs - satTooltip.shownAtMs <= 4000);
}

SatInfo satTooltipInfo() { return satTooltip.info; }

// Polar sky plot: center = zenith (elevation 90), edge = horizon (elevation
// 0). Also draws a heading chevron on the bezel (course-over-ground) when
// moving, and records each dot's screen position/data into skyDots[] for
// touch hit-testing.
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

void drawSatPanel(const RenderSnapshot &s) {
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
