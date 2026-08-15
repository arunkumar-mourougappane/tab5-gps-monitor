#include "render_pipeline.h"
#include <Arduino.h>
#include "display.h"
#include "render_snapshot.h"
#include "gps_model.h"
#include "nmea_parser.h"
#include "sd_logger.h"
#include "wifi_nmea.h"
#include "touch_input.h"
#include "ui_fix_panel.h"
#include "ui_sky_panel.h"
#include "ui_log_panel.h"
#include "ui_status_bar.h"

// ------------------------------------------------ signature hashing --
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
  bool tipVisible = satTooltipVisible(millis());
  h = hashVal(h, tipVisible);
  if (tipVisible) {
    SatInfo info = satTooltipInfo();
    h = hashStr(h, info.talker);
    h = hashVal(h, info.prn);
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

// -------------------------------------------------------- dirty rects --
static Rect dirtyRects[MAX_DIRTY];
int dirtyCount = 0;

void markDirty(const Rect &r) {
  if (r.w <= 0 || r.h <= 0) return;
  if (dirtyCount < MAX_DIRTY) dirtyRects[dirtyCount++] = r;
}

// pushImage() clips by adjusting the copied rect and source offset rather than
// masking, so a clipped pushSprite genuinely transfers fewer pixels. Note the
// canvas lives in PSRAM, which disables DMA on the blit -- making the transfer
// CPU-bound and the saving proportional to the area skipped.
//
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

void pushDirty(bool full) {
  if (full) {
    pushRegion({0, 0, SCREEN_W, SCREEN_H});
    return;
  }
  for (int i = 0; i < dirtyCount; i++) {
    pushRegion(dirtyRects[i]);
  }
}

// ------------------------------------------------------- render cycle --
void runRenderCycle(bool wantFix, bool wantSat, bool wantLog, bool pressChanged) {
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
  // pressChanged (computed by app_input's handleTouch(), which owns pressTarget's
  // transition) bypasses the poll interval so press highlights track the finger
  // instead of lagging up to a full poll behind it.
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
