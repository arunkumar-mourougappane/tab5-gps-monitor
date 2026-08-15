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

#include "theme.h"
#include "layout.h"

#include "gps_model.h"
#include "trip_stats.h"
#include "nmea_parser.h"

// Fixed size so the hit target matches the drawn chip regardless of label, and
// so the chip can't resize under a changing label the way the status badges did.
static constexpr int FILTER_CHIP_W = 92;
static constexpr int FILTER_CHIP_H = 26;


// logSentence()/handleRawSentence() now live in nmea_parser.cpp.

// formatFix()/gpsTaskFn()/startGpsTask() now live in gps_task.cpp.

// RenderSnapshot / captureSnapshot() now live in render_snapshot.h/.cpp.

// ---------------------------------------------------------- touch/UI state --
// Only ever touched from loop() (render side) -- no locking needed.


#include "power.h" // backlightOff, asleep, savedBrightness, enter*/wakeDisplay

// Which chip the finger is currently down on, so it can render pressed. Held
// across frames (not just on release) so the highlight tracks the finger.
enum class PressTarget : uint8_t { NONE, LIGHT, SLEEP, FILTER, DIM_OFF, DIM_DONE };
PressTarget pressTarget = PressTarget::NONE; // read by ui_status_bar.cpp, ui_dimmer.cpp

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

#include "ui_dimmer.h" // dimmerOpen/Dirty/Dragging, BRIGHTNESS_MIN/MAX, dimmer draw/geometry


#include "ui_widgets.h"   // drawHeroValue/MiniStat/Badge/CardFrame/Chip/Sparkline
#include "ui_chrome.h"    // drawStaticChrome()
#include "ui_status_bar.h" // badge geometry, LIGHT/SLEEP rects, drawStatusPill()


#include "power.h"      // panelSetBrightness()
#include "ui_dimmer.h"  // dimmer geometry, setBrightnessFromTouch(), drawDimmer()

#include "ui_fix_panel.h"
#include "ui_sky_panel.h"
#include "ui_log_panel.h"

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

// rememberBrightness()/blankScreen()/enterBacklightOff()/enterSleep()/wakeDisplay() now live in power.cpp.

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

  buildRadarSprite(); // sky-plot rings/bezel, matching skyCard's geometry

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
          setSatTooltip(hit);
          wantSat = true;
        } else if (inSky) {
          // Tapped the sky plot but not a satellite: close any open tooltip so
          // the tap still produces visible feedback instead of reading as dead.
          if (satTooltipActive()) {
            clearSatTooltip();
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
