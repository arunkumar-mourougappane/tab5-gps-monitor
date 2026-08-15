// Reads NMEA sentences from an M5Stack GPS/BDS Unit v1.1 (AT6668/ATGM336H)
// connected to Tab5's Port A (Grove), logs the decoded fix to USB serial, and
// renders a touch-interactive dashboard (position/trip card, satellite sky
// plot + signal list, raw NMEA log) on the Tab5's screen.
//
// Port A is a HY2.0-4P Grove connector: Black=GND, Red=5V, Yellow=G53, White=G54.
// By Grove/M5Stack convention Yellow carries the unit's RX line and White its TX
// line, so from the Tab5's side: G53 transmits to the GPS, G54 receives from it.
// This wasn't confirmed in M5Stack's Tab5 pinout doc -- swap GPS_RX_PIN/GPS_TX_PIN
// (config.h) if no sentences show up.
//
// TinyGPSPlus only parses GGA/RMC-family fields (location, altitude, speed,
// course, date/time, satellite count, HDOP). The satellite table (per-PRN
// elevation/azimuth/SNR) comes from GSV, and the 2D/3D fix mode + PDOP/VDOP
// come from GSA -- neither is exposed by TinyGPSPlus, so both are parsed
// directly off the raw sentence stream (nmea_parser.cpp).
//
// GPS ingestion runs on its own FreeRTOS task (gps_task.cpp), pinned to the
// core opposite the Arduino loop task, so a slow display redraw can never
// stall draining the UART. The default RX ring buffer (256B) only absorbs
// ~22ms of silence at 115200 baud -- comfortably less than a full-panel
// redraw was taking in a single-loop design, which is the most likely cause
// of dropped/garbled sentences ("choppy" updates). Shared state is protected
// by stateMutex (render_snapshot.h); the render side only holds it long
// enough to copy a snapshot, then draws unlocked so gpsTask is never blocked
// by slow screen I/O.
//
// Touch (app_input.cpp): tap the position card to flip between the live
// readout and a trip summary; tap a satellite dot in the sky plot for its
// detail; tap the NMEA card to expand it full-height. Touch state is only
// ever read/written from loop() (the render side), so it needs no locking
// against gpsTask.
//
// The firmware is split into modules under src/ -- this file is wiring only:
// setup() brings each subsystem up in dependency order, loop() calls
// app_input::handleTouch() then render_pipeline::runRenderCycle().

#include <M5Unified.h>
#include "config.h"
#include "display.h"
#include "layout.h"
#include "sd_logger.h"
#include "wifi_nmea.h"
#include "gps_task.h"       // GPSSerial, gps, startGpsTask()
#include "render_snapshot.h" // stateMutex
#include "ui_chrome.h"      // drawStaticChrome()
#include "ui_sky_panel.h"   // buildRadarSprite()
#include "app_input.h"      // handleTouch()
#include "render_pipeline.h" // runRenderCycle()

// Definition matching the extern in display.h. Every drawing module reaches
// the panel through this off-screen PSRAM sprite: draw* functions render into
// it, and render_pipeline blits the finished frame in pushSprite() calls.
// Without this, each fillRect-then-redraw step would be visible on the panel
// as a blank flash before the new content lands.
M5Canvas canvas(&M5.Display);

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
  bool wantFix, wantSat, wantLog, pressChanged;
  if (!handleTouch(wantFix, wantSat, wantLog, pressChanged)) return;
  runRenderCycle(wantFix, wantSat, wantLog, pressChanged);
}
