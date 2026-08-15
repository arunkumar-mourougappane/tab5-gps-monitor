#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "io/gps_task.h"
#include "core/config.h"
#include "model/gps_model.h"
#include "model/trip_stats.h"
#include "model/nmea_parser.h"
#include "io/sd_logger.h"
#include "model/render_snapshot.h"

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// Formats rather than prints: the caller holds stateMutex while filling the
// buffer (every field below is shared state) and writes it to the port after
// releasing, so a blocked USB CDC endpoint can't stall UART draining.
static void formatFix(char *out, size_t cap) {
  int n = snprintf(out, cap, "fix=%d sats=%d", (int)gps.location.isValid(),
                   gps.satellites.isValid() ? (int)gps.satellites.value() : 0);

  if (gps.location.isValid())
    n += snprintf(out + n, cap - n, " lat=%.6f lon=%.6f", gps.location.lat(), gps.location.lng());
  if (gps.altitude.isValid())
    n += snprintf(out + n, cap - n, " alt=%.1fm", gps.altitude.meters());
  if (gps.speed.isValid())
    n += snprintf(out + n, cap - n, " speed=%.1fkm/h", gps.speed.kmph());
  if (gps.date.isValid() && gps.time.isValid())
    snprintf(out + n, cap - n, " utc=%04d-%02d-%02d %02d:%02d:%02d", gps.date.year(), gps.date.month(),
             gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
}

static void gpsTaskFn(void *) {
  char lineBuf[RAW_LINE_MAX];
  int lineLen = 0;
  uint32_t taskStartMs = millis();
  uint32_t lastWarnMs = 0;
  uint32_t lastFixEpoch = 0;

  for (;;) {
    if (GPSSerial.available()) {
      bool updated = false;
      // Reading the port and assembling a line need no lock -- the UART and
      // lineBuf belong to this task. The lock is taken per completed sentence
      // instead of once around the whole drain, so the render task can
      // interleave between sentences rather than waiting out a full burst.
      while (GPSSerial.available()) {
        char c = GPSSerial.read();

        if (c == '$') {
          lineBuf[0] = '$';
          lineLen = 1;
        } else if (c == '\n') {
          if (lineLen > 6) {
            lineBuf[lineLen] = '\0';
            logSentence(lineBuf); // SD + TCP queue: gpsTask-only, so no lock

            xSemaphoreTake(stateMutex, portMAX_DELAY);
            // Feeding TinyGPSPlus the assembled sentence rather than the raw
            // byte stream: bytes outside a sentence carry no information it
            // can use, and a line long enough to have been truncated by the
            // assembler would fail its checksum either way.
            for (int k = 0; k < lineLen; k++) {
              if (gps.encode(lineBuf[k])) updated = true;
            }
            if (gps.encode('\r')) updated = true;
            if (gps.encode('\n')) updated = true;
            handleRawSentence(lineBuf, lineLen);
            xSemaphoreGive(stateMutex);
          }
          lineLen = 0;
        } else if (c != '\r' && lineLen > 0 && lineLen < RAW_LINE_MAX - 4) {
          lineBuf[lineLen++] = c;
        }
      }

      char trackRow[128] = {0};
      char fixLine[192] = {0};

      xSemaphoreTake(stateMutex, portMAX_DELAY);
      pruneStaleSats();

      // One epoch = one receiver fix. This block used to run once per drain
      // burst instead, and a burst is not a fix: the task wakes every 2ms, so
      // a second's worth of sentences arriving over ~50ms produces on the
      // order of 25 bursts. That meant ~25 track rows and ~25 SD flushes a
      // second rather than the one the CSV is meant to hold, and an HDOP
      // history spanning under two seconds rather than the last 40 fixes.
      //
      // The epoch key is the receiver's own UTC stamp, not TinyGPSPlus's
      // isUpdated(): reading a value clears that flag, and captureSnapshot()
      // reads every one of them from the render task five times a second, so
      // the update flags are not reliably observable from here.
      uint32_t epoch = gps.time.isValid() ? gps.time.value() : (millis() / 1000);
      bool newEpoch = epoch != lastFixEpoch;
      if (newEpoch) lastFixEpoch = epoch;

      if (newEpoch && gps.location.isValid()) {
        double lat = gps.location.lat(), lon = gps.location.lng();
        if (haveLastFix) {
          double d = haversineKm(lastFixLat, lastFixLon, lat, lon);
          if (d < 1.0) tripDistanceKm += d; // guard against wild jumps from a bad fix
        } else {
          haveLastFix = true;
          timeToFirstFixMs = millis() - taskStartMs;
          firstFixAbsMs = millis();
        }
        lastFixLat = lat;
        lastFixLon = lon;

        // Formatted here (every field is shared state), written after the
        // release below.
        if (sdReady) {
          snprintf(trackRow, sizeof(trackRow), "%04d-%02d-%02dT%02d:%02d:%02dZ,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%d\n",
                   gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(),
                   gps.time.second(), lat, lon, gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
                   gps.speed.isValid() ? gps.speed.kmph() : 0.0, gps.course.isValid() ? gps.course.deg() : 0.0,
                   gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, gps.satellites.isValid() ? gps.satellites.value() : 0);
        }
      }
      if (newEpoch) {
        if (gps.speed.isValid() && gps.speed.kmph() > maxSpeedKmph) maxSpeedKmph = gps.speed.kmph();
        if (gps.hdop.isValid()) pushHdopHistory((float)gps.hdop.hdop());
        pushSpeedHistory(gps.speed.isValid() ? (float)gps.speed.kmph() : 0.0f);
        int fixIdx = constrain(gsaFixType - 1, 0, 2);
        fixSecs[fixIdx]++;
        // Also once per fix rather than once per parsed sentence.
        if (updated) formatFix(fixLine, sizeof(fixLine));
      }
      xSemaphoreGive(stateMutex);

      // Both of these are slow, and neither is shared -- so they happen with
      // the lock already released.
      if (trackRow[0] != '\0') writeTrackRow(trackRow);
      if (fixLine[0] != '\0') Serial.println(fixLine);
    }

    uint32_t now = millis();
    if (now - taskStartMs > 15000 && now - lastWarnMs > 5000) {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      bool noData = gps.charsProcessed() < 10;
      xSemaphoreGive(stateMutex);
      if (noData) {
        Serial.println("WARNING: no GPS data received -- check wiring/baud rate");
      }
      lastWarnMs = now;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// Only spawns the task -- GPSSerial itself is configured earlier in setup(),
// at the same point it always was, so this doesn't reorder anything relative
// to Serial.begin()/initSdLogging()/stateMutex creation.
void startGpsTask() {
  int mainCore = xPortGetCoreID();
  int gpsCore = mainCore == 0 ? 1 : 0;
  xTaskCreatePinnedToCore(gpsTaskFn, "gps_task", 4096, nullptr, 2, nullptr, gpsCore);
}
