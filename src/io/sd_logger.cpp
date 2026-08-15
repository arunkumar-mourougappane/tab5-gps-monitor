#include <Arduino.h>
#include <SD_MMC.h>
#include "io/sd_logger.h"

bool sdReady = false;
static File nmeaLogFile;
static File trackLogFile;
static uint32_t sdFlushCounter = 0;

static int nextSdSessionIndex() {
  int idx = 1;
  File f = SD_MMC.open("/session.txt", FILE_READ);
  if (f) {
    idx = f.parseInt() + 1;
    f.close();
  }
  File fw = SD_MMC.open("/session.txt", FILE_WRITE);
  if (fw) {
    fw.print(idx);
    fw.close();
  }
  return idx;
}

void initSdLogging() {
  if (!SD_MMC.begin()) {
    Serial.println("SD_MMC mount failed -- SD logging disabled (no card, or unsupported card)");
    return;
  }
  int idx = nextSdSessionIndex();
  char nmeaPath[32], trackPath[32];
  snprintf(nmeaPath, sizeof(nmeaPath), "/gps_%04d.nmea", idx);
  snprintf(trackPath, sizeof(trackPath), "/track_%04d.csv", idx);

  nmeaLogFile = SD_MMC.open(nmeaPath, FILE_WRITE);
  trackLogFile = SD_MMC.open(trackPath, FILE_WRITE);
  if (trackLogFile) trackLogFile.println("utc,lat,lon,alt_m,speed_kmph,course_deg,hdop,sats_used");

  sdReady = (bool)nmeaLogFile && (bool)trackLogFile;
  Serial.printf("SD logging %s -- %s / %s\n", sdReady ? "enabled" : "FAILED to open log files", nmeaPath, trackPath);
}

void logToSD(const char *line) {
  if (!sdReady) return;
  nmeaLogFile.print(line);
  nmeaLogFile.print("\n");
  if (++sdFlushCounter >= 20) { // periodic flush balances durability vs. flash wear
    nmeaLogFile.flush();
    sdFlushCounter = 0;
  }
}

void writeTrackRow(const char *row) {
  if (!sdReady) return;
  trackLogFile.print(row);
  trackLogFile.flush(); // one row per fix -- safe to flush every write
}
