#include <TinyGPSPlus.h>
#include <cstring>
#include "render_snapshot.h"

SemaphoreHandle_t stateMutex;

// Owned by gps_task.cpp once that module is extracted; forward-declared here
// in the meantime -- the object itself still lives in main.cpp.
extern TinyGPSPlus gps;

void captureSnapshot(RenderSnapshot &s) {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  s.locValid = gps.location.isValid();
  s.lat = gps.location.lat();
  s.lon = gps.location.lng();
  s.altValid = gps.altitude.isValid();
  s.alt = gps.altitude.meters();
  s.spdValid = gps.speed.isValid();
  s.spd = gps.speed.kmph();
  s.crsValid = gps.course.isValid();
  s.crs = gps.course.deg();
  s.dateValid = gps.date.isValid();
  s.timeValid = gps.time.isValid();
  s.year = gps.date.year();
  s.month = gps.date.month();
  s.day = gps.date.day();
  s.hour = gps.time.hour();
  s.minute = gps.time.minute();
  s.second = gps.time.second();
  s.satsValid = gps.satellites.isValid();
  s.satsUsed = gps.satellites.value();
  s.hdopValid = gps.hdop.isValid();
  s.hdop = gps.hdop.hdop();
  s.fixType = gsaFixType;
  s.pdop = gsaPDOP;
  s.vdop = gsaVDOP;

  s.tripDistanceKm = tripDistanceKm;
  s.maxSpeedKmph = maxSpeedKmph;
  s.timeToFirstFixMs = timeToFirstFixMs;
  s.firstFixAbsMs = firstFixAbsMs;
  memcpy(s.hdopHistory, hdopHistory, sizeof(hdopHistory));
  s.hdopHistCount = hdopHistCount;
  s.hdopHistHead = hdopHistHead;
  memcpy(s.speedHistory, speedHistory, sizeof(speedHistory));
  s.speedHistCount = speedHistCount;
  s.speedHistHead = speedHistHead;
  memcpy(s.fixSecs, fixSecs, sizeof(fixSecs));

  memcpy(s.sats, satTable, sizeof(satTable)); // POD: no per-element String copies
  s.visibleSats = countVisibleSats(s.sats);

  memcpy(s.logLines, rawLog, sizeof(rawLog)); // one blockcopy, no per-line allocation
  s.logHead = rawLogHead;
  s.lastSentenceMs = lastSentenceMs;
  memcpy(s.typeCounts, sentenceTypeCounts, sizeof(sentenceTypeCounts));
  xSemaphoreGive(stateMutex);
}
