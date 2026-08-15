#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "gps_model.h"
#include "trip_stats.h"
#include "nmea_parser.h"

// ---------------------------------------------------------- render snapshot --
// Captured under stateMutex, then rendered unlocked so slow display I/O never
// blocks gpsTask from draining the UART.

// Guards every piece of state gpsTask writes and the render side reads:
// TinyGPSPlus's `gps`, the satellite table, trip stats/history, gsaFixType/
// PDOP/VDOP, and the raw NMEA log ring. Defined here since captureSnapshot()
// is exactly the boundary this mutex protects; gpsTask and the touch handler
// (for the filter-cycle ring clear) take it directly.
extern SemaphoreHandle_t stateMutex;

struct RenderSnapshot {
  bool locValid; double lat, lon;
  bool altValid; double alt;
  bool spdValid; double spd;
  bool crsValid; double crs;
  bool dateValid, timeValid;
  int year, month, day, hour, minute, second;
  bool satsValid; int satsUsed;
  bool hdopValid; double hdop;
  int fixType;
  float pdop, vdop;
  int visibleSats;

  double tripDistanceKm;
  double maxSpeedKmph;
  uint32_t timeToFirstFixMs;
  uint32_t firstFixAbsMs;
  float hdopHistory[SPARK_LEN];
  int hdopHistCount;
  uint32_t hdopHistHead;
  float speedHistory[SPARK_LEN];
  int speedHistCount;
  uint32_t speedHistHead;
  uint32_t fixSecs[3];

  SatInfo sats[MAX_SATS];

  char logLines[LOG_LINES][RAW_LINE_MAX];
  uint32_t logHead;
  uint32_t lastSentenceMs;
  uint32_t typeCounts[NUM_SENTENCE_TYPES];
};

void captureSnapshot(RenderSnapshot &s);
