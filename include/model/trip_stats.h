#pragma once
#include <cstdint>

// -------------------------------------------------------------- trip stats --
// Accumulated once per fix epoch inside gpsTask (under stateMutex), copied
// into the RenderSnapshot like everything else.

extern double tripDistanceKm;
extern double maxSpeedKmph;
extern bool haveLastFix;
extern double lastFixLat, lastFixLon;
extern uint32_t timeToFirstFixMs; // ms from task start to first fix, 0 = none yet
extern uint32_t firstFixAbsMs;    // absolute millis() at first fix, 0 = none yet

double haversineKm(double lat1, double lon1, double lat2, double lon2);

static constexpr int SPARK_LEN = 40;
extern float hdopHistory[SPARK_LEN];
extern int hdopHistCount;
extern uint32_t hdopHistHead;
void pushHdopHistory(float h);

// Session history for the trip view. The HDOP trend moved to the accuracy
// block on the live view, where it belongs next to the number it qualifies --
// so the trip face carries what only a session can answer instead: how the
// speed went, and how much of the trip actually had a usable fix.
extern float speedHistory[SPARK_LEN];
extern int speedHistCount;
extern uint32_t speedHistHead;
void pushSpeedHistory(float kmph);

// Seconds spent at each fix mode, indexed by GSA fix type - 1: no fix, 2D, 3D.
// One fix epoch is one second of receiver time, so these are counts of epochs.
extern uint32_t fixSecs[3];
