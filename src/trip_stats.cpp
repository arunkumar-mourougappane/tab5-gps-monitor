#include <Arduino.h> // radians()
#include <math.h>
#include "trip_stats.h"

double tripDistanceKm = 0;
double maxSpeedKmph = 0;
bool haveLastFix = false;
double lastFixLat = 0, lastFixLon = 0;
uint32_t timeToFirstFixMs = 0;
uint32_t firstFixAbsMs = 0;

double haversineKm(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kEarthRadiusKm = 6371.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return kEarthRadiusKm * 2 * atan2(sqrt(a), sqrt(1 - a));
}

float hdopHistory[SPARK_LEN] = {0};
int hdopHistCount = 0;
uint32_t hdopHistHead = 0;

void pushHdopHistory(float h) {
  hdopHistory[hdopHistHead % SPARK_LEN] = h;
  hdopHistHead++;
  if (hdopHistCount < SPARK_LEN) hdopHistCount++;
}

float speedHistory[SPARK_LEN] = {0};
int speedHistCount = 0;
uint32_t speedHistHead = 0;

void pushSpeedHistory(float kmph) {
  speedHistory[speedHistHead % SPARK_LEN] = kmph;
  speedHistHead++;
  if (speedHistCount < SPARK_LEN) speedHistCount++;
}

uint32_t fixSecs[3] = {0, 0, 0};
