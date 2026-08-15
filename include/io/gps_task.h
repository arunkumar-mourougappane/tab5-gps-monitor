#pragma once
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// -------------------------------------------------------------- GPS task --

extern HardwareSerial GPSSerial;
extern TinyGPSPlus gps;

// Configures GPSSerial and spawns gpsTaskFn pinned to the core opposite the
// Arduino loop task. Call once from setup(), after GPSSerial.begin().
void startGpsTask();
