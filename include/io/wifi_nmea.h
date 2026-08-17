#pragma once
#include "core/config.h"

#if ENABLE_WIFI_NMEA

// -------------------------------------------------------------- WiFi NMEA --
// Raw NMEA is broadcast over a small TCP server (port 10110, the de facto
// standard NMEA-over-TCP port used by tools like OpenCPN) so the stream can
// be viewed from a laptop without a serial cable. gpsTask never talks to the
// network directly -- it only pushes lines into a queue (via enqueueNmea()),
// so a stalled TCP client can never block UART draining.

extern volatile int nmeaClientCount;

// Cleared when the device sleeps so the radio can be powered down; the task
// owns the actual bring-up/tear-down so the UI thread never blocks on WiFi.
extern volatile bool wifiEnabled;

// The AP overlay's manual toggle, independent of sleep -- see
// applyWifiEnabled()'s comment for how the two combine.
extern volatile bool wifiUserDisabled;

// wifiEnabled = !asleep && !wifiUserDisabled. Note backlightOff deliberately
// stays out of this: enterBacklightOff() has never torn the AP down, only
// full enterSleep() does, and this preserves that distinction. Call after
// changing either input -- power.cpp calls it from enterSleep()/
// wakeDisplay(), and the overlay's toggle chip calls it after flipping
// wifiUserDisabled -- so wifiEnabled is never written directly from two
// places with two different ideas of what it should be.
void applyWifiEnabled();

// Exposed for the AP QR overlay (ui/ui_wifi_panel.cpp), which needs to show
// what it's advertising -- these stay the single source of truth used to
// actually bring the AP up in wifiTaskFn().
extern const char WIFI_AP_SSID[];
extern const char WIFI_AP_PASS[];
static constexpr uint16_t NMEA_TCP_PORT = 10110; // conventional NMEA-over-TCP port

void initWifiQueue();     // call once from setup(), before startWifiTask()
void startWifiTask();     // spawns wifiTaskFn

// Non-blocking: drops the line rather than ever stalling the caller.
void enqueueNmea(const char *line);

#endif // ENABLE_WIFI_NMEA
