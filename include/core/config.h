#pragma once
#include <cstdint>

// Bumped by scripts/bump-version.sh as part of cutting a release, alongside
// RELEASE_NOTES.md and before tagging -- kept in sync with the git tag by
// convention (there's no automated check tying the two together), so a
// release isn't done until both agree.
static constexpr char FIRMWARE_VERSION[] = "1.1.0";

static constexpr int GPS_RX_PIN = 54; // Tab5 RX <- GPS TX (White)
static constexpr int GPS_TX_PIN = 53; // Tab5 TX -> GPS RX (Yellow)
static constexpr uint32_t GPS_BAUD = 115200; // GPS/BDS Unit v1.1 default

// ESP32-P4+C6 WiFi (ESP-Hosted transport) is a newer code path; pioarduino's
// Tab5 board JSON is pinned to "esp32p4_es" (engineering-sample) silicon, and
// a documented Arduino-core issue around P4 chip-revision handling (fixed
// upstream after this platform version was cut) has caused boot crashes tied
// to hosted WiFi init on some physical units. If the board won't boot after
// flashing, flip this to 0 first to isolate whether WiFi is the cause.
#define ENABLE_WIFI_NMEA 1
