#pragma once
#include <cstdint>

// ------------------------------------------------------------ satellites --
// satTable/gsaFixType/gsaPDOP/gsaVDOP are written only by gpsTask (under
// stateMutex) and only ever read by the render side via a RenderSnapshot copy.

// talker is a fixed 2-char NMEA id ("GP"/"BD"/"GL"/"GA"/...) rather than a
// String, which keeps SatInfo a POD: the whole table is copied into every
// render snapshot, so this turns MAX_SATS String copies per poll into one
// memcpy with no heap traffic.
struct SatInfo {
  char talker[3];
  int prn;
  int elevation;
  int azimuth;
  int snr; // -1 = in view but not tracked
  uint32_t lastSeenMs;
  bool used;
};

// Sized for simultaneous multi-constellation tracking. This receiver reports
// GPS + BeiDou + GLONASS + Galileo at once, and the combined satellites-in-view
// count regularly runs 30-45 -- at the previous size of 32 the table filled and
// satellites fought over slots, making their dots jump around the sky plot.
static constexpr int MAX_SATS = 64;
extern SatInfo satTable[MAX_SATS];

extern int gsaFixType; // 1=no fix, 2=2D, 3=3D
extern float gsaPDOP, gsaVDOP;

void upsertSat(const char *talker, int prn, int elevation, int azimuth, int snr);
void pruneStaleSats();
int countVisibleSats(const SatInfo arr[MAX_SATS]);

// Strongest-first (the list truncates, so the strongest are the ones worth
// showing); ties broken deterministically on constellation/PRN so equal-signal
// rows don't reorder as table slots get reused. Returns the count placed into
// `order`.
int sortedSatOrder(const SatInfo arr[MAX_SATS], int order[MAX_SATS]);

uint16_t snrColor(int snr);
const char *hdopQuality(double h, uint16_t &color);
