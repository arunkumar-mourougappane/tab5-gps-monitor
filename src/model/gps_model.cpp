#include <Arduino.h> // millis()
#include <cstring>
#include "model/gps_model.h"
#include "core/theme.h"

SatInfo satTable[MAX_SATS];

int gsaFixType = 1;
float gsaPDOP = 0, gsaVDOP = 0;

void upsertSat(const char *talker, int prn, int elevation, int azimuth, int snr) {
  int freeSlot = -1;
  int oldestSlot = 0;
  uint32_t oldestMs = UINT32_MAX;

  for (int i = 0; i < MAX_SATS; i++) {
    if (satTable[i].used) {
      if (satTable[i].prn == prn && strcmp(satTable[i].talker, talker) == 0) {
        satTable[i].elevation = elevation;
        satTable[i].azimuth = azimuth;
        satTable[i].snr = snr;
        satTable[i].lastSeenMs = millis();
        return;
      }
      if (satTable[i].lastSeenMs < oldestMs) {
        oldestMs = satTable[i].lastSeenMs;
        oldestSlot = i;
      }
    } else if (freeSlot < 0) {
      freeSlot = i;
    }
  }

  // Falling back to the genuinely least-recently-seen entry, not slot 0: always
  // reusing slot 0 made every unmatched satellite overwrite the same entry, so
  // that one dot teleported around the plot on each GSV burst.
  SatInfo &dst = satTable[freeSlot >= 0 ? freeSlot : oldestSlot];
  strlcpy(dst.talker, talker, sizeof(dst.talker));
  dst.prn = prn;
  dst.elevation = elevation;
  dst.azimuth = azimuth;
  dst.snr = snr;
  dst.lastSeenMs = millis();
  dst.used = true;
}

void pruneStaleSats() {
  uint32_t now = millis();
  for (auto &s : satTable) {
    if (s.used && now - s.lastSeenMs > 6000) s.used = false;
  }
}

int countVisibleSats(const SatInfo arr[MAX_SATS]) {
  int n = 0;
  for (int i = 0; i < MAX_SATS; i++) n += arr[i].used ? 1 : 0;
  return n;
}

// Still strongest-first (the list truncates, so the strongest are the ones
// worth showing), but the SNR is bucketed before comparing and ties break on
// constellation/PRN. Raw SNR jitters a couple of dB between updates, which
// made rows swap places on nearly every refresh; bucketing absorbs that, and
// the deterministic tie-break stops equal-signal rows from reordering as table
// slots get reused.
static int satSortBucket(const SatInfo &s) { return s.snr < 0 ? -1 : s.snr / 3; }

static bool satSortsBefore(const SatInfo &a, const SatInfo &b) {
  int ba = satSortBucket(a), bb = satSortBucket(b);
  if (ba != bb) return ba > bb;
  int c = strcmp(a.talker, b.talker);
  if (c != 0) return c < 0;
  return a.prn < b.prn;
}

int sortedSatOrder(const SatInfo arr[MAX_SATS], int order[MAX_SATS]) {
  int countUsed = 0;
  for (int i = 0; i < MAX_SATS; i++) {
    if (arr[i].used) order[countUsed++] = i;
  }
  for (int i = 1; i < countUsed; i++) {
    int key = order[i], j = i - 1;
    while (j >= 0 && satSortsBefore(arr[key], arr[order[j]])) {
      order[j + 1] = order[j];
      j--;
    }
    order[j + 1] = key;
  }
  return countUsed;
}

uint16_t snrColor(int snr) {
  return snr < 0 ? COLOR_STATUS_NONE : snr < 20 ? COLOR_STATUS_BAD : snr < 35 ? COLOR_STATUS_WARN : COLOR_STATUS_GOOD;
}

const char *hdopQuality(double h, uint16_t &color) {
  if (h < 1.0) { color = COLOR_STATUS_GOOD; return "EXCELLENT"; }
  if (h < 2.5) { color = COLOR_STATUS_GOOD; return "GOOD"; }
  if (h < 5.0) { color = COLOR_STATUS_WARN; return "FAIR"; }
  color = COLOR_STATUS_BAD;
  return "POOR";
}
