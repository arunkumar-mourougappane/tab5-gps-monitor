// Reads NMEA sentences from an M5Stack GPS/BDS Unit v1.1 (AT6668/ATGM336H)
// connected to Tab5's Port A (Grove), logs the decoded fix to USB serial, and
// renders a touch-interactive dashboard (position/trip card, satellite sky
// plot + signal list, raw NMEA log) on the Tab5's screen.
//
// Port A is a HY2.0-4P Grove connector: Black=GND, Red=5V, Yellow=G53, White=G54.
// By Grove/M5Stack convention Yellow carries the unit's RX line and White its TX
// line, so from the Tab5's side: G53 transmits to the GPS, G54 receives from it.
// This wasn't confirmed in M5Stack's Tab5 pinout doc -- swap GPS_RX_PIN/GPS_TX_PIN
// below if no sentences show up.
//
// TinyGPSPlus only parses GGA/RMC-family fields (location, altitude, speed,
// course, date/time, satellite count, HDOP). The satellite table (per-PRN
// elevation/azimuth/SNR) comes from GSV, and the 2D/3D fix mode + PDOP/VDOP
// come from GSA -- neither is exposed by TinyGPSPlus, so both are parsed here
// directly off the raw sentence stream.
//
// GPS ingestion runs on its own FreeRTOS task, pinned to the core opposite
// the Arduino loop task, so a slow display redraw can never stall draining
// the UART. The default RX ring buffer (256B) only absorbs ~22ms of silence
// at 115200 baud -- comfortably less than a full-panel redraw was taking in
// a single-loop design, which is the most likely cause of dropped/garbled
// sentences ("choppy" updates). Shared state is protected by a mutex; the
// render side only holds it long enough to copy a snapshot, then draws
// unlocked so the GPS task is never blocked by slow screen I/O.
//
// Touch: tap the position card to flip between the live readout and a trip
// summary (distance/max speed/HDOP trend); tap a satellite dot in the sky
// plot for its detail; tap the NMEA card to expand it full-height. Touch
// state is only ever read/written from loop() (the render side), so it
// needs no locking against gpsTask.

#include <M5Unified.h>
// Not reached by the normal M5GFX include chain; needed for the DCS brightness
// workaround in panelSetBrightness() below.
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>
#include <TinyGPSPlus.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <SD_MMC.h>
#include "config.h"
#include "display.h"

#if ENABLE_WIFI_NMEA
#include <WiFi.h>
#endif

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// Definition matching the extern in display.h.
M5Canvas canvas(&M5.Display);

// Pre-rendered sky-plot background (rings + N/S/E/W labels): built once in
// setup() and blitted each tick instead of redrawing a filled circle, three
// ring outlines, two crosshair lines, and four text labels from scratch --
// that redraw was one of the most expensive things happening every 200ms.
M5Canvas radarBg(&canvas);
static int radarBgSize = 0;

#include "theme.h"
#include "layout.h"

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
static SatInfo satTable[MAX_SATS];

static int gsaFixType = 1; // 1=no fix, 2=2D, 3=3D
static float gsaPDOP = 0, gsaVDOP = 0;

static void upsertSat(const char *talker, int prn, int elevation, int azimuth, int snr) {
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

static void pruneStaleSats() {
  uint32_t now = millis();
  for (auto &s : satTable) {
    if (s.used && now - s.lastSeenMs > 6000) s.used = false;
  }
}

static int countVisibleSats(const SatInfo arr[MAX_SATS]) {
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

static int sortedSatOrder(const SatInfo arr[MAX_SATS], int order[MAX_SATS]) {
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

static uint16_t snrColor(int snr) {
  return snr < 0 ? COLOR_STATUS_NONE : snr < 20 ? COLOR_STATUS_BAD : snr < 35 ? COLOR_STATUS_WARN : COLOR_STATUS_GOOD;
}

static const char *hdopQuality(double h, uint16_t &color) {
  if (h < 1.0) { color = COLOR_STATUS_GOOD; return "EXCELLENT"; }
  if (h < 2.5) { color = COLOR_STATUS_GOOD; return "GOOD"; }
  if (h < 5.0) { color = COLOR_STATUS_WARN; return "FAIR"; }
  color = COLOR_STATUS_BAD;
  return "POOR";
}

// -------------------------------------------------------------- trip stats --
// Accumulated once per GPS burst inside gpsTask (under stateMutex), copied
// into the RenderSnapshot like everything else.

static double tripDistanceKm = 0;
static double maxSpeedKmph = 0;
static bool haveLastFix = false;
static double lastFixLat = 0, lastFixLon = 0;
static uint32_t timeToFirstFixMs = 0; // ms from task start to first fix, 0 = none yet
static uint32_t firstFixAbsMs = 0;    // absolute millis() at first fix, 0 = none yet

static double haversineKm(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kEarthRadiusKm = 6371.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return kEarthRadiusKm * 2 * atan2(sqrt(a), sqrt(1 - a));
}

static constexpr int SPARK_LEN = 40;
static float hdopHistory[SPARK_LEN] = {0};
static int hdopHistCount = 0;
static uint32_t hdopHistHead = 0;

static void pushHdopHistory(float h) {
  hdopHistory[hdopHistHead % SPARK_LEN] = h;
  hdopHistHead++;
  if (hdopHistCount < SPARK_LEN) hdopHistCount++;
}

// Session history for the trip view. The HDOP trend moved to the accuracy
// block on the live view, where it belongs next to the number it qualifies --
// so the trip face carries what only a session can answer instead: how the
// speed went, and how much of the trip actually had a usable fix.
static float speedHistory[SPARK_LEN] = {0};
static int speedHistCount = 0;
static uint32_t speedHistHead = 0;

static void pushSpeedHistory(float kmph) {
  speedHistory[speedHistHead % SPARK_LEN] = kmph;
  speedHistHead++;
  if (speedHistCount < SPARK_LEN) speedHistCount++;
}

// Seconds spent at each fix mode, indexed by GSA fix type - 1: no fix, 2D, 3D.
// One fix epoch is one second of receiver time, so these are counts of epochs.
static uint32_t fixSecs[3] = {0, 0, 0};

// -------------------------------------------------- SD logging / WiFi NMEA --
// SD_MMC needs no manual pin config: the m5stack_tab5 board variant defines
// BOARD_HAS_SDMMC/BOARD_SDMMC_SLOT, so SD_MMC.begin() picks them up itself.
// Raw NMEA is broadcast over a small TCP server (port 10110, the de facto
// standard NMEA-over-TCP port used by tools like OpenCPN) so the stream can
// be viewed from a laptop without a serial cable. gpsTask never talks to the
// network directly -- it only pushes lines into a queue, so a stalled TCP
// client can never block UART draining.

static bool sdReady = false;
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

static void initSdLogging() {
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

#if ENABLE_WIFI_NMEA
static constexpr char WIFI_AP_SSID[] = "Tab5-GPS";
static constexpr char WIFI_AP_PASS[] = "gpstest123"; // WPA2 requires >=8 chars
static constexpr uint16_t NMEA_TCP_PORT = 10110; // conventional NMEA-over-TCP port

struct NmeaQueueMsg { char text[96]; };
static QueueHandle_t nmeaQueue;
static WiFiServer nmeaServer(NMEA_TCP_PORT);
static WiFiClient nmeaClients[4];
static volatile int nmeaClientCount = 0;

// Cleared when the device sleeps so the radio can be powered down; the task
// owns the actual bring-up/tear-down so the UI thread never blocks on WiFi.
static volatile bool wifiEnabled = true;

static void wifiTaskFn(void *) {
  bool apUp = false;

  for (;;) {
    if (wifiEnabled && !apUp) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
      nmeaServer.begin();
      apUp = true;
      Serial.printf("WiFi AP '%s' up -- connect to %s:%u for raw NMEA (TCP)\n", WIFI_AP_SSID,
                    WiFi.softAPIP().toString().c_str(), NMEA_TCP_PORT);
    } else if (!wifiEnabled && apUp) {
      for (auto &c : nmeaClients) {
        if (c.connected()) c.stop();
      }
      nmeaServer.end();
      WiFi.softAPdisconnect(true); // true also powers the radio down
      WiFi.mode(WIFI_OFF);
      nmeaClientCount = 0;
      apUp = false;
      Serial.println("WiFi AP down (device asleep)");
    }

    if (apUp) {
      if (nmeaServer.hasClient()) {
        WiFiClient newClient = nmeaServer.accept();
        bool placed = false;
        for (auto &c : nmeaClients) {
          if (!c.connected()) {
            c = newClient;
            placed = true;
            break;
          }
        }
        if (!placed) newClient.stop(); // pool full
      }

      NmeaQueueMsg msg;
      while (xQueueReceive(nmeaQueue, &msg, 0) == pdTRUE) {
        for (auto &c : nmeaClients) {
          if (c.connected()) {
            c.print(msg.text);
            c.print("\r\n");
          }
        }
      }

      int cnt = 0;
      for (auto &c : nmeaClients) if (c.connected()) cnt++;
      nmeaClientCount = cnt;
    } else {
      // Keep draining while the radio is down, otherwise the queue sits full
      // and gpsTask's non-blocking sends all fail until wake.
      NmeaQueueMsg msg;
      while (xQueueReceive(nmeaQueue, &msg, 0) == pdTRUE) {
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
#endif

// ------------------------------------------------------- NMEA line parsing --
// All of this only ever runs inside gpsTask, holding stateMutex.

// Splits in place: each comma becomes a NUL and `out` points into `s`, so a
// sentence costs no allocations at all. The previous String-based version
// heap-allocated one substring per field -- up to 24 per sentence, ~50
// sentences a second, every one of them inside stateMutex.
static int splitCSV(char *s, char *out[], int maxTokens) {
  int count = 0;
  char *p = s;
  while (count < maxTokens) {
    out[count++] = p;
    char *comma = strchr(p, ',');
    if (comma == nullptr) break;
    *comma = '\0';
    p = comma + 1;
  }
  return count;
}

static void parseGSV(const char *talker, char *tokens[], int n) {
  // $xxGSV,numMsgs,msgNum,numSatsInView,[prn,elev,az,snr]x4...*CS
  int idx = 4;
  while (idx + 3 < n) {
    if (tokens[idx][0] != '\0') {
      int prn = atoi(tokens[idx]);
      int elev = tokens[idx + 1][0] ? atoi(tokens[idx + 1]) : -1;
      int az = tokens[idx + 2][0] ? atoi(tokens[idx + 2]) : -1;
      int snr = tokens[idx + 3][0] ? atoi(tokens[idx + 3]) : -1;
      upsertSat(talker, prn, elev, az, snr);
    }
    idx += 4;
  }
}

static void parseGSA(char *tokens[], int n) {
  // $xxGSA,mode,fixType,prn1..prn12,PDOP,HDOP,VDOP*CS
  if (n < 18) return;
  gsaFixType = tokens[2][0] ? atoi(tokens[2]) : 1;
  gsaPDOP = atof(tokens[15]);
  gsaVDOP = atof(tokens[17]);
}

static constexpr int LOG_LINES = 40;
// Longest legal NMEA sentence is 82 bytes including CRLF; the assembler caps a
// line at 100 like the String version did, so 104 leaves room for the NUL.
static constexpr int RAW_LINE_MAX = 104;
static char rawLog[LOG_LINES][RAW_LINE_MAX];
static uint32_t rawLogHead = 0;
static uint32_t lastSentenceMs = 0;

static constexpr int NUM_SENTENCE_TYPES = 9;
static const char *SENTENCE_TYPES[NUM_SENTENCE_TYPES] = {"GGA", "RMC", "GSV", "GSA", "GLL", "VTG", "ZDA", "TXT", "OTH"};
static uint32_t sentenceTypeCounts[NUM_SENTENCE_TYPES] = {0};

// On-screen sentence filter. GSV is the bulk of the traffic (one message per
// four satellites, per constellation, every second), so hiding it turns the
// log from a flood into something readable.
//
// This is a *view* control only: SD logging, the TCP stream, and the GSV/GSA
// parsing that feeds the sky plot all continue to see every sentence.
// Mutated from loop() under stateMutex; read by gpsTask under the same lock.
enum class NmeaFilter : uint8_t { ALL, NO_GSV, POSITION };
static constexpr int NMEA_FILTER_COUNT = 3;
static uint8_t nmeaFilter = (uint8_t)NmeaFilter::ALL;

static const char *nmeaFilterLabel() {
  switch ((NmeaFilter)nmeaFilter) {
    case NmeaFilter::NO_GSV: return "NO GSV";
    case NmeaFilter::POSITION: return "POS";
    default: return "ALL";
  }
}

static bool sentencePassesFilter(const char *type) {
  switch ((NmeaFilter)nmeaFilter) {
    case NmeaFilter::NO_GSV: return strcmp(type, "GSV") != 0;
    case NmeaFilter::POSITION: return strcmp(type, "GGA") == 0 || strcmp(type, "RMC") == 0;
    default: return true;
  }
}

// Fixed size so the hit target matches the drawn chip regardless of label, and
// so the chip can't resize under a changing label the way the status badges did.
static constexpr int FILTER_CHIP_W = 92;
static constexpr int FILTER_CHIP_H = 26;

static Rect nmeaFilterChipRect() {
  return {logCard.x + logCard.w - 18 - FILTER_CHIP_W, logCard.y + (CARD_HEADER_H - FILTER_CHIP_H) / 2,
          FILTER_CHIP_W, FILTER_CHIP_H};
}

// The chip is only 26px tall, well under a fingertip. Its target is padded to
// 44 and reaches slightly into the card body; that region belongs to the log
// card, which is why the filter is hit-tested before it -- a near miss used to
// expand the whole card instead, which is a worse outcome than doing nothing.
static Rect nmeaFilterHitRect() {
  Rect c = nmeaFilterChipRect();
  return {c.x - 16, c.y - 5, c.w + 24, c.h + 18};
}

// The two sinks that only gpsTask ever touches: the SD log and the TCP fan-out
// queue. Split out of handleRawSentence() so the caller can run them without
// holding stateMutex -- an SD write is the slowest thing in the ingest path,
// and nothing on the render side can see either sink.
static void logSentence(const char *line) {
  if (sdReady) {
    nmeaLogFile.print(line);
    nmeaLogFile.print("\n");
    if (++sdFlushCounter >= 20) { // periodic flush balances durability vs. flash wear
      nmeaLogFile.flush();
      sdFlushCounter = 0;
    }
  }

#if ENABLE_WIFI_NMEA
  if (nmeaQueue) {
    NmeaQueueMsg msg;
    strlcpy(msg.text, line, sizeof(msg.text));
    xQueueSend(nmeaQueue, &msg, 0); // non-blocking: drop rather than ever stall gpsTask
  }
#endif
}

// Everything here touches state the render task reads, so it runs under
// stateMutex -- and now nothing else does.
static void handleRawSentence(const char *line, int len) {
  lastSentenceMs = millis();

  // The body up to '*' is what gets tokenised. Copied rather than tokenised in
  // place because splitCSV() writes NULs over the commas, and the caller's
  // buffer has already been handed to SD and the TCP queue as one string.
  char core[RAW_LINE_MAX];
  int coreLen = len;
  const char *star = strchr(line, '*');
  if (star != nullptr) coreLen = (int)(star - line);
  if (coreLen >= RAW_LINE_MAX) coreLen = RAW_LINE_MAX - 1;
  memcpy(core, line, coreLen);
  core[coreLen] = '\0';

  bool wellFormed = coreLen >= 6 && core[0] == '$';

  char talker[3] = {0};
  char type[4] = {0};
  if (wellFormed) {
    memcpy(talker, core + 1, 2);
    memcpy(type, core + 3, 3);

    // Counted before filtering: these stay a record of what the receiver
    // actually emits, not of what the log happens to be showing.
    bool matched = false;
    for (int i = 0; i < NUM_SENTENCE_TYPES - 1; i++) {
      if (strcmp(type, SENTENCE_TYPES[i]) == 0) {
        sentenceTypeCounts[i]++;
        matched = true;
        break;
      }
    }
    if (!matched) sentenceTypeCounts[NUM_SENTENCE_TYPES - 1]++;
  }

  // Filter on the way *in* so the ring holds LOG_LINES matching sentences.
  // Filtering at draw time instead would leave only the couple of position
  // sentences that survive within the last LOG_LINES of GSV-dominated traffic.
  // Malformed lines are always kept -- they're the interesting ones.
  if (!wellFormed || sentencePassesFilter(type)) {
    strlcpy(rawLog[rawLogHead % LOG_LINES], line, RAW_LINE_MAX);
    rawLogHead++;
  }

  if (!wellFormed) return;

  // Parsing is deliberately outside the filter: the sky plot and fix mode must
  // keep updating no matter what the log view is set to show.
  char *tokens[24];
  int n = splitCSV(core, tokens, 24);

  if (strcmp(type, "GSV") == 0) {
    parseGSV(talker, tokens, n);
  } else if (strcmp(type, "GSA") == 0) {
    parseGSA(tokens, n);
  }
}

// Formats rather than prints: the caller holds stateMutex while filling the
// buffer (every field below is shared state) and writes it to the port after
// releasing, so a blocked USB CDC endpoint can't stall UART draining.
static void formatFix(char *out, size_t cap) {
  int n = snprintf(out, cap, "fix=%d sats=%d", (int)gps.location.isValid(),
                   gps.satellites.isValid() ? (int)gps.satellites.value() : 0);

  if (gps.location.isValid())
    n += snprintf(out + n, cap - n, " lat=%.6f lon=%.6f", gps.location.lat(), gps.location.lng());
  if (gps.altitude.isValid())
    n += snprintf(out + n, cap - n, " alt=%.1fm", gps.altitude.meters());
  if (gps.speed.isValid())
    n += snprintf(out + n, cap - n, " speed=%.1fkm/h", gps.speed.kmph());
  if (gps.date.isValid() && gps.time.isValid())
    snprintf(out + n, cap - n, " utc=%04d-%02d-%02d %02d:%02d:%02d", gps.date.year(), gps.date.month(),
             gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
}

// -------------------------------------------------------------- GPS task --

static SemaphoreHandle_t stateMutex;

static void gpsTaskFn(void *) {
  char lineBuf[RAW_LINE_MAX];
  int lineLen = 0;
  uint32_t taskStartMs = millis();
  uint32_t lastWarnMs = 0;
  uint32_t lastFixEpoch = 0;

  for (;;) {
    if (GPSSerial.available()) {
      bool updated = false;
      // Reading the port and assembling a line need no lock -- the UART and
      // lineBuf belong to this task. The lock is taken per completed sentence
      // instead of once around the whole drain, so the render task can
      // interleave between sentences rather than waiting out a full burst.
      while (GPSSerial.available()) {
        char c = GPSSerial.read();

        if (c == '$') {
          lineBuf[0] = '$';
          lineLen = 1;
        } else if (c == '\n') {
          if (lineLen > 6) {
            lineBuf[lineLen] = '\0';
            logSentence(lineBuf); // SD + TCP queue: gpsTask-only, so no lock

            xSemaphoreTake(stateMutex, portMAX_DELAY);
            // Feeding TinyGPSPlus the assembled sentence rather than the raw
            // byte stream: bytes outside a sentence carry no information it
            // can use, and a line long enough to have been truncated by the
            // assembler would fail its checksum either way.
            for (int k = 0; k < lineLen; k++) {
              if (gps.encode(lineBuf[k])) updated = true;
            }
            if (gps.encode('\r')) updated = true;
            if (gps.encode('\n')) updated = true;
            handleRawSentence(lineBuf, lineLen);
            xSemaphoreGive(stateMutex);
          }
          lineLen = 0;
        } else if (c != '\r' && lineLen > 0 && lineLen < RAW_LINE_MAX - 4) {
          lineBuf[lineLen++] = c;
        }
      }

      char trackRow[128] = {0};
      char fixLine[192] = {0};

      xSemaphoreTake(stateMutex, portMAX_DELAY);
      pruneStaleSats();

      // One epoch = one receiver fix. This block used to run once per drain
      // burst instead, and a burst is not a fix: the task wakes every 2ms, so
      // a second's worth of sentences arriving over ~50ms produces on the
      // order of 25 bursts. That meant ~25 track rows and ~25 SD flushes a
      // second rather than the one the CSV is meant to hold, and an HDOP
      // history spanning under two seconds rather than the last 40 fixes.
      //
      // The epoch key is the receiver's own UTC stamp, not TinyGPSPlus's
      // isUpdated(): reading a value clears that flag, and captureSnapshot()
      // reads every one of them from the render task five times a second, so
      // the update flags are not reliably observable from here.
      uint32_t epoch = gps.time.isValid() ? gps.time.value() : (millis() / 1000);
      bool newEpoch = epoch != lastFixEpoch;
      if (newEpoch) lastFixEpoch = epoch;

      if (newEpoch && gps.location.isValid()) {
        double lat = gps.location.lat(), lon = gps.location.lng();
        if (haveLastFix) {
          double d = haversineKm(lastFixLat, lastFixLon, lat, lon);
          if (d < 1.0) tripDistanceKm += d; // guard against wild jumps from a bad fix
        } else {
          haveLastFix = true;
          timeToFirstFixMs = millis() - taskStartMs;
          firstFixAbsMs = millis();
        }
        lastFixLat = lat;
        lastFixLon = lon;

        // Formatted here (every field is shared state), written after the
        // release below.
        if (sdReady) {
          snprintf(trackRow, sizeof(trackRow), "%04d-%02d-%02dT%02d:%02d:%02dZ,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f,%d\n",
                   gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(),
                   gps.time.second(), lat, lon, gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
                   gps.speed.isValid() ? gps.speed.kmph() : 0.0, gps.course.isValid() ? gps.course.deg() : 0.0,
                   gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, gps.satellites.isValid() ? gps.satellites.value() : 0);
        }
      }
      if (newEpoch) {
        if (gps.speed.isValid() && gps.speed.kmph() > maxSpeedKmph) maxSpeedKmph = gps.speed.kmph();
        if (gps.hdop.isValid()) pushHdopHistory((float)gps.hdop.hdop());
        pushSpeedHistory(gps.speed.isValid() ? (float)gps.speed.kmph() : 0.0f);
        int fixIdx = constrain(gsaFixType - 1, 0, 2);
        fixSecs[fixIdx]++;
        // Also once per fix rather than once per parsed sentence.
        if (updated) formatFix(fixLine, sizeof(fixLine));
      }
      xSemaphoreGive(stateMutex);

      // Both of these are slow, and neither is shared -- so they happen with
      // the lock already released.
      if (trackRow[0] != '\0') {
        trackLogFile.print(trackRow);
        trackLogFile.flush(); // one row per fix -- safe to flush every write
      }
      if (fixLine[0] != '\0') Serial.println(fixLine);
    }

    uint32_t now = millis();
    if (now - taskStartMs > 15000 && now - lastWarnMs > 5000) {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      bool noData = gps.charsProcessed() < 10;
      xSemaphoreGive(stateMutex);
      if (noData) {
        Serial.println("WARNING: no GPS data received -- check wiring/baud rate");
      }
      lastWarnMs = now;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ---------------------------------------------------------- render snapshot --
// Captured under stateMutex, then rendered unlocked so slow display I/O never
// blocks gpsTask from draining the UART.

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

static void captureSnapshot(RenderSnapshot &s) {
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

// ---------------------------------------------------------- touch/UI state --
// Only ever touched from loop() (render side) -- no locking needed.

enum class PositionView : uint8_t { LIVE, TRIP };
static PositionView positionView = PositionView::LIVE;
// logExpanded is defined in layout.cpp (computeLayout() needs it); declared
// via layout.h's extern.

struct SkyDot { int x, y; SatInfo info; };
static SkyDot skyDots[MAX_SATS];
static int skyDotCount = 0;

struct SatTooltip { bool active = false; SatInfo info; uint32_t shownAtMs = 0; };
static SatTooltip satTooltip;

// Screen-dark states. Both are woken by a tap anywhere; the difference is that
// sleep also powers the radio down and puts the panel itself to sleep rather
// than only zeroing the backlight.
//
// Neither stops GPS ingestion or SD logging: for a logger, "screen off" means
// keep recording in your pocket, not stop working.
static bool backlightOff = false;
static bool asleep = false;
static uint8_t savedBrightness = 128;

// Which chip the finger is currently down on, so it can render pressed. Held
// across frames (not just on release) so the highlight tracks the finger.
enum class PressTarget : uint8_t { NONE, LIGHT, SLEEP, FILTER, DIM_OFF, DIM_DONE };
static PressTarget pressTarget = PressTarget::NONE;

// Which chip the press *started* on. A button fires on release of the press
// that began inside it, rather than on a fresh hit test of the lift position:
// re-testing the lift meant a finger that drifted a few pixels off the chip
// lit the highlight and then did nothing at all, which reads as a button that
// only works sometimes.
static PressTarget capturedTarget = PressTarget::NONE;

// Touch sampling. Hoisted out of loop() so it can also run between the slices
// of a screen push -- a full-canvas blit is the longest thing the render side
// does, and while it runs nothing polls the panel. A quick tap that started
// and ended inside one of those windows was simply never seen: the driver
// only reads the hardware when we call it, so there is no event left to
// recover afterwards.
//
// getDetail(i) resolves through _touch_raw[i].id, and _touch_raw is only
// refreshed for the points the current scan reports. On release the driver
// reports zero points, so those entries hold stale data from an earlier scan
// -- and unlike getTouchPointRaw(), getDetail() does not clamp the index
// against the live point count. Reading a second slot that way is therefore
// not deterministic: the stale id can alias back to slot 0 or point at an
// unrelated detail slot.
//
// Instead: sample coordinates only while a contact is definitely down (raw
// data valid then), and detect the lift ourselves as the absence of a press.
// That is unambiguous, fires exactly once per physical touch, and is immune
// both to the ghost second contact this panel emits and to the state machine
// escalating a tap to hold/flick/drag.
static bool touchWasDown = false;
static bool touchPressing = false;
static bool touchPressEdge = false;   // consumed by loop()
static bool touchReleaseEdge = false; // consumed by loop()
static int touchLastX = -1, touchLastY = -1; // current contact, for drags
static int touchDownX = -1, touchDownY = -1; // where the press began, for hit tests

static void sampleTouch() {
  M5.update();
  auto touch = M5.Touch.getDetail(0);
  if (touch.isPressed()) {
    if (!touchWasDown) {
      touchPressEdge = true;
      touchDownX = touch.x;
      touchDownY = touch.y;
    }
    touchWasDown = true;
    touchPressing = true;
    touchLastX = touch.x;
    touchLastY = touch.y;
  } else {
    touchPressing = false;
    if (touchWasDown) {
      touchWasDown = false;
      touchReleaseEdge = true;
    }
  }
}

// Brightness overlay, opened from the LIGHT chip.
static bool dimmerOpen = false;
static bool dimmerDirty = false;
static bool dimmerDragging = false; // slider grabbed, follows X until the lift
static constexpr uint8_t BRIGHTNESS_MIN = 8; // slider floor: never fully dark
static constexpr uint8_t BRIGHTNESS_MAX = 255;

// Finger-sized hit target. The dots are drawn at only 4-9px radius, but a
// fingertip covers ~40px on this panel -- matching the visual size made them
// effectively untappable, and a miss silently does nothing because the sky
// area isn't covered by any other tap target.
//
// Picks the *nearest* dot rather than the first in array order that happens
// to qualify: satellites cluster near the zenith, and since the tooltip
// reports one specific satellite's details, grabbing an arbitrary neighbour
// would show data for a satellite the user didn't aim at.
static constexpr int SKY_DOT_HIT_RADIUS = 34;

static int hitTestSkyDot(int px, int py) {
  int best = -1;
  int bestDistSq = SKY_DOT_HIT_RADIUS * SKY_DOT_HIT_RADIUS;
  for (int i = 0; i < skyDotCount; i++) {
    int dx = px - skyDots[i].x, dy = py - skyDots[i].y;
    int distSq = dx * dx + dy * dy;
    if (distSq <= bestDistSq) {
      bestDistSq = distSq;
      best = i;
    }
  }
  return best;
}

// ------------------------------------------------------------- rendering --

// Draws label above value, auto-shrinking the value if it would overflow `w`.
// Returns the y position just below the drawn value.
static int drawHeroValue(int x, int y, int w, const char *label, const char *value, uint8_t bigSize = 2) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(label);
  y += d.fontHeight() + 4;

  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setTextSize(bigSize);
  if (d.textWidth(value) > w) d.setTextSize(1);
  d.setCursor(x, y);
  d.print(value);
  y += d.fontHeight() + 2;
  return y;
}

static void drawMiniStat(int x, int y, const char *label, const char *value) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(label);
  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(x, y + 20);
  d.print(value);
}

// Rounded pill with a leading text (no dot) -- used for fix/sat count badges.
static int drawBadge(int x, int y, const char *text, uint16_t bg, uint16_t fg) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  int h = 30;
  int w = d.textWidth(text) + 28;
  d.fillRoundRect(x, y, w, h, h / 2, bg);
  d.setTextColor(fg, bg);
  d.setCursor(x + 14, y + (h - d.fontHeight()) / 2);
  d.print(text);
  return w;
}

static void drawCardFrame(const Rect &r, const char *label, uint16_t accent) {
  auto &d = canvas;
  d.fillRoundRect(r.x, r.y, r.w, r.h, CARD_RADIUS, COLOR_CARD_BG);
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(r.x + 18, r.y + 11);
  d.print(label);
  d.drawFastHLine(r.x + 18, r.y + CARD_HEADER_H, r.w - 36, accent);
}

// Header text doubles as the touch-affordance hint, and only needs to be
// repainted when a view toggles -- so drawStaticChrome() is called again on
// every touch transition rather than every render tick.
void drawFilterChip(); // defined below; called from layout.cpp's relayout()

void drawStaticChrome() { // called from layout.cpp's relayout()
  auto &d = canvas;
  d.fillScreen(COLOR_BG);

  d.fillRect(0, 0, SCREEN_W, TOPBAR_H, COLOR_TOPBAR_BG);
  d.drawFastHLine(0, TOPBAR_H, SCREEN_W, COLOR_DIVIDER);
  d.setFont(&fonts::Font4);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_TOPBAR_BG);
  d.setCursor(MARGIN, (TOPBAR_H - d.fontHeight()) / 2);
  d.print("TAB5 GPS MONITOR");

  if (!logExpanded) {
    drawCardFrame(fixCard, positionView == PositionView::LIVE ? "POSITION  (tap: trip stats)" : "TRIP STATS  (tap: live)",
                  COLOR_ACCENT);
    drawCardFrame(skyCard, "SATELLITES  (tap a dot for detail)", COLOR_ACCENT);
  }
  drawCardFrame(logCard, logExpanded ? "NMEA STREAM  (tap to collapse)" : "NMEA STREAM  (tap to expand)",
                COLOR_ACCENT_GREEN);

  drawFilterChip(); // sits in the log card header, at its right end
}

// Shared top-bar badge geometry. Every pill is laid out as
//   [PAD_L][icon][ICON_GAP][text][PAD_R]
// and its width is always derived from the string that is actually drawn.
// Keeping these in one place is deliberate: the battery badge previously
// sized itself from "100%" while printing "100% CHG", so the label overran
// the pill and collided with the badge to its right.
//
// Sized against two hard limits. Vertically the bar is TOPBAR_H (56px), so a
// badge cannot exceed that minus breathing room at each edge; 36 leaves 10px
// above and below. Horizontally the row is right-aligned and chains leftward,
// and drawStatusPill() repaints the chips before the badges -- so a row long
// enough to reach the SLEEP chip's right edge (518) would paint over it.
//
// This is a deliberate midpoint: the pills were 32px and briefly 42px, which
// read as oversized on the panel.
static constexpr int BADGE_H = 36;
static constexpr int BADGE_PAD_L = 16;
static constexpr int BADGE_PAD_R = 16;
static constexpr int BADGE_ICON_GAP = 11;
static constexpr int BADGE_GAP = 11; // spacing between adjacent badges
static constexpr int BADGE_DOT_R = 6;
// The classic font ramp has no size between Font2 (16px) and Font4 (26px) --
// Font0 is 8px and the next step up is 26 -- so the badge text stays at Font2
// while the pill around it grew. Getting an intermediate size means bringing
// in a GFX face (FreeSans9pt7b, DejaVu18), which would put a second typeface
// in a bar that is otherwise all Font2/Font4.
//
// Single knob for the whole top bar: the badges and the LIGHT/SLEEP chips
// take their face from here. Everything else drawn through drawChip() keeps
// its own default -- the NMEA filter chip is only 26px tall.
#define BADGE_FONT (&fonts::Font2)

// Action chips live on the LEFT of the top bar, at fixed positions clear of the
// title. Deliberately not chained onto the right-hand badge row: those pills
// resize with their labels, which would shift these buttons' hit rects around
// under the user's finger.
static constexpr int TOPBAR_CTRL_X = 300;
static constexpr int TOPBAR_CTRL_W = 104;

static Rect lightBtnRect() {
  return {TOPBAR_CTRL_X, (TOPBAR_H - BADGE_H) / 2, TOPBAR_CTRL_W, BADGE_H};
}

static Rect sleepBtnRect() {
  return {TOPBAR_CTRL_X + TOPBAR_CTRL_W + BADGE_GAP, (TOPBAR_H - BADGE_H) / 2, TOPBAR_CTRL_W, BADGE_H};
}

// Touch targets, deliberately larger than the chips drawn inside them. A
// fingertip covers roughly 40px on this panel and these chips are 36 tall, so
// aiming at one and landing a few pixels high or low was a miss. Nothing else
// in the top bar is tappable, so both claim its full height; horizontally they
// take 5px of the 11px gap each, which keeps them from overlapping each other.
static Rect lightHitRect() {
  Rect c = lightBtnRect();
  return {c.x - 5, 0, c.w + 10, TOPBAR_H};
}

static Rect sleepHitRect() {
  Rect c = sleepBtnRect();
  return {c.x - 5, 0, c.w + 10, TOPBAR_H};
}

// Press feedback is a colour swap rather than an inset/shrink: an inset would
// leave a ring of the previous fill behind unless the surrounding background
// were also repainted, and that background differs per call site.
static void drawChip(const Rect &c, const char *label, uint16_t bg, uint16_t fg, bool pressed,
                     const lgfx::IFont *font = &fonts::Font2) {
  auto &d = canvas;
  uint16_t fill = pressed ? COLOR_ACCENT : bg;
  uint16_t text = pressed ? COLOR_BG : fg;
  d.fillRoundRect(c.x, c.y, c.w, c.h, c.h / 2, fill);
  d.setFont(font);
  d.setTextSize(1);
  d.setTextColor(text, fill);
  d.setCursor(c.x + (c.w - d.textWidth(label)) / 2, c.y + (c.h - d.fontHeight()) / 2);
  d.print(label);
}

// ------------------------------------------------ panel brightness via DCS --
// M5.Display.setBrightness() is a no-op on Tab5: Panel_Device::setBrightness()
// forwards to a Light instance and Tab5's init path never attaches one, while
// Panel_DSI/ST7123/ST7121 implement no brightness of their own and neither IO
// expander carries a backlight pin. The one remaining route is the panel's own
// DSI command channel: DCS 0x51 (set_display_brightness), which only takes
// effect once 0x53 (write_control_display) has enabled the brightness block.
//
// write_params() is protected, so it's reached through a derived type that adds
// no members and therefore shares the base layout -- the same reinterpret_cast
// idiom M5GFX itself uses in getPanel(). The type is never instantiated.
//
// SPECULATIVE: this only works if the ST7121/ST7123 actually drives the
// backlight. If the panel ignores 0x51 the level won't change, and only the
// blank-to-black path will be visible. Verify on hardware before relying on it.
struct DsiBrightnessAccess : public lgfx::Panel_DSI {
  bool writeDcs(uint32_t cmd, const uint8_t *data, size_t len) { return write_params(cmd, data, len); }
};

static bool panelSetBrightness(uint8_t level) {
  auto *p = M5.Display.getPanel();
  if (p == nullptr) return false;
  auto *acc = reinterpret_cast<DsiBrightnessAccess *>(p);
  uint8_t ctrl = 0x2C; // BCTRL | DD | BL -- enable the brightness control block
  acc->writeDcs(0x53, &ctrl, 1);
  return acc->writeDcs(0x51, &level, 1);
}

// ---------------------------------------------------------- dimmer overlay --

static Rect dimmerPanelRect() {
  constexpr int w = 560, h = 210;
  return {(SCREEN_W - w) / 2, (SCREEN_H - h) / 2, w, h};
}

static Rect dimmerTrackRect() {
  Rect p = dimmerPanelRect();
  return {p.x + 30, p.y + 82, p.w - 60, 26};
}

static Rect dimmerOffRect() {
  Rect p = dimmerPanelRect();
  return {p.x + 30, p.y + p.h - 62, 150, 44};
}

static Rect dimmerDoneRect() {
  Rect p = dimmerPanelRect();
  return {p.x + p.w - 30 - 150, p.y + p.h - 62, 150, 44};
}

// Grab area for the slider. The track is drawn 26px tall but the knob riding
// on it is 34, and a horizontal drag never stays inside a 26px band -- so the
// area that can start and hold a drag is padded well past both. It stops short
// of OFF/DONE at y+148 and of the percentage readout above.
static Rect dimmerTrackHitRect() {
  Rect t = dimmerTrackRect();
  return {t.x - 10, t.y - 18, t.w + 20, t.h + 36};
}

// True when the level actually moved, so a drag that hasn't crossed into the
// next step doesn't repaint the overlay or re-issue the panel command.
static bool setBrightnessFromTouch(int tx) {
  Rect t = dimmerTrackRect();
  int rel = constrain(tx - t.x, 0, t.w);
  uint8_t level = (uint8_t)(BRIGHTNESS_MIN + (long)(BRIGHTNESS_MAX - BRIGHTNESS_MIN) * rel / t.w);
  if (level == savedBrightness) return false;
  savedBrightness = level;
  M5.Display.setBrightness(savedBrightness); // no-op today; harmless if M5GFX gains a Light
  panelSetBrightness(savedBrightness);
  return true;
}

static void drawDimmer() {
  auto &d = canvas;
  Rect p = dimmerPanelRect();

  d.fillRoundRect(p.x, p.y, p.w, p.h, CARD_RADIUS, COLOR_CARD_BG);
  d.drawRoundRect(p.x, p.y, p.w, p.h, CARD_RADIUS, COLOR_ACCENT);

  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(p.x + 30, p.y + 22);
  d.print("BRIGHTNESS");

  int pct = (savedBrightness - BRIGHTNESS_MIN) * 100 / (BRIGHTNESS_MAX - BRIGHTNESS_MIN);
  char pctBuf[8];
  snprintf(pctBuf, sizeof(pctBuf), "%d%%", pct);
  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(p.x + p.w - 30 - d.textWidth(pctBuf), p.y + 16);
  d.print(pctBuf);

  Rect t = dimmerTrackRect();
  d.fillRoundRect(t.x, t.y, t.w, t.h, t.h / 2, COLOR_BG);
  int fillW = max(t.h, t.w * pct / 100);
  d.fillRoundRect(t.x, t.y, fillW, t.h, t.h / 2, COLOR_ACCENT);
  // Knob, clamped so it stays fully inside the track at both extremes.
  int knobX = constrain(t.x + fillW, t.x + t.h / 2, t.x + t.w - t.h / 2);
  d.fillCircle(knobX, t.y + t.h / 2, t.h / 2 + 4, COLOR_TEXT_PRIMARY);

  drawChip(dimmerOffRect(), "OFF", COLOR_TOPBAR_BG, COLOR_TEXT_PRIMARY, pressTarget == PressTarget::DIM_OFF);
  drawChip(dimmerDoneRect(), "DONE", COLOR_TOPBAR_BG, COLOR_TEXT_PRIMARY, pressTarget == PressTarget::DIM_DONE);
}

// Drawn from drawLogPanel() (per frame, so it can animate) as well as from
// drawStaticChrome() (so it survives a relayout). Its fixed size means the
// self-contained fill fully covers the previous state -- no clear needed.
void drawFilterChip() {
  bool filtering = (NmeaFilter)nmeaFilter != NmeaFilter::ALL;
  drawChip(nmeaFilterChipRect(), nmeaFilterLabel(), filtering ? COLOR_ACCENT_GREEN : COLOR_TOPBAR_BG,
           filtering ? COLOR_BG : COLOR_TEXT_SECONDARY, pressTarget == PressTarget::FILTER);
}

static PressTarget hitTestChips(int x, int y) {
  if (dimmerOpen) {
    if (pointInRect(x, y, dimmerOffRect())) return PressTarget::DIM_OFF;
    if (pointInRect(x, y, dimmerDoneRect())) return PressTarget::DIM_DONE;
    return PressTarget::NONE;
  }
  if (pointInRect(x, y, lightHitRect())) return PressTarget::LIGHT;
  if (pointInRect(x, y, sleepHitRect())) return PressTarget::SLEEP;
  if (pointInRect(x, y, nmeaFilterHitRect())) return PressTarget::FILTER;
  return PressTarget::NONE;
}

// Right-aligned "dot + label" pill, returning its left edge so callers can
// chain the next badge leftward from it.
static int drawDotBadge(int rightEdgeX, int py, const char *text, uint16_t dotColor) {
  auto &d = canvas;
  d.setFont(BADGE_FONT);
  d.setTextSize(1);

  int iconW = BADGE_DOT_R * 2;
  int textX = BADGE_PAD_L + iconW + BADGE_ICON_GAP;
  int pillW = textX + d.textWidth(text) + BADGE_PAD_R;
  int px = rightEdgeX - pillW;

  d.fillRoundRect(px, py, pillW, BADGE_H, BADGE_H / 2, COLOR_CARD_BG);
  d.fillCircle(px + BADGE_PAD_L + BADGE_DOT_R, py + BADGE_H / 2, BADGE_DOT_R, dotColor);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(px + textX, py + (BADGE_H - d.fontHeight()) / 2);
  d.print(text);
  return px;
}

// M5.Power reads go over I2C to the PMIC. Both the status-bar signature and
// the badge itself want them, and the signature is sampled every 200ms -- so
// an uncached pair costs ten I2C round-trips a second for a value that moves
// on the order of minutes. Refreshed at 1Hz and shared by both callers.
static int32_t battLevel = -1; // 0-100, negative if unknown
static bool battCharging = false;
static uint32_t battReadMs = 0;

static void refreshBattery() {
  uint32_t now = millis();
  if (battReadMs != 0 && now - battReadMs < 1000) return;
  battReadMs = now;
  battLevel = M5.Power.getBatteryLevel();
  battCharging = M5.Power.isCharging() == m5::Power_Class::is_charging_t::is_charging;
}

static int drawBatteryBadge(int rightEdgeX, int py) {
  auto &d = canvas;
  refreshBattery();
  int32_t level = battLevel;
  bool charging = battCharging;

  char text[16];
  if (level >= 0) snprintf(text, sizeof(text), "%d%%%s", (int)level, charging ? " CHG" : "");
  else snprintf(text, sizeof(text), "--%s", charging ? " CHG" : "");

  d.setFont(BADGE_FONT);
  d.setTextSize(1);

  // Icon scaled with the pill.
  constexpr int bw = 26, bh = 16, nubW = 4;
  int iconW = bw + nubW;
  int textX = BADGE_PAD_L + iconW + BADGE_ICON_GAP;
  int pillW = textX + d.textWidth(text) + BADGE_PAD_R; // measured on the drawn string
  int px = rightEdgeX - pillW;

  uint16_t barColor = level < 0 ? COLOR_STATUS_NONE : level < 20 ? COLOR_STATUS_BAD : level < 40 ? COLOR_STATUS_WARN : COLOR_STATUS_GOOD;

  d.fillRoundRect(px, py, pillW, BADGE_H, BADGE_H / 2, COLOR_CARD_BG);

  int bx = px + BADGE_PAD_L, by = py + BADGE_H / 2 - bh / 2;
  d.drawRoundRect(bx, by, bw, bh, 3, COLOR_TEXT_SECONDARY);
  d.fillRect(bx + bw, by + 4, nubW, bh - 8, COLOR_TEXT_SECONDARY); // terminal nub
  if (level >= 0) {
    int fillW = max(2, (bw - 4) * constrain((int)level, 0, 100) / 100);
    d.fillRoundRect(bx + 2, by + 2, fillW, bh - 4, 2, barColor);
  }

  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(px + textX, py + (BADGE_H - d.fontHeight()) / 2);
  d.print(text);
  return px;
}

static void drawStatusPill(uint32_t snapLastSentenceMs) {
  auto &d = canvas;

  // Every badge here is right-aligned and chains leftward off the one before
  // it, and each pill's width comes from its own label -- "100%" vs
  // "100% CHG", "SD" vs "NO SD", "RECEIVING" vs "NO DATA", "100%" vs "99%".
  // When any label changes width the whole row shifts, but each badge only
  // repaints its own current footprint, leaving the previous pill's rounded
  // cap and trailing glyphs behind. Unlike the cards, the top-bar background
  // is painted only once (drawStaticChrome at setup), so nothing ever cleans
  // those up. Wipe the strip first. The badge row is far narrower than half
  // the screen, so clearing the right half stays well clear of the title.
  d.fillRect(TOPBAR_CTRL_X, 0, SCREEN_W - TOPBAR_CTRL_X, TOPBAR_H, COLOR_TOPBAR_BG);

  drawChip(lightBtnRect(), "LIGHT", COLOR_CARD_BG, COLOR_TEXT_SECONDARY, pressTarget == PressTarget::LIGHT,
           BADGE_FONT);
  drawChip(sleepBtnRect(), "SLEEP", COLOR_CARD_BG, COLOR_TEXT_SECONDARY, pressTarget == PressTarget::SLEEP,
           BADGE_FONT);

  bool haveData = snapLastSentenceMs != 0 && millis() - snapLastSentenceMs < 5000;
  int py = (TOPBAR_H - BADGE_H) / 2;

  // Right-aligned, each badge chaining leftward off the previous one's left edge.
  int px = drawDotBadge(SCREEN_W - MARGIN, py, haveData ? "RECEIVING" : "NO DATA",
                        haveData ? COLOR_STATUS_GOOD : COLOR_STATUS_BAD);
  int bpx = drawBatteryBadge(px - BADGE_GAP, py);
#if ENABLE_WIFI_NMEA
  char apBuf[12];
  snprintf(apBuf, sizeof(apBuf), "AP %d", nmeaClientCount);
  bpx = drawDotBadge(bpx - BADGE_GAP, py, apBuf,
                     nmeaClientCount > 0 ? COLOR_STATUS_GOOD : COLOR_TEXT_SECONDARY);
#endif
  drawDotBadge(bpx - BADGE_GAP, py, sdReady ? "SD" : "NO SD",
               sdReady ? COLOR_STATUS_GOOD : COLOR_STATUS_NONE);
}

static void drawSparkline(int x, int y, int w, int h, const float *values, int count, uint32_t head,
                          uint16_t color, float maxVal); // defined below, with the trip view

// Medium-weight value+quality-badge / value+caption block, used for the
// accuracy and time readouts -- a step down from the lat/lon hero values but
// well above a plain caption line.
static int drawAccuracyBlock(int x, int y, int w, const RenderSnapshot &s) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print("ACCURACY (HDOP)");
  y += d.fontHeight() + 4;

  char val[12];
  if (s.hdopValid) snprintf(val, sizeof(val), "%.1f", s.hdop);
  else snprintf(val, sizeof(val), "--");
  uint16_t qColor = COLOR_STATUS_NONE;
  const char *qLabel = s.hdopValid ? hdopQuality(s.hdop, qColor) : "NO FIX";

  d.setFont(&fonts::Font4);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(val);
  int valW = d.textWidth(val);
  int heroH = d.fontHeight();

  int bh = 24;
  d.setFont(&fonts::Font2);
  int bw = d.textWidth(qLabel) + 20;
  int bx = x + valW + 14;
  int by = y + (heroH - bh) / 2;
  d.fillRoundRect(bx, by, bw, bh, bh / 2, qColor);
  d.setTextColor(COLOR_BG, qColor);
  d.setCursor(bx + 10, by + (bh - d.fontHeight()) / 2);
  d.print(qLabel);

  // The HDOP trend sits directly under its own value, in the space the
  // PDOP/VDOP caption used to hold. A single number says nothing about whether
  // the fix is settling or degrading, which is the question the accuracy block
  // exists to answer; PDOP/VDOP are still in the stream and on the SD log for
  // anyone who wants the full trio.
  //
  // 18px tall so the block's baseline still lines up with the date line the
  // time block ends on, and the same fixed 0-5 scale the trip view uses -- a
  // trace that autoscaled would make a steady poor fix look identical to a
  // steady good one.
  y += heroH + 4;
  constexpr int TREND_H = 18;
  drawSparkline(x, y, w, TREND_H, s.hdopHistory, s.hdopHistCount, s.hdopHistHead, COLOR_ACCENT, 5.0f);
  y += TREND_H + 2;
  return y;
}

static int drawTimeBlock(int x, int y, int w, const RenderSnapshot &s) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print("TIME (UTC)");
  y += d.fontHeight() + 4;

  char timeBuf[10], dateBuf[16];
  if (s.timeValid) snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d", s.hour, s.minute, s.second);
  else snprintf(timeBuf, sizeof(timeBuf), "--:--:--");
  if (s.dateValid) snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d", s.year, s.month, s.day);
  else snprintf(dateBuf, sizeof(dateBuf), "----------");

  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setTextSize(1);
  d.setCursor(x, y);
  d.print(timeBuf);
  y += d.fontHeight() + 4;

  d.setFont(&fonts::Font2);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(dateBuf);
  y += d.fontHeight() + 2;
  return y;
}

// Ring-buffer line chart. `count`/`head` follow the same convention as the
// raw-log ring buffer: `head` is the next write index, the most recent
// sample is at (head-1).
static void drawSparkline(int x, int y, int w, int h, const float *values, int count, uint32_t head,
                           uint16_t color, float maxVal) {
  auto &d = canvas;
  d.fillRect(x, y, w, h, COLOR_BG);
  if (count < 2) return;

  int prevX = 0, prevY = 0;
  for (int i = 0; i < count; i++) {
    int idx = (int)((head - count + i) % SPARK_LEN);
    if (idx < 0) idx += SPARK_LEN;
    float norm = maxVal > 0 ? constrain(values[idx] / maxVal, 0.0f, 1.0f) : 0;
    int px = x + (int)((float)i / (count - 1) * w);
    int py = y + h - (int)(norm * h);
    if (i > 0) d.drawLine(prevX, prevY, px, py, color);
    prevX = px;
    prevY = py;
  }
}

static int drawTripView(int x, int y, int w, const RenderSnapshot &s) {
  auto &d = canvas;
  int colW = w / 2;

  uint32_t elapsedMs = s.firstFixAbsMs ? (millis() - s.firstFixAbsMs) : 0;
  double avgSpeed = elapsedMs > 2000 ? s.tripDistanceKm / (elapsedMs / 3600000.0) : 0;

  char distBuf[20], maxBuf[20], avgBuf[20];
  snprintf(distBuf, sizeof(distBuf), "%.2f km", s.tripDistanceKm);
  snprintf(maxBuf, sizeof(maxBuf), "%.1f km/h", s.maxSpeedKmph);
  snprintf(avgBuf, sizeof(avgBuf), "%.1f km/h", avgSpeed);

  int y1 = drawHeroValue(x, y, colW - 10, "DISTANCE", distBuf, 1);
  int y2 = drawHeroValue(x + colW, y, colW - 10, "MAX SPEED", maxBuf, 1);
  y = max(y1, y2) + 10;

  char tripBuf[12];
  uint32_t secs = elapsedMs / 1000;
  snprintf(tripBuf, sizeof(tripBuf), "%02u:%02u:%02u", (unsigned)(secs / 3600), (unsigned)((secs / 60) % 60),
           (unsigned)(secs % 60));
  int y3 = drawHeroValue(x, y, colW - 10, "TRIP TIME", tripBuf, 1);
  int y4 = drawHeroValue(x + colW, y, colW - 10, "AVG SPEED", avgBuf, 1);
  y = max(y3, y4) + 8;

  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  char ttffBuf[36];
  if (s.timeToFirstFixMs > 0) snprintf(ttffBuf, sizeof(ttffBuf), "TIME TO FIRST FIX: %.1fs", s.timeToFirstFixMs / 1000.0f);
  else snprintf(ttffBuf, sizeof(ttffBuf), "TIME TO FIRST FIX: --");
  d.setCursor(x, y);
  d.print(ttffBuf);
  y += d.fontHeight() + 10;

  // Speed over the session. Autoscaled to the trip's own maximum rather than a
  // fixed ceiling -- unlike HDOP, there is no absolute scale that suits both a
  // walk and a motorway, and MAX SPEED is printed directly above, so the top of
  // the trace is already labelled.
  float speedScale = max(10.0f, (float)s.maxSpeedKmph);
  d.setCursor(x, y);
  d.print("SPEED");
  y += d.fontHeight() + 4;
  int sparkH = 36;
  drawSparkline(x, y, w, sparkH, s.speedHistory, s.speedHistCount, s.speedHistHead, COLOR_ACCENT, speedScale);
  y += sparkH + 14;

  // How much of the trip actually had a usable fix. For a logger this is the
  // question the recording is only as good as, and nothing on the live view
  // can answer it -- that shows the fix you have now, not the one you had for
  // the last twenty minutes.
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print("FIX QUALITY");
  y += d.fontHeight() + 4;

  uint32_t total = s.fixSecs[0] + s.fixSecs[1] + s.fixSecs[2];
  int barH = 16;
  d.fillRoundRect(x, y, w, barH, barH / 2, COLOR_BG);
  if (total > 0) {
    // Painted worst-first so the eye lands on the good segment's length.
    const uint16_t segColor[3] = {COLOR_STATUS_BAD, COLOR_STATUS_WARN, COLOR_STATUS_GOOD};
    int segX = x;
    for (int i = 0; i < 3; i++) {
      int segW = (int)((uint64_t)w * s.fixSecs[i] / total);
      if (segW > 0) d.fillRect(segX, y, segW, barH, segColor[i]);
      segX += segW;
    }
  }
  y += barH + 8;

  // Legend doubles as the readout: colour plus the actual time at each mode.
  const char *segLabel[3] = {"NONE", "2D", "3D"};
  const uint16_t segColor[3] = {COLOR_STATUS_BAD, COLOR_STATUS_WARN, COLOR_STATUS_GOOD};
  int colStep = w / 3;
  for (int i = 0; i < 3; i++) {
    int lx = x + colStep * i;
    d.fillRect(lx, y + 4, 10, 10, segColor[i]);
    char buf[24];
    uint32_t secs = s.fixSecs[i];
    snprintf(buf, sizeof(buf), "%s %u:%02u", segLabel[i], (unsigned)(secs / 60), (unsigned)(secs % 60));
    d.setTextColor(secs > 0 ? COLOR_TEXT_PRIMARY : COLOR_STATUS_NONE, COLOR_CARD_BG);
    d.setCursor(lx + 16, y);
    d.print(buf);
  }
  y += d.fontHeight() + 2;
  return y;
}

static void drawFixPanel(const RenderSnapshot &s) {
  auto &d = canvas;
  Rect r = innerOf(fixCard);
  d.fillRect(r.x, r.y, r.w, r.h, COLOR_CARD_BG);

  int y = r.y;
  const char *fixStr = s.fixType == 3 ? "3D FIX" : s.fixType == 2 ? "2D FIX" : "NO FIX";
  uint16_t fixColor = s.fixType == 3 ? COLOR_STATUS_GOOD : s.fixType == 2 ? COLOR_STATUS_WARN : COLOR_STATUS_BAD;
  drawBadge(r.x, y, fixStr, fixColor, COLOR_BG);

  char satsBuf[24];
  snprintf(satsBuf, sizeof(satsBuf), "%d/%d SATS", s.satsValid ? s.satsUsed : 0, s.visibleSats);
  d.setFont(&fonts::Font2);
  int satsW = d.textWidth(satsBuf) + 28;
  drawBadge(r.x + r.w - satsW, y, satsBuf, COLOR_TOPBAR_BG, COLOR_TEXT_PRIMARY);
  y += 30 + 22;

  if (positionView == PositionView::TRIP) {
    drawTripView(r.x, y, r.w, s);
    return;
  }

  char latBuf[16], lonBuf[16], altBuf[16], spdBuf[16], crsBuf[16];
  if (s.locValid) {
    snprintf(latBuf, sizeof(latBuf), "%.6f", s.lat);
    snprintf(lonBuf, sizeof(lonBuf), "%.6f", s.lon);
  } else {
    strlcpy(latBuf, "---", sizeof(latBuf));
    strlcpy(lonBuf, "---", sizeof(lonBuf));
  }
  if (s.altValid) snprintf(altBuf, sizeof(altBuf), "%.0fm", s.alt);
  else strlcpy(altBuf, "---", sizeof(altBuf));
  if (s.spdValid) snprintf(spdBuf, sizeof(spdBuf), "%.1fkm/h", s.spd);
  else strlcpy(spdBuf, "---", sizeof(spdBuf));
  if (s.crsValid) snprintf(crsBuf, sizeof(crsBuf), "%.0f%c", s.crs, (char)0xB0);
  else strlcpy(crsBuf, "---", sizeof(crsBuf));

  y = drawHeroValue(r.x, y, r.w, "LATITUDE", latBuf);
  y += 12;
  y = drawHeroValue(r.x, y, r.w, "LONGITUDE", lonBuf);
  y += 20;

  int colW = r.w / 3;
  drawMiniStat(r.x, y, "ALT", altBuf);
  drawMiniStat(r.x + colW, y, "SPEED", spdBuf);
  drawMiniStat(r.x + colW * 2, y, "CRS", crsBuf);
  y += 58;

  int colW2 = r.w / 2;
  int yAcc = drawAccuracyBlock(r.x, y, colW2 - 10, s);
  int yTime = drawTimeBlock(r.x + colW2, y, colW2 - 10, s);
  (void)max(yAcc, yTime);
}

// Polar sky plot: center = zenith (elevation 90), edge = horizon (elevation
// 0). Also draws a heading needle (course-over-ground) when moving, and
// records each dot's screen position/data into skyDots[] for touch hit-testing.
static void drawSkyPlot(const SatInfo arr[MAX_SATS], int cx, int cy, int radius, bool showNeedle, float courseDeg) {
  auto &d = canvas;
  // Blit the pre-rendered rings/labels instead of redrawing them -- this
  // also resets the area to a clean background, erasing last tick's dots.
  radarBg.pushSprite(&canvas, cx - radarBgSize / 2, cy - radarBgSize / 2);

  // Course rides the bezel as a chevron instead of a needle through the middle.
  // A needle looks right on an aircraft HSI, where it points across empty
  // gauge -- here it lies on top of the readout. Measured against a simulated
  // sky, a solid arrow covers about 1.5 of 14 satellites at any moment and a
  // compass-style needle about 2.6; the chevron covers none of them, because
  // nothing enters the plot at all.
  if (showNeedle) {
    float a = radians(courseDeg);
    float aL = radians(courseDeg - 5.0f), aR = radians(courseDeg + 5.0f);
    int tipX = cx + (int)((radius + 20) * sinf(a));
    int tipY = cy - (int)((radius + 20) * cosf(a));
    int lx = cx + (int)((radius + 3) * sinf(aL));
    int ly = cy - (int)((radius + 3) * cosf(aL));
    int rx = cx + (int)((radius + 3) * sinf(aR));
    int ry = cy - (int)((radius + 3) * cosf(aR));
    d.fillTriangle(tipX, tipY, lx, ly, rx, ry, COLOR_ACCENT);
    // A short arc anchors the chevron to the rim so it still reads as a bearing
    // when it happens to sit between two ticks. LGFX arc angles start at +X and
    // run clockwise, hence the -90.
    d.fillArc(cx, cy, radius + 1, radius + 4, courseDeg - 96.0f, courseDeg - 84.0f, COLOR_ACCENT);
  } else {
    // Below ~1km/h course-over-ground is noise, so there is no bearing to draw.
    // Say so, rather than removing the marker and leaving the instrument
    // looking broken.
    d.setFont(&fonts::Font2);
    d.setTextSize(1);
    d.setTextColor(COLOR_STATUS_NONE, COLOR_BG);
    const char *msg = "NO COURSE";
    d.setCursor(cx - d.textWidth(msg) / 2, cy + (int)(radius * 0.72f));
    d.print(msg);
  }

  skyDotCount = 0;
  for (int i = 0; i < MAX_SATS; i++) {
    const SatInfo &s = arr[i];
    if (!s.used || s.elevation < 0 || s.azimuth < 0) continue;
    float az = radians((float)s.azimuth);
    float rr = radius * (90 - s.elevation) / 90.0f;
    int x = cx + (int)(rr * sinf(az));
    int y = cy - (int)(rr * cosf(az));
    int dotR = s.snr < 0 ? 4 : map(constrain(s.snr, 0, 55), 0, 55, 4, 9);
    d.fillCircle(x, y, dotR, snrColor(s.snr));
    if (skyDotCount < MAX_SATS) skyDots[skyDotCount++] = {x, y, s};
  }
}

static void drawSatList(const SatInfo arr[MAX_SATS], int x, int y, int w, int h) {
  auto &d = canvas;
  d.fillRect(x, y, w, h, COLOR_CARD_BG);
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print("SIGNAL");
  int rowY = y + d.fontHeight() + 10;

  int order[MAX_SATS];
  int countUsed = sortedSatOrder(arr, order);

  int rowH = 24;
  int maxRows = (y + h - rowY) / rowH;
  int labelW = 56;
  int barX = x + labelW;
  int barW = w - labelW - 34;
  if (barW < 10) barW = 10;

  for (int k = 0; k < countUsed && k < maxRows; k++) {
    const SatInfo &s = arr[order[k]];
    char buf[12];
    snprintf(buf, sizeof(buf), "%s%d", s.talker, s.prn);
    d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
    d.setCursor(x, rowY);
    d.print(buf);

    int bw = s.snr < 0 ? 2 : map(constrain(s.snr, 0, 55), 0, 55, 2, barW);
    d.fillRoundRect(barX, rowY + 2, barW, 12, 6, COLOR_BG);
    d.fillRoundRect(barX, rowY + 2, bw, 12, 6, snrColor(s.snr));

    char snrBuf[8];
    if (s.snr >= 0) snprintf(snrBuf, sizeof(snrBuf), "%d", s.snr);
    else snprintf(snrBuf, sizeof(snrBuf), "-");
    d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
    d.setCursor(barX + barW + 6, rowY);
    d.print(snrBuf);

    rowY += rowH;
  }
}

// Small floating detail card for whichever satellite dot was last tapped;
// self-expires 4s after the tap.
static void drawSatTooltip(const Rect &r) {
  if (!satTooltip.active) return;
  if (millis() - satTooltip.shownAtMs > 4000) {
    satTooltip.active = false;
    return;
  }

  auto &d = canvas;
  const SatInfo &s = satTooltip.info;
  char snrBuf[8];
  if (s.snr >= 0) snprintf(snrBuf, sizeof(snrBuf), "%d", s.snr);
  else strlcpy(snrBuf, "-", sizeof(snrBuf));
  char buf[48];
  snprintf(buf, sizeof(buf), "%s%d  EL %d  AZ %d  SNR %s", s.talker, s.prn, s.elevation, s.azimuth, snrBuf);

  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  int tw = d.textWidth(buf);
  int boxW = tw + 24, boxH = 30;
  int bx = r.x + (r.w - boxW) / 2;
  int by = r.y + r.h - boxH - 6;
  uint16_t bg = snrColor(s.snr);

  d.fillRoundRect(bx, by, boxW, boxH, boxH / 2, bg);
  d.setTextColor(COLOR_BG, bg);
  d.setCursor(bx + 12, by + (boxH - d.fontHeight()) / 2);
  d.print(buf);
}

static void drawSatPanel(const RenderSnapshot &s) {
  Rect r = innerOf(skyCard);
  // The radar blit and drawSatList() each clear their own region, so a
  // full-panel clear here would be redundant -- except drawSatTooltip()
  // floats near the bottom, centered across the full width, and can fall
  // outside both of those regions. Clear just its footprint so a stale or
  // expired tooltip box can never linger.
  canvas.fillRect(r.x, r.y + r.h - 40, r.w, 40, COLOR_CARD_BG);

  SkyGeom g = skyGeom(r);
  bool showNeedle = s.crsValid && s.spdValid && s.spd > 1.0; // course-over-ground is noisy near-stationary
  drawSkyPlot(s.sats, g.cx, g.cy, g.radius, showNeedle, (float)s.crs);

  int listX = g.cx + g.size / 2 + 4;
  int listW = r.x + r.w - listX;
  drawSatList(s.sats, listX, r.y, listW, r.h);

  drawSatTooltip(r);
}

static void drawLogPanel(const RenderSnapshot &s) {
  auto &d = canvas;
  Rect r = innerOf(logCard);
  d.fillRect(r.x, r.y, r.w, r.h, COLOR_CARD_BG);

  drawFilterChip(); // above innerOf(), but inside the logCard dirty rect

  // Expanded is the "read the sentences" view, so it gets larger type and the
  // full card width. The width matters: a long GSV sentence at Font4 runs to
  // roughly 980px, which would clip against the ~968px left column that the
  // sentence-type counters leave behind. Those counters stay in the collapsed
  // view, where the smaller font leaves width to spare.
  bool big = logExpanded;
  d.setFont(big ? &fonts::Font4 : &fonts::Font2);
  d.setTextSize(1);

  int rightW = big ? 0 : 220;
  int leftW = big ? r.w : r.w - rightW - 24;

  d.setTextColor(COLOR_ACCENT_GREEN, COLOR_CARD_BG);
  int rowH = d.fontHeight() + (big ? 6 : 5);
  int maxRows = r.h / rowH;
  int shown = min<uint32_t>(min<uint32_t>(s.logHead, LOG_LINES), (uint32_t)maxRows);

  // Oldest at the top, newest on the last row. Bottom-anchored so the newest
  // sentence sits on the final row even before the buffer has filled, instead
  // of the block hugging the top with a gap beneath it.
  int y = r.y + (maxRows - shown) * rowH;
  for (int i = 0; i < shown; i++) {
    int idx = (s.logHead - shown + i) % LOG_LINES;
    d.setCursor(r.x, y);
    d.print(s.logLines[idx]);
    y += rowH;
  }

  if (big) return; // no counter column in the expanded view

  int dividerX = r.x + leftW + 12;
  d.drawFastVLine(dividerX, r.y, r.h, COLOR_DIVIDER);

  // Right: cumulative sentence-type counts -- gives the wide log card a
  // purpose beyond the raw text, which rarely fills its own width.
  int cx0 = dividerX + 12;
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(cx0, r.y);
  d.print("SENTENCE TYPES");
  int cy = r.y + d.fontHeight() + 8;
  int colW = rightW / 2 - 6;
  int rowH2 = d.fontHeight() + 8;
  for (int i = 0; i < NUM_SENTENCE_TYPES; i++) {
    int col = i % 2, row = i / 2;
    int cxp = cx0 + col * (colW + 12);
    int cyp = cy + row * rowH2;
    char buf[16];
    snprintf(buf, sizeof(buf), "%-3s %lu", SENTENCE_TYPES[i], (unsigned long)s.typeCounts[i]);
    d.setTextColor(s.typeCounts[i] > 0 ? COLOR_TEXT_PRIMARY : COLOR_STATUS_NONE, COLOR_CARD_BG);
    d.setCursor(cxp, cyp);
    d.print(buf);
  }
}

// -------------------------------------------------- change detection/dirty --
// Each panel gets a cheap signature of exactly the values it renders. A panel
// is only redrawn when its own signature changes, so panels update at their
// own natural rates -- the fix/satellite cards effectively at the GPS's 1Hz,
// the NMEA log as sentences land, the badge row only when a badge's value
// changes -- rather than all of them on a shared timer.
//
// Fields are hashed individually rather than memcmp'd over a struct, so
// padding bytes can't produce spurious mismatches.

template <typename T>
static uint32_t hashVal(uint32_t h, const T &v) {
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&v);
  for (size_t i = 0; i < sizeof(T); i++) {
    h ^= p[i];
    h *= 16777619u;
  }
  return h;
}

static uint32_t hashStr(uint32_t h, const char *s) {
  while (*s) {
    h ^= (uint8_t)*s++;
    h *= 16777619u;
  }
  return h;
}

static constexpr uint32_t FNV_SEED = 2166136261u;

static uint32_t sigStatusBar(const RenderSnapshot &s) {
  uint32_t h = FNV_SEED;
  bool haveData = s.lastSentenceMs != 0 && millis() - s.lastSentenceMs < 5000;
  refreshBattery(); // rate-limited: the badge below reads the same cached pair
  h = hashVal(h, haveData);
  h = hashVal(h, battLevel);
  h = hashVal(h, battCharging);
  h = hashVal(h, sdReady);
#if ENABLE_WIFI_NMEA
  int clients = nmeaClientCount;
  h = hashVal(h, clients);
#endif
  bool pressLight = pressTarget == PressTarget::LIGHT;
  bool pressSleep = pressTarget == PressTarget::SLEEP;
  h = hashVal(h, pressLight);
  h = hashVal(h, pressSleep);
  return h;
}

static uint32_t sigFixPanel(const RenderSnapshot &s) {
  uint32_t h = FNV_SEED;
  uint8_t view = (uint8_t)positionView;
  h = hashVal(h, view);
  h = hashVal(h, s.fixType);
  h = hashVal(h, s.satsValid); h = hashVal(h, s.satsUsed); h = hashVal(h, s.visibleSats);

  if (view == (uint8_t)PositionView::TRIP) {
    h = hashVal(h, s.tripDistanceKm);
    h = hashVal(h, s.maxSpeedKmph);
    h = hashVal(h, s.timeToFirstFixMs);
    h = hashVal(h, s.speedHistHead); // the trend the trip face draws now
    for (int i = 0; i < 3; i++) h = hashVal(h, s.fixSecs[i]);
    // The trip clock ticks visibly, so fold in whole elapsed seconds.
    uint32_t elapsedSec = s.firstFixAbsMs ? (millis() - s.firstFixAbsMs) / 1000 : 0;
    h = hashVal(h, elapsedSec);
  } else {
    h = hashVal(h, s.locValid); h = hashVal(h, s.lat); h = hashVal(h, s.lon);
    h = hashVal(h, s.altValid); h = hashVal(h, s.alt);
    h = hashVal(h, s.spdValid); h = hashVal(h, s.spd);
    h = hashVal(h, s.crsValid); h = hashVal(h, s.crs);
    h = hashVal(h, s.hdopValid); h = hashVal(h, s.hdop);
    // The live view draws the HDOP trend now, not the DOP trio -- so the graph
    // has to be able to trigger its own repaint, and a PDOP that jitters
    // without anything visible changing must not.
    h = hashVal(h, s.hdopHistHead);
    h = hashVal(h, s.dateValid); h = hashVal(h, s.timeValid);
    h = hashVal(h, s.year); h = hashVal(h, s.month); h = hashVal(h, s.day);
    h = hashVal(h, s.hour); h = hashVal(h, s.minute); h = hashVal(h, s.second);
  }
  return h;
}

static uint32_t sigSatPanel(const RenderSnapshot &s) {
  uint32_t h = FNV_SEED;
  for (int i = 0; i < MAX_SATS; i++) {
    const SatInfo &sat = s.sats[i];
    if (!sat.used) continue;
    h = hashStr(h, sat.talker);
    h = hashVal(h, sat.prn);
    h = hashVal(h, sat.elevation);
    h = hashVal(h, sat.azimuth);
    h = hashVal(h, sat.snr);
  }
  bool needle = s.crsValid && s.spdValid && s.spd > 1.0;
  h = hashVal(h, needle);
  if (needle) h = hashVal(h, s.crs);

  // Fold in tooltip visibility so its 4s expiry triggers a repaint on its own.
  bool tipVisible = satTooltip.active && (millis() - satTooltip.shownAtMs <= 4000);
  h = hashVal(h, tipVisible);
  if (tipVisible) {
    h = hashStr(h, satTooltip.info.talker);
    h = hashVal(h, satTooltip.info.prn);
  }
  return h;
}

static uint32_t sigLogPanel(const RenderSnapshot &s) {
  uint32_t h = FNV_SEED;
  h = hashVal(h, s.logHead); // sentence-type counts advance in lockstep with this
  h = hashVal(h, logExpanded);
  h = hashVal(h, nmeaFilter); // the chip label lives in this panel
  bool pressFilter = pressTarget == PressTarget::FILTER;
  h = hashVal(h, pressFilter);
  return h;
}

// Regions whose canvas pixels changed this frame, pushed individually so a
// push costs only the area that actually moved rather than the full screen.
static constexpr int MAX_DIRTY = 4;
static Rect dirtyRects[MAX_DIRTY];
static int dirtyCount = 0;

static void markDirty(const Rect &r) {
  if (r.w <= 0 || r.h <= 0) return;
  if (dirtyCount < MAX_DIRTY) dirtyRects[dirtyCount++] = r;
}

// pushImage() clips by adjusting the copied rect and source offset rather than
// masking, so a clipped pushSprite genuinely transfers fewer pixels. Note the
// canvas lives in PSRAM, which disables DMA on the blit -- making the transfer
// CPU-bound and the saving proportional to the area skipped.
// Pushed in horizontal bands with a touch sample between them. A full-canvas
// push moves 1280x720x2 bytes out of PSRAM with no DMA, and for its whole
// duration nothing polls the panel -- so a tap that began and ended inside it
// was lost outright. Banding costs a few extra clip setups and bounds the
// blind window to one slice instead of the whole blit. It is invisible: the
// canvas is already fully composed, so the bands carry finished pixels.
static constexpr int PUSH_SLICE_H = 120;

static void pushRegion(const Rect &r) {
  int bottom = r.y + r.h;
  for (int y = r.y; y < bottom; y += PUSH_SLICE_H) {
    int h = min(PUSH_SLICE_H, bottom - y);
    M5.Display.setClipRect(r.x, y, r.w, h);
    canvas.pushSprite(&M5.Display, 0, 0);
    sampleTouch();
  }
  M5.Display.clearClipRect();
}

static void pushDirty(bool full) {
  if (full) {
    pushRegion({0, 0, SCREEN_W, SCREEN_H});
    return;
  }
  for (int i = 0; i < dirtyCount; i++) {
    pushRegion(dirtyRects[i]);
  }
}

// ------------------------------------------------------------------ setup --
// computeLayout()/relayout() now live in layout.cpp.

// ------------------------------------------------------------ power states --

static void rememberBrightness() {
  uint8_t b = M5.Display.getBrightness();
  if (b > 0) savedBrightness = b; // never latch 0, or wake would restore to dark
}

// NOTE: M5.Display.setBrightness() is a no-op on Tab5 with M5GFX 0.2.26 --
// Panel_Device::setBrightness() forwards to a Light instance, and Tab5's init
// path never attaches one (Panel_DSI/ST7123/ST7121 implement no brightness
// either, and neither IO expander carries a backlight pin). The calls are kept
// so this does the right thing if a future M5GFX wires one up, but the visible
// effect comes from blanking the canvas to black and pushing that.
static void blankScreen() {
  canvas.fillScreen(0);
  canvas.pushSprite(&M5.Display, 0, 0);
}

// The backlight really does go off at 0: M5GFX attaches no Light instance for
// Tab5, so M5.Display.setBrightness() does nothing and the DCS 0x51 write in
// panelSetBrightness() is the only control there is. It works -- confirmed on
// hardware by the panel visibly glowing when this was briefly set to
// BRIGHTNESS_MIN instead.
//
// That brief detour was an attempt to explain a sleeping screen that would not
// wake on a tap, on the theory that the ST7123 -- one controller for both the
// display and the digitiser -- might stop reporting touch with its display
// block idled. It is a real possibility: M5GFX's own Tab5 timing block warns
// that shrinking the vertical front porch "will cause the touch panel to stop
// working". But the same commit also fixed the wake path itself, which needed
// a press AND a release both seen through a 20ms sampling window, and that is
// the likelier reason a tap did nothing.
//
// So: dark again, with the wake fixes kept. If tapping a sleeping screen stops
// working, the "touch while dark" line on the serial monitor says which half is
// at fault -- and the targeted next step is clearing the BL bit in DCS 0x53
// (ctrl 0x28 rather than 0x2C), which switches the backlight off without
// touching the brightness value at all.
static void enterBacklightOff() {
  rememberBrightness();
  backlightOff = true;
  M5.Display.setBrightness(0);
  panelSetBrightness(0);
  blankScreen();
}

// Deliberately zeroes the backlight rather than calling M5.Display.sleep().
// Tab5's newer panels use an integrated display+touch controller (ST7123 /
// ST7121), so putting the panel to sleep risks taking the touch digitiser with
// it -- which would make tap-to-wake impossible. The backlight is the dominant
// consumer anyway, and the radio going down is the other half of the saving.
static void enterSleep() {
  rememberBrightness();
  asleep = true;
  wifiEnabled = false; // wifiTask tears the AP down and powers the radio off
  M5.Display.setBrightness(0);
  panelSetBrightness(0); // see enterBacklightOff() for why this is 0 again
  blankScreen();
}

static void wakeDisplay() {
  if (asleep) {
    wifiEnabled = true; // wifiTask brings the AP back up
    asleep = false;
  }
  backlightOff = false;
  M5.Display.setBrightness(savedBrightness);
  panelSetBrightness(savedBrightness);
  relayout(); // force a full repaint and full push
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  if (M5.Display.height() > M5.Display.width()) {
    M5.Display.setRotation((M5.Display.getRotation() + 1) % 4);
  }
  SCREEN_W = M5.Display.width();
  SCREEN_H = M5.Display.height();

  computeLayout();

  canvas.setColorDepth(16);
  canvas.setPsram(true);
  if (!canvas.createSprite(SCREEN_W, SCREEN_H)) {
    Serial.println("ERROR: failed to allocate PSRAM canvas -- check BOARD_HAS_PSRAM build flag");
  }

  // Build the sky-plot radar background once, matching the geometry
  // drawSatPanel() computes from skyCard (fixed for the life of the program
  // -- logExpanded zeroes skyCard out, but never resizes it otherwise).
  {
    SkyGeom g = skyGeom(innerOf(skyCard));
    int radius = g.radius;
    radarBgSize = g.size;

    radarBg.setColorDepth(16);
    radarBg.setPsram(true);
    radarBg.createSprite(radarBgSize, radarBgSize);
    radarBg.fillSprite(COLOR_CARD_BG);

    int bcx = radarBgSize / 2, bcy = radarBgSize / 2;
    radarBg.fillCircle(bcx, bcy, radius, COLOR_BG);
    radarBg.drawCircle(bcx, bcy, radius, COLOR_DIVIDER);
    radarBg.drawCircle(bcx, bcy, radius * 2 / 3, COLOR_DIVIDER);
    radarBg.drawCircle(bcx, bcy, radius / 3, COLOR_DIVIDER);
    radarBg.drawFastHLine(bcx - radius, bcy, radius * 2, COLOR_DIVIDER);
    radarBg.drawFastVLine(bcx, bcy - radius, radius * 2, COLOR_DIVIDER);

    // Tick ring: every 10 degrees, longer and brighter every 30. This is what
    // makes an azimuth readable off the plot -- four cardinal letters alone
    // leave you interpolating across a 90 degree gap.
    for (int a = 0; a < 360; a += 10) {
      bool major = (a % 30) == 0;
      float rad_a = radians((float)a);
      int r0 = radius + 3, r1 = radius + (major ? 15 : 8);
      int x0 = bcx + (int)(r0 * sinf(rad_a)), y0 = bcy - (int)(r0 * cosf(rad_a));
      int x1 = bcx + (int)(r1 * sinf(rad_a)), y1 = bcy - (int)(r1 * cosf(rad_a));
      if (major) {
        radarBg.drawWideLine(x0, y0, x1, y1, 2.0f, COLOR_TEXT_SECONDARY);
      } else {
        radarBg.drawLine(x0, y0, x1, y1, COLOR_DIVIDER);
      }
    }

    // Fixed index at north, in COLOR_TEXT_PRIMARY. White means "reference that
    // never moves" here; the live course chevron is COLOR_ACCENT so the two can
    // never be confused, and neither can be mistaken for a satellite, which is
    // always one of the three signal colours.
    radarBg.fillTriangle(bcx, bcy - radius - 2, bcx - 7, bcy - radius - 16, bcx + 7, bcy - radius - 16,
                         COLOR_TEXT_PRIMARY);

    // Elevation rings mean nothing unlabelled. Inner ring is 60 degrees, middle
    // 30 -- rr = radius * (90 - el) / 90, so the labels sit where each ring
    // crosses the vertical axis.
    radarBg.setFont(&fonts::Font0);
    radarBg.setTextSize(1);
    radarBg.setTextColor(COLOR_TEXT_SECONDARY, COLOR_BG);
    radarBg.setCursor(bcx + 4, bcy - radius / 3 - 3);
    radarBg.print("60");
    radarBg.setCursor(bcx + 4, bcy - radius * 2 / 3 - 3);
    radarBg.print("30");

    // Cardinals move out past the tick ring and step up to Font2: they were
    // Font0, the smallest type anywhere on the panel.
    radarBg.setFont(&fonts::Font2);
    radarBg.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
    auto label = [&](const char *s, int lx, int ly) {
      int tw = radarBg.textWidth(s), th = radarBg.fontHeight();
      radarBg.setCursor(lx - tw / 2, ly - th / 2);
      radarBg.print(s);
    };
    int lr = radius + 30;
    label("N", bcx, bcy - lr);
    label("S", bcx, bcy + lr);
    label("E", bcx + lr, bcy);
    label("W", bcx - lr, bcy);
  }

  drawStaticChrome();
  canvas.pushSprite(&M5.Display, 0, 0);

  Serial.begin(115200);
  GPSSerial.setRxBufferSize(2048); // headroom for the mutex-guarded parse burst
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Serial.println("Tab5 GPS logger starting...");

  initSdLogging();

  stateMutex = xSemaphoreCreateMutex();
#if ENABLE_WIFI_NMEA
  nmeaQueue = xQueueCreate(64, sizeof(NmeaQueueMsg));
#endif
  int mainCore = xPortGetCoreID();
  int gpsCore = mainCore == 0 ? 1 : 0;
  xTaskCreatePinnedToCore(gpsTaskFn, "gps_task", 4096, nullptr, 2, nullptr, gpsCore);
#if ENABLE_WIFI_NMEA
  xTaskCreatePinnedToCore(wifiTaskFn, "wifi_task", 4096, nullptr, 1, nullptr, tskNO_AFFINITY);
#endif
}

void loop() {
  M5.update();

  // Touch edge detection, done here rather than via wasClicked()/wasReleased().
  //
  // getDetail(i) resolves through _touch_raw[i].id, and _touch_raw is only
  // refreshed for the points the current scan reports. On release the driver
  // reports zero points, so those entries hold stale data from an earlier
  // scan -- and unlike getTouchPointRaw(), getDetail() does not clamp the
  // index against the live point count. Reading a second slot that way is
  // therefore not deterministic: the stale id can alias back to slot 0 or
  // point at an unrelated detail slot.
  //
  // Instead: sample coordinates only while a contact is definitely down (raw
  // data valid then), and detect the lift ourselves as the absence of a
  // press. That is unambiguous, fires exactly once per physical touch, and
  // is immune both to the ghost second contact this panel emits and to the
  // state machine escalating a tap to hold/flick/drag.
  sampleTouch();
  bool pressing = touchPressing;
  bool pressEdge = touchPressEdge;
  bool released = touchReleaseEdge;
  touchPressEdge = false;
  touchReleaseEdge = false;

  // Everything below hit-tests the touch-DOWN position, not the lift position.
  // A fingertip rolls as it leaves the glass, so the last sample before a lift
  // can sit several pixels from where the user actually aimed -- and it is the
  // aim that should decide.
  if (pressEdge) capturedTarget = hitTestChips(touchDownX, touchDownY);

  // While the screen is dark, a tap only restores it and is consumed -- you
  // can't aim at a control you can't see, so no other target should fire.
  // Note GPS ingestion, SD logging and trip stats all keep running here; only
  // the display work is skipped.
  if (asleep || backlightOff) {
    // Wake on the press, not the lift. Waiting for a full press-release cycle
    // meant both edges had to be observed through a 20ms sampling window; the
    // press alone is unambiguous, and there is nothing else a tap could mean
    // while the screen is dark.
    if (pressEdge || released) {
      capturedTarget = PressTarget::NONE; // the waking tap commits nothing else
      wakeDisplay();
      return;
    }
    // Escape hatch: if the panel stops reporting touch while dark, the power
    // button still gets the display back.
    if (M5.BtnPWR.wasClicked()) {
      capturedTarget = PressTarget::NONE;
      wakeDisplay();
      return;
    }
    // Whether the digitiser reports anything at all while dark is the one
    // thing that cannot be reasoned about off-device, so say so on the wire.
    static uint32_t lastDarkLogMs = 0;
    if (touchPressing && millis() - lastDarkLogMs > 1000) {
      lastDarkLogMs = millis();
      Serial.printf("touch while dark: %d,%d\n", touchLastX, touchLastY);
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // nothing is visible, so ease off the CPU
    return;
  }

  // Highlight whichever chip the finger is currently over, and drop it on lift.
  // The highlight shows the captured chip for the whole press, rather than
  // re-testing the moving contact. That keeps it honest: what is lit is what
  // will fire on release, including when the finger has drifted off the chip.
  PressTarget prevPress = pressTarget;
  if (pressing) {
    pressTarget = capturedTarget;
  } else if (released) {
    pressTarget = PressTarget::NONE;
  }
  bool pressChanged = pressTarget != prevPress;

  // The brightness overlay is modal: it owns touch and drawing while open, so
  // the panels underneath can't paint over it.
  if (dimmerOpen) {
    // A press that starts on the slider grabs it, and keeps it until the lift.
    // Only X matters from then on: testing the live position against the track
    // each sample meant any vertical drift mid-slide stopped the value dead,
    // which reads as the slider sticking.
    if (pressEdge && pointInRect(touchDownX, touchDownY, dimmerTrackHitRect())) {
      dimmerDragging = true;
    }
    if (dimmerDragging && pressing) {
      if (setBrightnessFromTouch(touchLastX)) dimmerDirty = true;
    }
    if (released) dimmerDragging = false;
    if (pressChanged) dimmerDirty = true;

    if (released) {
      PressTarget hit = capturedTarget;
      capturedTarget = PressTarget::NONE;
      if (hit == PressTarget::DIM_OFF) {
        dimmerOpen = false;
        enterBacklightOff(); // keeps the level just chosen for the next wake
        return;
      }
      if (hit == PressTarget::DIM_DONE || !pointInRect(touchDownX, touchDownY, dimmerPanelRect())) {
        dimmerOpen = false;
        relayout(); // repaint whatever the overlay was covering
        return;
      }
      dimmerDirty = true;
    }

    if (dimmerDirty) {
      dimmerDirty = false;
      drawDimmer();
      dirtyCount = 0;
      markDirty(dimmerPanelRect());
      pushDirty(false);
    }
    return;
  }

  // Each branch marks only the panel(s) it actually affects. A satellite tap
  // only needs the sat panel redrawn; the fix/log cards are untouched by it.
  // A view/expand toggle calls relayout(), which repaints the whole canvas'
  // static chrome, so all three panel *contents* need repainting afterward
  // regardless of which card was tapped.
  bool wantFix = false, wantSat = false, wantLog = false;
  {
    {
      // The touch-DOWN position, not the lift position: a fingertip rolls as
      // it leaves the glass, and the aim is what should decide.
      int tx = touchDownX, ty = touchDownY;
      if (released && tx >= 0 && ty >= 0) {
        // Chips commit on the press that began inside them, so a small drift
        // before the lift no longer swallows the tap.
        PressTarget chip = capturedTarget;
        capturedTarget = PressTarget::NONE;

        // Scoping the hit test to skyCard keeps the enlarged pick radius from
        // reaching across the gap into a neighbouring card.
        bool inSky = !logExpanded && pointInRect(tx, ty, skyCard);
        int hit = inSky ? hitTestSkyDot(tx, ty) : -1;
        if (chip == PressTarget::LIGHT) {
          dimmerOpen = true;
          dimmerDirty = true;
          return; // the modal block draws it on the next pass
        } else if (chip == PressTarget::SLEEP) {
          enterSleep();
          return;
        } else if (hit >= 0) {
          satTooltip = {true, skyDots[hit].info, millis()};
          wantSat = true;
        } else if (inSky) {
          // Tapped the sky plot but not a satellite: close any open tooltip so
          // the tap still produces visible feedback instead of reading as dead.
          if (satTooltip.active) {
            satTooltip.active = false;
            wantSat = true;
          }
        } else if (!logExpanded && pointInRect(tx, ty, fixCard)) {
          positionView = positionView == PositionView::LIVE ? PositionView::TRIP : PositionView::LIVE;
          relayout();
          wantFix = wantSat = wantLog = true;
        } else if (chip == PressTarget::FILTER) {
          // Checked before the logCard branch -- the chip sits inside it.
          // Clearing the ring under the lock gives immediate feedback and
          // avoids showing a mix of pre- and post-filter sentences.
          xSemaphoreTake(stateMutex, portMAX_DELAY);
          nmeaFilter = (nmeaFilter + 1) % NMEA_FILTER_COUNT;
          rawLogHead = 0;
          for (auto &l : rawLog) l[0] = '\0';
          xSemaphoreGive(stateMutex);
          relayout();
          wantFix = wantSat = wantLog = true;
        } else if (pointInRect(tx, ty, logCard)) {
          logExpanded = !logExpanded;
          relayout();
          wantFix = wantSat = wantLog = true;
        }
      }
    }
  }
  bool touchAction = wantFix || wantSat || wantLog;

  // relayout() repaints the whole canvas (background + card frames), so those
  // frames need a full-screen push rather than per-panel dirty rects.
  bool fullRepaint = relayoutPending;
  relayoutPending = false;

  static uint32_t lastPollMs = 0;
  static RenderSnapshot snap;
  static bool haveSnapshot = false;
  uint32_t now = millis();

  // Polling only samples signatures -- it does not imply any drawing. Panels
  // whose values are unchanged cost nothing here, so this rate sets how
  // quickly a change is *noticed*, not how often anything is repainted.
  bool poll = now - lastPollMs >= 200;
  // pressChanged bypasses the poll interval so press highlights track the
  // finger instead of lagging up to a full poll behind it.
  if (!poll && !touchAction && !fullRepaint && !pressChanged) return;
  if (poll) lastPollMs = now;

  // Only re-snapshot on a poll. A touch changes UI state, not GPS state, and
  // captureSnapshot() copies MAX_SATS + LOG_LINES Strings while holding
  // stateMutex -- doing that per tap added heap churn to the tap path and
  // blocked gpsTask from draining the UART for its duration.
  if (poll || !haveSnapshot) {
    captureSnapshot(snap);
    haveSnapshot = true;
  }

  static uint32_t lastSigStatus = 0, lastSigFix = 0, lastSigSat = 0, lastSigLog = 0;
  uint32_t nSigStatus = sigStatusBar(snap);
  uint32_t nSigFix = sigFixPanel(snap);
  uint32_t nSigSat = sigSatPanel(snap);
  uint32_t nSigLog = sigLogPanel(snap);

  bool doStatus = fullRepaint || nSigStatus != lastSigStatus;
  bool doFix = !logExpanded && (fullRepaint || wantFix || nSigFix != lastSigFix);
  bool doSat = !logExpanded && (fullRepaint || wantSat || nSigSat != lastSigSat);
  bool doLog = fullRepaint || wantLog || nSigLog != lastSigLog;

  lastSigStatus = nSigStatus;
  lastSigFix = nSigFix;
  lastSigSat = nSigSat;
  lastSigLog = nSigLog;

  if (!doStatus && !doFix && !doSat && !doLog) return;

  dirtyCount = 0;
  if (doStatus) {
    drawStatusPill(snap.lastSentenceMs);
    markDirty({TOPBAR_CTRL_X, 0, SCREEN_W - TOPBAR_CTRL_X, TOPBAR_H});
  }
  if (doFix) {
    drawFixPanel(snap);
    markDirty(fixCard);
  }
  if (doSat) {
    drawSatPanel(snap);
    markDirty(skyCard);
  }
  if (doLog) {
    drawLogPanel(snap);
    markDirty(logCard);
  }

  pushDirty(fullRepaint);
}
