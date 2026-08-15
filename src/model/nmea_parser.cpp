#include <Arduino.h> // millis()
#include <cstring>
#include <cstdlib>
#include "model/nmea_parser.h"
#include "model/gps_model.h"
#include "io/sd_logger.h"
#include "io/wifi_nmea.h"

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

char rawLog[LOG_LINES][RAW_LINE_MAX];
uint32_t rawLogHead = 0;
uint32_t lastSentenceMs = 0;

const char *SENTENCE_TYPES[NUM_SENTENCE_TYPES] = {"GGA", "RMC", "GSV", "GSA", "GLL", "VTG", "ZDA", "TXT", "OTH"};
uint32_t sentenceTypeCounts[NUM_SENTENCE_TYPES] = {0};

uint8_t nmeaFilter = (uint8_t)NmeaFilter::ALL;

const char *nmeaFilterLabel() {
  switch ((NmeaFilter)nmeaFilter) {
    case NmeaFilter::NO_GSV: return "NO GSV";
    case NmeaFilter::POSITION: return "POS";
    default: return "ALL";
  }
}

bool sentencePassesFilter(const char *type) {
  switch ((NmeaFilter)nmeaFilter) {
    case NmeaFilter::NO_GSV: return strcmp(type, "GSV") != 0;
    case NmeaFilter::POSITION: return strcmp(type, "GGA") == 0 || strcmp(type, "RMC") == 0;
    default: return true;
  }
}

void logSentence(const char *line) {
  logToSD(line);
#if ENABLE_WIFI_NMEA
  enqueueNmea(line);
#endif
}

void handleRawSentence(const char *line, int len) {
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
