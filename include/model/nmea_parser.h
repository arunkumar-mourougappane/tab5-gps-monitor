#pragma once
#include <cstdint>

// ------------------------------------------------------- NMEA line parsing --
// All of this only ever runs inside gpsTask, holding stateMutex.

static constexpr int LOG_LINES = 40;
// Longest legal NMEA sentence is 82 bytes including CRLF; the assembler caps a
// line at 100 like the String version did, so 104 leaves room for the NUL.
static constexpr int RAW_LINE_MAX = 104;
extern char rawLog[LOG_LINES][RAW_LINE_MAX];
extern uint32_t rawLogHead;
extern uint32_t lastSentenceMs;

static constexpr int NUM_SENTENCE_TYPES = 9;
extern const char *SENTENCE_TYPES[NUM_SENTENCE_TYPES];
extern uint32_t sentenceTypeCounts[NUM_SENTENCE_TYPES];

// On-screen sentence filter. GSV is the bulk of the traffic (one message per
// four satellites, per constellation, every second), so hiding it turns the
// log from a flood into something readable.
//
// This is a *view* control only: SD logging, the TCP stream, and the GSV/GSA
// parsing that feeds the sky plot all continue to see every sentence.
// Mutated from loop() under stateMutex; read by gpsTask under the same lock.
enum class NmeaFilter : uint8_t { ALL, NO_GSV, POSITION };
static constexpr int NMEA_FILTER_COUNT = 3;
extern uint8_t nmeaFilter;

const char *nmeaFilterLabel();
bool sentencePassesFilter(const char *type);

// The two sinks that only gpsTask ever touches: the SD log and the TCP
// fan-out queue. Deliberately called by the caller *outside* stateMutex --
// an SD write is the slowest thing in the ingest path, and nothing on the
// render side can see either sink.
void logSentence(const char *line);

// Everything here touches state the render task reads, so it must be called
// holding stateMutex.
void handleRawSentence(const char *line, int len);
