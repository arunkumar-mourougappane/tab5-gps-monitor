#pragma once

// -------------------------------------------------------------- SD logging --
// SD_MMC needs no manual pin config: the m5stack_tab5 board variant defines
// BOARD_HAS_SDMMC/BOARD_SDMMC_SLOT, so SD_MMC.begin() picks them up itself.

extern bool sdReady;

void initSdLogging();

// Appends one NMEA line to the session's .nmea file, flushing periodically.
// No-op if sdReady is false.
void logToSD(const char *line);

// Writes and flushes one row (already formatted, trailing newline included)
// to the session's track .csv file. No-op if sdReady is false.
void writeTrackRow(const char *row);
