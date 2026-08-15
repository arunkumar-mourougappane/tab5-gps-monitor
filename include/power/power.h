#pragma once
#include <cstdint>

// ------------------------------------------------------------ power states --

// Screen-dark states. Both are woken by a tap anywhere; the difference is that
// sleep also powers the radio down and puts the panel itself to sleep rather
// than only zeroing the backlight.
//
// Neither stops GPS ingestion or SD logging: for a logger, "screen off" means
// keep recording in your pocket, not stop working.
extern bool backlightOff;
extern bool asleep;
extern uint8_t savedBrightness;

// Sets the panel's backlight via a DCS write -- the only brightness control
// that exists on this hardware. Returns false if the panel object isn't
// reachable.
bool panelSetBrightness(uint8_t level);

void enterBacklightOff();
void enterSleep();
void wakeDisplay();
