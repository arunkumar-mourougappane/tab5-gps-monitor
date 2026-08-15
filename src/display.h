#pragma once
#include <M5Unified.h>

// Off-screen frame buffer: every draw* function renders into this (in PSRAM)
// instead of the physical panel, and loop() blits the finished frame in one
// pushSprite() call. Without this, each fillRect-then-redraw step is visible
// on the panel as a blank flash before the new content lands. Constructed in
// main.cpp; every drawing module includes this header to reach it.
extern M5Canvas canvas;
