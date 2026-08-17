#pragma once
#include "core/layout.h"

// -------------------------------------------------------- developer easter egg --
// A tiny hidden gag screen, opened by tapping the top-bar title seven times
// within three seconds (app_input.cpp's tap counter, against
// ui_chrome.h's titleHitRect()). Purely cosmetic -- no functional content,
// nothing to configure. Closes on any tap, unlike the other modals: there's
// nothing here to choose between.

extern bool eggOpen;
extern bool eggDirty;

Rect eggPanelRect();

// Resets the acquire-sequence animation to its start -- call once, right
// when eggOpen is first set true.
void eggReset();

// Advances the acquire-sequence/twinkle animation by however much time has
// passed since the last call, setting eggDirty when something changed. This
// modal isn't on runRenderCycle()'s poll (handleTouch() returns false for
// as long as eggOpen is true), so app_input.cpp calls this by hand once per
// iteration while it's open -- the same reason the WiFi overlay hand-polls
// nmeaClientCount.
void eggAdvance();

void drawEasterEgg();
