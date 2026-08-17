#pragma once
#include "core/layout.h"

// ------------------------------------------------------------- ui chrome --
// Whole-screen static background: top-bar title and all three card frames.
// Called from layout.cpp's relayout() after a full repaint.
void drawStaticChrome();

// The title text's left portion of the top bar, up to where the LIGHT chip
// starts (TOPBAR_CTRL_X) -- a fixed rect like lightBtnRect(), since the title
// is only ever drawn once per relayout(), not per frame. Tapped seven times
// within three seconds, this opens the developer easter egg
// (app_input.cpp/ui_easter_egg.h) -- deliberately not run through the
// pressTarget highlight system, unlike the real chips, since a visible press
// state would give away exactly what it's counting.
Rect titleHitRect();
