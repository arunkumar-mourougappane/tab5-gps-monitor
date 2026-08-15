#pragma once
#include "layout.h"

// -------------------------------------------------- render pipeline --
// Signature hashing + dirty-rect tracking: each panel redraws only when its
// own inputs change, and only the changed regions get pushed to the panel.

// Regions whose canvas pixels changed this frame, pushed individually so a
// push costs only the area that actually moved rather than the full screen.
static constexpr int MAX_DIRTY = 4;
extern int dirtyCount;
void markDirty(const Rect &r);

// Pushed in horizontal bands with a touch sample between them -- see pushDirty
// in render_pipeline.cpp for why. Exposed so app_input's modal dimmer overlay
// (which draws and pushes outside the normal poll cycle) can use the same path.
void pushDirty(bool full);

// Runs one poll of the render cycle: re-snapshots at ~5Hz (or immediately if a
// touch just changed UI state), hashes each panel's inputs, redraws and pushes
// whatever changed. want{Fix,Sat,Log} force a panel to redraw even if its
// signature didn't move (state a touch changed but a signature can't see, like
// which position/sat-tooltip/filter view is showing).
void runRenderCycle(bool wantFix, bool wantSat, bool wantLog, bool pressChanged);
