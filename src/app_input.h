#pragma once

// -------------------------------------------------------------- app input --
// The touch-driven finite-state controller: samples touch, handles the
// dark-screen wake path and the modal brightness overlay, and dispatches
// taps against the three cards. Called once per loop() iteration.
//
// Returns false when the frame is fully handled here (dark-screen tap
// consumed, or the dimmer modal drew and pushed its own overlay) -- the
// caller should skip runRenderCycle() entirely in that case. Returns true
// otherwise, with want{Fix,Sat,Log}/pressChanged set for runRenderCycle().
bool handleTouch(bool &wantFix, bool &wantSat, bool &wantLog, bool &pressChanged);
