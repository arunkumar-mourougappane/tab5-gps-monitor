#pragma once
#include <cstdint>

// ------------------------------------------------------------- touch input --
// Only ever touched from loop() (render side) -- no locking needed.

// Which chip the finger is currently down on, so it can render pressed. Held
// across frames (not just on release) so the highlight tracks the finger.
enum class PressTarget : uint8_t { NONE, LIGHT, SLEEP, FILTER, DIM_OFF, DIM_DONE, AP, AP_TOGGLE, AP_DONE };
extern PressTarget pressTarget;

extern bool touchPressing;
extern bool touchPressEdge;   // consumed by app_input's handleTouch()
extern bool touchReleaseEdge; // consumed by app_input's handleTouch()
extern int touchLastX, touchLastY; // current contact, for drags
extern int touchDownX, touchDownY; // where the press began, for hit tests

// Hoisted out of loop() so it can also run between the slices of a screen
// push (render_pipeline's pushRegion()) -- a full-canvas blit is the longest
// thing the render side does, and while it runs nothing polls the panel. A
// quick tap that started and ended inside one of those windows was simply
// never seen: the driver only reads the hardware when we call it, so there
// is no event left to recover afterwards.
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
void sampleTouch();
