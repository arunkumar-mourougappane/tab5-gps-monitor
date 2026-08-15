#include "input/touch_input.h"
#include "core/display.h"

PressTarget pressTarget = PressTarget::NONE;

bool touchPressing = false;
bool touchPressEdge = false;
bool touchReleaseEdge = false;
int touchLastX = -1, touchLastY = -1;
int touchDownX = -1, touchDownY = -1;

static bool touchWasDown = false;

void sampleTouch() {
  M5.update();
  auto touch = M5.Touch.getDetail(0);
  if (touch.isPressed()) {
    if (!touchWasDown) {
      touchPressEdge = true;
      touchDownX = touch.x;
      touchDownY = touch.y;
    }
    touchWasDown = true;
    touchPressing = true;
    touchLastX = touch.x;
    touchLastY = touch.y;
  } else {
    touchPressing = false;
    if (touchWasDown) {
      touchWasDown = false;
      touchReleaseEdge = true;
    }
  }
}
