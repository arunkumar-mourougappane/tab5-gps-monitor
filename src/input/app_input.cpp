#include "input/app_input.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "core/display.h"
#include "input/touch_input.h"
#include "core/layout.h"
#include "power/power.h"
#include "ui/ui_dimmer.h"
#include "ui/ui_status_bar.h"
#include "ui/ui_log_panel.h"
#include "ui/ui_fix_panel.h"
#include "ui/ui_sky_panel.h"
#include "ui/ui_wifi_panel.h"
#include "io/wifi_nmea.h"
#include "model/nmea_parser.h"
#include "model/render_snapshot.h"
#include "render/render_pipeline.h"

// Which chip the press *started* on. A button fires on release of the press
// that began inside it, rather than on a fresh hit test of the lift position:
// re-testing the lift meant a finger that drifted a few pixels off the chip
// lit the highlight and then did nothing at all, which reads as a button that
// only works sometimes.
static PressTarget capturedTarget = PressTarget::NONE;

static PressTarget hitTestChips(int x, int y) {
  if (dimmerOpen) {
    if (pointInRect(x, y, dimmerOffRect())) return PressTarget::DIM_OFF;
    if (pointInRect(x, y, dimmerDoneRect())) return PressTarget::DIM_DONE;
    return PressTarget::NONE;
  }
#if ENABLE_WIFI_NMEA
  if (apOpen) {
    if (pointInRect(x, y, wifiToggleRect())) return PressTarget::AP_TOGGLE;
    if (pointInRect(x, y, wifiDoneRect())) return PressTarget::AP_DONE;
    return PressTarget::NONE;
  }
#endif
  if (pointInRect(x, y, lightHitRect())) return PressTarget::LIGHT;
  if (pointInRect(x, y, sleepHitRect())) return PressTarget::SLEEP;
  if (pointInRect(x, y, nmeaFilterHitRect())) return PressTarget::FILTER;
#if ENABLE_WIFI_NMEA
  if (pointInRect(x, y, apBadgeHitRect())) return PressTarget::AP;
#endif
  return PressTarget::NONE;
}

bool handleTouch(bool &wantFix, bool &wantSat, bool &wantLog, bool &pressChanged) {
  wantFix = wantSat = wantLog = false;
  pressChanged = false;

  // Touch edge detection, done here rather than via wasClicked()/wasReleased()
  // -- see sampleTouch()'s own comment for why.
  sampleTouch();
  bool pressing = touchPressing;
  bool pressEdge = touchPressEdge;
  bool released = touchReleaseEdge;
  touchPressEdge = false;
  touchReleaseEdge = false;

  // Everything below hit-tests the touch-DOWN position, not the lift position.
  // A fingertip rolls as it leaves the glass, so the last sample before a lift
  // can sit several pixels from where the user actually aimed -- and it is the
  // aim that should decide.
  if (pressEdge) capturedTarget = hitTestChips(touchDownX, touchDownY);

  // While the screen is dark, a tap only restores it and is consumed -- you
  // can't aim at a control you can't see, so no other target should fire.
  // Note GPS ingestion, SD logging and trip stats all keep running here; only
  // the display work is skipped.
  if (asleep || backlightOff) {
    // Wake on the press, not the lift. Waiting for a full press-release cycle
    // meant both edges had to be observed through a 20ms sampling window; the
    // press alone is unambiguous, and there is nothing else a tap could mean
    // while the screen is dark.
    if (pressEdge || released) {
      capturedTarget = PressTarget::NONE; // the waking tap commits nothing else
      wakeDisplay();
      return false;
    }
    // Escape hatch: if the panel stops reporting touch while dark, the power
    // button still gets the display back.
    if (M5.BtnPWR.wasClicked()) {
      capturedTarget = PressTarget::NONE;
      wakeDisplay();
      return false;
    }
    // Whether the digitiser reports anything at all while dark is the one
    // thing that cannot be reasoned about off-device, so say so on the wire.
    static uint32_t lastDarkLogMs = 0;
    if (touchPressing && millis() - lastDarkLogMs > 1000) {
      lastDarkLogMs = millis();
      Serial.printf("touch while dark: %d,%d\n", touchLastX, touchLastY);
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // nothing is visible, so ease off the CPU
    return false;
  }

  // Highlight whichever chip the finger is currently over, and drop it on lift.
  // The highlight shows the captured chip for the whole press, rather than
  // re-testing the moving contact. That keeps it honest: what is lit is what
  // will fire on release, including when the finger has drifted off the chip.
  PressTarget prevPress = pressTarget;
  if (pressing) {
    pressTarget = capturedTarget;
  } else if (released) {
    pressTarget = PressTarget::NONE;
  }
  pressChanged = pressTarget != prevPress;

  // The brightness overlay is modal: it owns touch and drawing while open, so
  // the panels underneath can't paint over it.
  if (dimmerOpen) {
    // A press that starts on the slider grabs it, and keeps it until the lift.
    // Only X matters from then on: testing the live position against the track
    // each sample meant any vertical drift mid-slide stopped the value dead,
    // which reads as the slider sticking.
    if (pressEdge && pointInRect(touchDownX, touchDownY, dimmerTrackHitRect())) {
      dimmerDragging = true;
    }
    if (dimmerDragging && pressing) {
      if (setBrightnessFromTouch(touchLastX)) dimmerDirty = true;
    }
    if (released) dimmerDragging = false;
    if (pressChanged) dimmerDirty = true;

    if (released) {
      PressTarget hit = capturedTarget;
      capturedTarget = PressTarget::NONE;
      if (hit == PressTarget::DIM_OFF) {
        dimmerOpen = false;
        enterBacklightOff(); // keeps the level just chosen for the next wake
        return false;
      }
      if (hit == PressTarget::DIM_DONE || !pointInRect(touchDownX, touchDownY, dimmerPanelRect())) {
        dimmerOpen = false;
        relayout(); // repaint whatever the overlay was covering
        return false;
      }
      dimmerDirty = true;
    }

    if (dimmerDirty) {
      dimmerDirty = false;
      drawDimmer();
      dirtyCount = 0;
      markDirty(dimmerPanelRect());
      pushDirty(false);
    }
    return false;
  }

#if ENABLE_WIFI_NMEA
  // The AP overlay is modal too, same shape as the brightness one above --
  // no drag to handle, so it's the simpler of the two.
  if (apOpen) {
    if (pressChanged) apDirty = true;

    if (released) {
      PressTarget hit = capturedTarget;
      capturedTarget = PressTarget::NONE;
      if (hit == PressTarget::AP_TOGGLE) {
        wifiUserDisabled = !wifiUserDisabled;
        applyWifiEnabled();
        apDirty = true;
      } else if (hit == PressTarget::AP_DONE || !pointInRect(touchDownX, touchDownY, wifiPanelRect())) {
        apOpen = false;
        relayout(); // repaint whatever the overlay was covering
        return false;
      }
    }

    if (apDirty) {
      apDirty = false;
      drawWifiPanel();
      dirtyCount = 0;
      markDirty(wifiPanelRect());
      pushDirty(false);
    }
    return false;
  }
#endif

  // Each branch marks only the panel(s) it actually affects. A satellite tap
  // only needs the sat panel redrawn; the fix/log cards are untouched by it.
  // A view/expand toggle calls relayout(), which repaints the whole canvas'
  // static chrome, so all three panel *contents* need repainting afterward
  // regardless of which card was tapped.
  // The touch-DOWN position, not the lift position: a fingertip rolls as
  // it leaves the glass, and the aim is what should decide.
  int tx = touchDownX, ty = touchDownY;
  if (released && tx >= 0 && ty >= 0) {
    // Chips commit on the press that began inside them, so a small drift
    // before the lift no longer swallows the tap.
    PressTarget chip = capturedTarget;
    capturedTarget = PressTarget::NONE;

    // Scoping the hit test to skyCard keeps the enlarged pick radius from
    // reaching across the gap into a neighbouring card.
    bool inSky = !logExpanded && pointInRect(tx, ty, skyCard);
    int hit = inSky ? hitTestSkyDot(tx, ty) : -1;
    if (chip == PressTarget::LIGHT) {
      dimmerOpen = true;
      dimmerDirty = true;
      return false; // the modal block draws it on the next pass
    } else if (chip == PressTarget::SLEEP) {
      enterSleep();
      return false;
#if ENABLE_WIFI_NMEA
    } else if (chip == PressTarget::AP) {
      apOpen = true;
      apDirty = true;
      return false; // the modal block draws it on the next pass
#endif
    } else if (hit >= 0) {
      setSatTooltip(hit);
      wantSat = true;
    } else if (inSky) {
      // Tapped the sky plot but not a satellite: close any open tooltip so
      // the tap still produces visible feedback instead of reading as dead.
      if (satTooltipActive()) {
        clearSatTooltip();
        wantSat = true;
      }
    } else if (!logExpanded && pointInRect(tx, ty, fixCard)) {
      positionView = positionView == PositionView::LIVE ? PositionView::TRIP : PositionView::LIVE;
      relayout();
      wantFix = wantSat = wantLog = true;
    } else if (chip == PressTarget::FILTER) {
      // Checked before the logCard branch -- the chip sits inside it.
      // Clearing the ring under the lock gives immediate feedback and
      // avoids showing a mix of pre- and post-filter sentences.
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      nmeaFilter = (nmeaFilter + 1) % NMEA_FILTER_COUNT;
      rawLogHead = 0;
      for (auto &l : rawLog) l[0] = '\0';
      xSemaphoreGive(stateMutex);
      relayout();
      wantFix = wantSat = wantLog = true;
    } else if (pointInRect(tx, ty, logCard)) {
      logExpanded = !logExpanded;
      relayout();
      wantFix = wantSat = wantLog = true;
    }
  }

  return true;
}
