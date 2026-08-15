#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>
#include "power.h"
#include "display.h"
#include "layout.h"
#include "wifi_nmea.h"

bool backlightOff = false;
bool asleep = false;
uint8_t savedBrightness = 128;

// ------------------------------------------------ panel brightness via DCS --
// M5.Display.setBrightness() is a no-op on Tab5: Panel_Device::setBrightness()
// forwards to a Light instance and Tab5's init path never attaches one, while
// Panel_DSI/ST7123/ST7121 implement no brightness of their own and neither IO
// expander carries a backlight pin. The one remaining route is the panel's own
// DSI command channel: DCS 0x51 (set_display_brightness), which only takes
// effect once 0x53 (write_control_display) has enabled the brightness block.
//
// write_params() is protected, so it's reached through a derived type that adds
// no members and therefore shares the base layout -- the same reinterpret_cast
// idiom M5GFX itself uses in getPanel(). The type is never instantiated.
struct DsiBrightnessAccess : public lgfx::Panel_DSI {
  bool writeDcs(uint32_t cmd, const uint8_t *data, size_t len) { return write_params(cmd, data, len); }
};

bool panelSetBrightness(uint8_t level) {
  auto *p = M5.Display.getPanel();
  if (p == nullptr) return false;
  auto *acc = reinterpret_cast<DsiBrightnessAccess *>(p);
  uint8_t ctrl = 0x2C; // BCTRL | DD | BL -- enable the brightness control block
  acc->writeDcs(0x53, &ctrl, 1);
  return acc->writeDcs(0x51, &level, 1);
}

static void rememberBrightness() {
  uint8_t b = M5.Display.getBrightness();
  if (b > 0) savedBrightness = b; // never latch 0, or wake would restore to dark
}

// NOTE: M5.Display.setBrightness() is a no-op on Tab5 with M5GFX 0.2.26 --
// Panel_Device::setBrightness() forwards to a Light instance, and Tab5's init
// path never attaches one (Panel_DSI/ST7123/ST7121 implement no brightness
// either, and neither IO expander carries a backlight pin). The calls are kept
// so this does the right thing if a future M5GFX wires one up, but the visible
// effect comes from blanking the canvas to black and pushing that.
static void blankScreen() {
  canvas.fillScreen(0);
  canvas.pushSprite(&M5.Display, 0, 0);
}

// The backlight really does go off at 0: M5GFX attaches no Light instance for
// Tab5, so M5.Display.setBrightness() does nothing and the DCS 0x51 write
// above is the only control there is. It works -- confirmed on hardware by
// the panel visibly glowing when this was briefly set to BRIGHTNESS_MIN
// instead (see ui_dimmer.h for that constant).
//
// That brief detour was an attempt to explain a sleeping screen that would not
// wake on a tap, on the theory that the ST7123 -- one controller for both the
// display and the digitiser -- might stop reporting touch with its display
// block idled. It is a real possibility: M5GFX's own Tab5 timing block warns
// that shrinking the vertical front porch "will cause the touch panel to stop
// working". But the same commit also fixed the wake path itself, which needed
// a press AND a release both seen through a 20ms sampling window, and that is
// the likelier reason a tap did nothing.
//
// So: dark again, with the wake fixes kept. If tapping a sleeping screen stops
// working, the "touch while dark" line on the serial monitor says which half is
// at fault -- and the targeted next step is clearing the BL bit in DCS 0x53
// (ctrl 0x28 rather than 0x2C), which switches the backlight off without
// touching the brightness value at all.
void enterBacklightOff() {
  rememberBrightness();
  backlightOff = true;
  M5.Display.setBrightness(0);
  panelSetBrightness(0);
  blankScreen();
}

// Deliberately zeroes the backlight rather than calling M5.Display.sleep().
// Tab5's newer panels use an integrated display+touch controller (ST7123 /
// ST7121), so putting the panel to sleep risks taking the touch digitiser with
// it -- which would make tap-to-wake impossible. The backlight is the dominant
// consumer anyway, and the radio going down is the other half of the saving.
void enterSleep() {
  rememberBrightness();
  asleep = true;
#if ENABLE_WIFI_NMEA
  wifiEnabled = false; // wifiTask tears the AP down and powers the radio off
#endif
  M5.Display.setBrightness(0);
  panelSetBrightness(0); // see enterBacklightOff() for why this is 0 again
  blankScreen();
}

void wakeDisplay() {
  if (asleep) {
#if ENABLE_WIFI_NMEA
    wifiEnabled = true; // wifiTask brings the AP back up
#endif
    asleep = false;
  }
  backlightOff = false;
  M5.Display.setBrightness(savedBrightness);
  panelSetBrightness(savedBrightness);
  relayout(); // force a full repaint and full push
}
