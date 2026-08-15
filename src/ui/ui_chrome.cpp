#include "ui/ui_chrome.h"
#include "core/display.h"
#include "core/theme.h"
#include "core/layout.h"
#include "ui/ui_widgets.h"

// Forward-declared rather than included from ui_log_panel.h / ui_fix_panel.h:
// those modules don't exist as separate translation units yet -- PositionView
// and positionView are still defined in main.cpp.
void drawFilterChip();
enum class PositionView : uint8_t { LIVE, TRIP };
extern PositionView positionView;

// Header text doubles as the touch-affordance hint, and only needs to be
// repainted when a view toggles -- so drawStaticChrome() is called again on
// every touch transition rather than every render tick.
void drawStaticChrome() {
  auto &d = canvas;
  d.fillScreen(COLOR_BG);

  d.fillRect(0, 0, SCREEN_W, TOPBAR_H, COLOR_TOPBAR_BG);
  d.drawFastHLine(0, TOPBAR_H, SCREEN_W, COLOR_DIVIDER);
  d.setFont(&fonts::Font4);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_TOPBAR_BG);
  d.setCursor(MARGIN, (TOPBAR_H - d.fontHeight()) / 2);
  d.print("TAB5 GPS MONITOR");

  if (!logExpanded) {
    drawCardFrame(fixCard, positionView == PositionView::LIVE ? "POSITION  (tap: trip stats)" : "TRIP STATS  (tap: live)",
                  COLOR_ACCENT);
    drawCardFrame(skyCard, "SATELLITES  (tap a dot for detail)", COLOR_ACCENT);
  }
  drawCardFrame(logCard, logExpanded ? "NMEA STREAM  (tap to collapse)" : "NMEA STREAM  (tap to expand)",
                COLOR_ACCENT_GREEN);

  drawFilterChip(); // sits in the log card header, at its right end
}
