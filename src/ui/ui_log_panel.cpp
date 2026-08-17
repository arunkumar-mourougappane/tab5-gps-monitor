#include "ui/ui_log_panel.h"
#include "core/display.h"
#include "core/theme.h"
#include "ui/ui_widgets.h"
#include "model/nmea_parser.h"
#include "input/touch_input.h"

// Fixed size so the hit target matches the drawn chip regardless of label, and
// so the chip can't resize under a changing label the way the status badges did.
static constexpr int FILTER_CHIP_W = 92;
static constexpr int FILTER_CHIP_H = 26;

Rect nmeaFilterChipRect() {
  return {logCard.x + logCard.w - 18 - FILTER_CHIP_W, logCard.y + (CARD_HEADER_H - FILTER_CHIP_H) / 2,
          FILTER_CHIP_W, FILTER_CHIP_H};
}

// The chip is only 26px tall, well under a fingertip. Its target is padded to
// 44 and reaches slightly into the card body; that region belongs to the log
// card, which is why the filter is hit-tested before it -- a near miss used to
// expand the whole card instead, which is a worse outcome than doing nothing.
Rect nmeaFilterHitRect() {
  Rect c = nmeaFilterChipRect();
  return {c.x - 16, c.y - 5, c.w + 24, c.h + 18};
}

// Its fixed size means the self-contained fill fully covers the previous
// state -- no clear needed.
void drawFilterChip() {
  bool filtering = (NmeaFilter)nmeaFilter != NmeaFilter::ALL;
  drawChip(nmeaFilterChipRect(), nmeaFilterLabel(), filtering ? COLOR_ACCENT_GREEN : COLOR_TOPBAR_BG,
           filtering ? COLOR_BG : COLOR_TEXT_SECONDARY, pressTarget == PressTarget::FILTER);
}

void drawLogPanel(const RenderSnapshot &s) {
  auto &d = canvas;
  Rect r = innerOf(logCard);
  d.fillRect(r.x, r.y, r.w, r.h, COLOR_CARD_BG);

  drawFilterChip(); // above innerOf(), but inside the logCard dirty rect

  // Expanded is the "read the sentences" view, so it gets larger type and the
  // full card width. The width matters: a long GSV sentence at Font4 runs to
  // roughly 980px, which would clip against the ~968px left column that the
  // sentence-type counters leave behind. Those counters stay in the collapsed
  // view, where the smaller font leaves width to spare.
  bool big = logExpanded;
  d.setFont(big ? &fonts::Font4 : &fonts::Font2);
  d.setTextSize(1);

  int rightW = big ? 0 : 220;
  int leftW = big ? r.w : r.w - rightW - 24;

  d.setTextColor(COLOR_ACCENT_GREEN, COLOR_CARD_BG);
  int rowH = d.fontHeight() + (big ? 6 : 5);
  int maxRows = r.h / rowH;
  int shown = min<uint32_t>(min<uint32_t>(s.logHead, LOG_LINES), (uint32_t)maxRows);

  // Oldest at the top, newest on the last row. Bottom-anchored so the newest
  // sentence sits on the final row even before the buffer has filled, instead
  // of the block hugging the top with a gap beneath it.
  int y = r.y + (maxRows - shown) * rowH;
  for (int i = 0; i < shown; i++) {
    int idx = (s.logHead - shown + i) % LOG_LINES;
    d.setCursor(r.x, y);
    d.print(s.logLines[idx]);
    y += rowH;
  }

  if (big) return; // no counter column in the expanded view

  int dividerX = r.x + leftW + 12;
  d.drawFastVLine(dividerX, r.y, r.h, COLOR_DIVIDER);

  // Right: cumulative sentence-type counts -- gives the wide log card a
  // purpose beyond the raw text, which rarely fills its own width.
  int cx0 = dividerX + 12;
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(cx0, r.y);
  d.print("SENTENCE TYPES");
  int cy = r.y + d.fontHeight() + 8;
  int colW = rightW / 2 - 6;
  int rowH2 = d.fontHeight() + 8;
  for (int i = 0; i < NUM_SENTENCE_TYPES; i++) {
    int col = i % 2, row = i / 2;
    int cxp = cx0 + col * (colW + 12);
    int cyp = cy + row * rowH2;
    char buf[16];
    snprintf(buf, sizeof(buf), "%-3s %lu", SENTENCE_TYPES[i], (unsigned long)s.typeCounts[i]);
    d.setTextColor(s.typeCounts[i] > 0 ? COLOR_TEXT_PRIMARY : COLOR_STATUS_NONE, COLOR_CARD_BG);
    d.setCursor(cxp, cyp);
    d.print(buf);
  }
}
