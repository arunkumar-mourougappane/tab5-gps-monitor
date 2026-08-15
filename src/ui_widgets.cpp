#include "ui_widgets.h"
#include "theme.h"
#include "trip_stats.h" // SPARK_LEN, for the ring-buffer wraparound below

// Draws label above value, auto-shrinking the value if it would overflow `w`.
// Returns the y position just below the drawn value.
int drawHeroValue(int x, int y, int w, const char *label, const char *value, uint8_t bigSize) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(label);
  y += d.fontHeight() + 4;

  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setTextSize(bigSize);
  if (d.textWidth(value) > w) d.setTextSize(1);
  d.setCursor(x, y);
  d.print(value);
  y += d.fontHeight() + 2;
  return y;
}

void drawMiniStat(int x, int y, const char *label, const char *value) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(x, y);
  d.print(label);
  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(x, y + 20);
  d.print(value);
}

int drawBadge(int x, int y, const char *text, uint16_t bg, uint16_t fg) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  int h = 30;
  int w = d.textWidth(text) + 28;
  d.fillRoundRect(x, y, w, h, h / 2, bg);
  d.setTextColor(fg, bg);
  d.setCursor(x + 14, y + (h - d.fontHeight()) / 2);
  d.print(text);
  return w;
}

void drawCardFrame(const Rect &r, const char *label, uint16_t accent) {
  auto &d = canvas;
  d.fillRoundRect(r.x, r.y, r.w, r.h, CARD_RADIUS, COLOR_CARD_BG);
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(r.x + 18, r.y + 11);
  d.print(label);
  d.drawFastHLine(r.x + 18, r.y + CARD_HEADER_H, r.w - 36, accent);
}

void drawChip(const Rect &c, const char *label, uint16_t bg, uint16_t fg, bool pressed,
              const lgfx::IFont *font) {
  auto &d = canvas;
  uint16_t fill = pressed ? COLOR_ACCENT : bg;
  uint16_t text = pressed ? COLOR_BG : fg;
  d.fillRoundRect(c.x, c.y, c.w, c.h, c.h / 2, fill);
  d.setFont(font);
  d.setTextSize(1);
  d.setTextColor(text, fill);
  d.setCursor(c.x + (c.w - d.textWidth(label)) / 2, c.y + (c.h - d.fontHeight()) / 2);
  d.print(label);
}

void drawSparkline(int x, int y, int w, int h, const float *values, int count, uint32_t head,
                    uint16_t color, float maxVal) {
  auto &d = canvas;
  d.fillRect(x, y, w, h, COLOR_BG);
  if (count < 2) return;

  int prevX = 0, prevY = 0;
  for (int i = 0; i < count; i++) {
    int idx = (int)((head - count + i) % SPARK_LEN);
    if (idx < 0) idx += SPARK_LEN;
    float norm = maxVal > 0 ? constrain(values[idx] / maxVal, 0.0f, 1.0f) : 0;
    int px = x + (int)((float)i / (count - 1) * w);
    int py = y + h - (int)(norm * h);
    if (i > 0) d.drawLine(prevX, prevY, px, py, color);
    prevX = px;
    prevY = py;
  }
}
