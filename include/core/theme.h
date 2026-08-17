#pragma once
#include <cstdint>

// rgb565: quantises 8-bit RGB to the 16-bit format the panel actually stores.
constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static constexpr uint16_t COLOR_BG = rgb565(9, 11, 17);          // page background
static constexpr uint16_t COLOR_TOPBAR_BG = rgb565(16, 18, 27);  // top bar / badges
static constexpr uint16_t COLOR_CARD_BG = rgb565(21, 24, 35);    // card surface
static constexpr uint16_t COLOR_DIVIDER = rgb565(45, 50, 66);    // hairlines / grid rings
static constexpr uint16_t COLOR_TEXT_PRIMARY = rgb565(236, 239, 245);
static constexpr uint16_t COLOR_TEXT_SECONDARY = rgb565(138, 146, 166);
static constexpr uint16_t COLOR_ACCENT = rgb565(88, 168, 232);   // position card accent
static constexpr uint16_t COLOR_ACCENT_GREEN = rgb565(96, 210, 150); // log card accent
static constexpr uint16_t COLOR_STATUS_GOOD = rgb565(84, 214, 140);
static constexpr uint16_t COLOR_STATUS_WARN = rgb565(232, 188, 74);
static constexpr uint16_t COLOR_STATUS_BAD = rgb565(232, 92, 92);
static constexpr uint16_t COLOR_STATUS_NONE = rgb565(90, 96, 112);
// True white: QR codes need real light-on-dark contrast to scan reliably,
// not a themed near-white -- the only place this panel departs from its
// own palette on purpose.
static constexpr uint16_t COLOR_WHITE = rgb565(255, 255, 255);
