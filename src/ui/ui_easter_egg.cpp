#include "ui/ui_easter_egg.h"
#include <cstring>
#include <cctype>
#include "core/config.h" // FIRMWARE_VERSION
#include "core/display.h"
#include "core/theme.h"
#include "ui/ui_widgets.h" // drawBadge()

bool eggOpen = false;
bool eggDirty = false;

// Geometry carried over from the docs/ mockup review, sized for the reveal
// phase's content (art + punchline + credits + version) -- the original
// 420x300 only fit the static art+punchline this replaces.
Rect eggPanelRect() {
  constexpr int w = 560, h = 460;
  return {(SCREEN_W - w) / 2, (SCREEN_H - h) / 2, w, h};
}

// -------------------------------------------------------- acquire sequence --
// Three steps, ~650ms each, mimicking the real POSITION card's own fixType
// language (see ui_fix_panel.cpp) before landing on the reveal. One-shot,
// not looping -- it plays once per open, not indefinitely.
struct AcquireStep { const char *fix; uint16_t color; const char *sats; const char *hdop; };
static const AcquireStep ACQUIRE_STEPS[3] = {
  {"NO FIX", COLOR_STATUS_BAD, "0/0", "--"},
  {"2D FIX", COLOR_STATUS_WARN, "6/16", "3.2"},
  {"3D FIX", COLOR_STATUS_GOOD, "14/16", "0.9"},
};
static constexpr uint32_t ACQUIRE_STEP_MS = 650;
static constexpr uint32_t TWINKLE_MS = 260;

static bool eggRevealed = false;
static uint8_t eggAcquireStep = 0;
static uint32_t eggPhaseMs = 0;

void eggReset() {
  eggRevealed = false;
  eggAcquireStep = 0;
  eggPhaseMs = millis();
}

void eggAdvance() {
  uint32_t now = millis();
  if (!eggRevealed) {
    if (now - eggPhaseMs >= ACQUIRE_STEP_MS) {
      eggPhaseMs = now;
      if (eggAcquireStep < 2) eggAcquireStep++;
      else eggRevealed = true;
      eggDirty = true;
    }
  } else if (now - eggPhaseMs >= TWINKLE_MS) {
    eggPhaseMs = now;
    eggDirty = true; // drawEasterEgg() re-rolls which stars are lit itself
  }
}

// -------------------------------------------------------- obfuscated credits --
// XOR obfuscation, not encryption: keeps the plain text out of source (no
// grep/GitHub-readable string) and out of a plain `strings` dump of the
// binary, but the key sits right here -- anyone disassembling the firmware
// can recover it. Not trying to be more than that.
static constexpr uint8_t OBFUSCATION_KEY = 0x42;

static const uint8_t NAME_BLOB[] = {
  0x03, 0x30, 0x37, 0x2C, 0x29, 0x37, 0x2F, 0x23, 0x30, 0x62, 0x0F, 0x2D,
  0x37, 0x30, 0x2D, 0x37, 0x25, 0x23, 0x32, 0x32, 0x23, 0x2C, 0x27,
};
static const uint8_t GITHUB_BLOB[] = {
  0x25, 0x2B, 0x36, 0x2A, 0x37, 0x20, 0x6C, 0x21, 0x2D, 0x2F, 0x6D, 0x23,
  0x30, 0x37, 0x2C, 0x29, 0x37, 0x2F, 0x23, 0x30, 0x6F, 0x2F, 0x2D, 0x37,
  0x30, 0x2D, 0x37, 0x25, 0x23, 0x32, 0x32, 0x23, 0x2C, 0x27,
};

static void decodeBlob(const uint8_t *blob, size_t len, char *out, size_t outSize) {
  size_t n = min(len, outSize - 1);
  for (size_t i = 0; i < n; i++) out[i] = (char)(blob[i] ^ OBFUSCATION_KEY);
  out[n] = '\0';
}

// -------------------------------------------------------------- drawing --
static void drawCenteredMono(int cx, int y, const char *text, uint8_t size) {
  auto &d = canvas;
  d.setFont(&fonts::Font0); // the one fixed-pitch face -- see the header comment
  d.setTextSize(size);
  d.setTextColor(COLOR_ACCENT_GREEN, COLOR_CARD_BG);
  d.setCursor(cx - d.textWidth(text) / 2, y);
  d.print(text);
}

static void drawCenteredText(int cx, int y, const char *text, uint16_t color, const lgfx::IFont *font) {
  auto &d = canvas;
  d.setFont(font);
  d.setTextSize(1);
  d.setTextColor(color, COLOR_CARD_BG);
  d.setCursor(cx - d.textWidth(text) / 2, y);
  d.print(text);
}

static void drawAcquirePhase(const Rect &p) {
  auto &d = canvas;
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(p.x + 30, p.y + 30);
  d.print("ACQUIRING SIGNAL...");

  const AcquireStep &s = ACQUIRE_STEPS[eggAcquireStep];
  int y = p.y + 66;
  drawBadge(p.x + 30, y, s.fix, s.color, COLOR_BG);

  d.setFont(&fonts::Font2);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(p.x + 190, y + 3);
  d.print("SATS");
  d.setCursor(p.x + 300, y + 3);
  d.print("HDOP");
  d.setFont(&fonts::Font4);
  d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
  d.setCursor(p.x + 190, y + 21);
  d.print(s.sats);
  d.setCursor(p.x + 300, y + 21);
  d.print(s.hdop);
}

// Font0/Font2/Font4 are all ASCII-only (space..~) -- confirmed while fixing
// the CRS degree sign (see DECISIONS.md) -- so every literal below is kept
// to that range on purpose. No middle-dot, no smart quotes.
static const char *ART_TEMPLATE[5] = {
  "   .   *   .",
  "  *  \\ | /  *",
  "----( SAT )----",
  "  *  / | \\  *",
  "   .   *   .",
};

static void drawRevealPhase(const Rect &p, int cx) {
  int y = p.y + 40;
  char rowBuf[20];
  for (int r = 0; r < 5; r++) {
    strlcpy(rowBuf, ART_TEMPLATE[r], sizeof(rowBuf));
    for (char *c = rowBuf; *c; c++) {
      if ((*c == '.' || *c == '*') && random(100) < 30) {
        *c = (*c == '.') ? '*' : '.';
      }
    }
    drawCenteredMono(cx, y, rowBuf, 2);
    y += 34;
  }

  y += 16;
  drawCenteredText(cx, y, "3D FIX: EASTER EGG LOCATED", COLOR_TEXT_PRIMARY, &fonts::Font2);
  y += 34;

  char nameBuf[32], ghBuf[48];
  decodeBlob(NAME_BLOB, sizeof(NAME_BLOB), nameBuf, sizeof(nameBuf));
  decodeBlob(GITHUB_BLOB, sizeof(GITHUB_BLOB), ghBuf, sizeof(ghBuf));

  // A fake NMEA sentence, but a real checksum -- the same XOR-of-the-body
  // rule every genuine sentence in the log panel is checked against.
  char body[40];
  snprintf(body, sizeof(body), "GPDEV,%s", nameBuf);
  for (char *c = body; *c; c++) *c = (char)toupper((unsigned char)*c);
  uint8_t ck = 0;
  for (const char *c = body; *c; c++) ck ^= (uint8_t)*c;
  char sentence[48];
  snprintf(sentence, sizeof(sentence), "$%s*%02X", body, ck);
  drawCenteredText(cx, y, sentence, COLOR_ACCENT_GREEN, &fonts::Font2);
  y += 30;

  drawCenteredText(cx, y, "BUILT BY", COLOR_TEXT_SECONDARY, &fonts::Font2);
  y += 20;
  drawCenteredText(cx, y, nameBuf, COLOR_TEXT_PRIMARY, &fonts::Font2);
  y += 24;
  drawCenteredText(cx, y, ghBuf, COLOR_ACCENT, &fonts::Font2);
  y += 32;

  char verBuf[48];
  // __DATE__ is compiler-provided ("Mon dd yyyy") -- no build-info infra
  // needed for this half; FIRMWARE_VERSION (config.h) is the hand-bumped
  // half, kept in sync with the release tag by scripts/bump-version.sh.
  snprintf(verBuf, sizeof(verBuf), "v%s - built %s", FIRMWARE_VERSION, __DATE__);
  drawCenteredText(cx, y, verBuf, COLOR_TEXT_SECONDARY, &fonts::Font2);
  y += 26;

  drawCenteredText(cx, y, "(tap anywhere to lose the signal)", COLOR_TEXT_SECONDARY, &fonts::Font2);
}

void drawEasterEgg() {
  auto &d = canvas;
  Rect p = eggPanelRect();

  d.fillRoundRect(p.x, p.y, p.w, p.h, CARD_RADIUS, COLOR_CARD_BG);
  d.drawRoundRect(p.x, p.y, p.w, p.h, CARD_RADIUS, COLOR_ACCENT_GREEN);

  if (!eggRevealed) drawAcquirePhase(p);
  else drawRevealPhase(p, p.x + p.w / 2);
}
