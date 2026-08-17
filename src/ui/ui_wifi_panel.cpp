#include "ui/ui_wifi_panel.h"

#if ENABLE_WIFI_NMEA

#include <cmath>
#include <WiFi.h>
#include <qrcode.h>
#include "core/display.h"
#include "core/theme.h"
#include "ui/ui_widgets.h"
#include "io/wifi_nmea.h"
#include "input/touch_input.h"

bool apOpen = false;
bool apDirty = false;

// Centred, same (SCREEN_W - w) / 2 pattern as dimmerPanelRect() -- geometry
// carried over from the docs/index.html prototype and provisional: the QR's
// physical size on real hardware needs a real phone-camera scan test before
// this is more than a first guess (see docs/wifi-ap-qr-join.md).
Rect wifiPanelRect() {
  constexpr int w = 680, h = 460;
  return {(SCREEN_W - w) / 2, (SCREEN_H - h) / 2, w, h};
}

Rect wifiQrRect() {
  Rect p = wifiPanelRect();
  return {p.x + 30, p.y + 60, 260, 260};
}

Rect wifiToggleRect() {
  Rect p = wifiPanelRect();
  return {p.x + 30, p.y + p.h - 62, 190, 44};
}

Rect wifiDoneRect() {
  Rect p = wifiPanelRect();
  return {p.x + p.w - 30 - 150, p.y + p.h - 62, 150, 44};
}

// The join string is fixed at compile time, so the QR is encoded once here
// rather than per frame. Version 3 (29x29 modules) / ECC_MEDIUM was confirmed
// against this exact string while writing docs/wifi-ap-qr-join.md -- if
// WIFI_AP_SSID/WIFI_AP_PASS ever change, a longer string may need a higher
// version (qrcode_initText() below will simply fail if version 3 is too
// small, rather than silently truncating).
static constexpr uint8_t WIFI_QR_VERSION = 3;
static QRCode wifiQr;
static uint8_t wifiQrBuffer[128]; // qrcode_getBufferSize(3) == 106; padded for headroom

void initWifiPanel() {
  char payload[96];
  snprintf(payload, sizeof(payload), "WIFI:T:WPA;S:%s;P:%s;;", WIFI_AP_SSID, WIFI_AP_PASS);
  qrcode_initText(&wifiQr, wifiQrBuffer, WIFI_QR_VERSION, ECC_MEDIUM, payload);
}

static void drawQrCode(const Rect &box) {
  auto &d = canvas;
  d.fillRoundRect(box.x, box.y, box.w, box.h, 6, COLOR_WHITE);

  int modules = wifiQr.size;
  constexpr int quietModules = 4; // spec minimum
  float m = (float)box.w / (modules + quietModules * 2);
  int x0 = box.x + (int)(quietModules * m), y0 = box.y + (int)(quietModules * m);
  int cell = (int)ceilf(m); // ceil rather than floor: a hairline gap between
                            // modules reads as noise to a camera; slight
                            // overlap does not.
  for (int ry = 0; ry < modules; ry++) {
    for (int cx = 0; cx < modules; cx++) {
      if (qrcode_getModule(&wifiQr, cx, ry)) {
        d.fillRect(x0 + (int)(cx * m), y0 + (int)(ry * m), cell, cell, TFT_BLACK);
      }
    }
  }
}

void drawWifiPanel() {
  auto &d = canvas;
  Rect p = wifiPanelRect();
  bool apUp = wifiEnabled;

  d.fillRoundRect(p.x, p.y, p.w, p.h, CARD_RADIUS, COLOR_CARD_BG);
  d.drawRoundRect(p.x, p.y, p.w, p.h, CARD_RADIUS, COLOR_ACCENT);

  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(p.x + 30, p.y + 22);
  d.print("WI-FI ACCESS POINT");

  Rect qr = wifiQrRect();
  if (apUp) {
    drawQrCode(qr);
  } else {
    d.drawRoundRect(qr.x, qr.y, qr.w, qr.h, 6, COLOR_DIVIDER);
    d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
    d.setCursor(qr.x + 20, qr.y + qr.h / 2 - 16);
    d.print("AP is off --");
    d.setCursor(qr.x + 20, qr.y + qr.h / 2 + 4);
    d.print("enable to let a phone join");
  }

  int colX = qr.x + qr.w + 30;
  char addrBuf[32];
  if (apUp) snprintf(addrBuf, sizeof(addrBuf), "%s:%u", WiFi.softAPIP().toString().c_str(), NMEA_TCP_PORT);
  else strlcpy(addrBuf, "--", sizeof(addrBuf));
  int clients = apUp ? nmeaClientCount : 0;
  char clientsBuf[16];
  snprintf(clientsBuf, sizeof(clientsBuf), "%d client%s", clients, clients == 1 ? "" : "s");

  const char *rows[4][2] = {
    {"SSID", apUp ? WIFI_AP_SSID : "--"},
    {"PASSWORD", apUp ? WIFI_AP_PASS : "--"},
    {"ADDRESS", addrBuf},
    {"CONNECTED", clientsBuf},
  };
  for (int i = 0; i < 4; i++) {
    int y = qr.y + 4 + i * 66;
    d.setFont(&fonts::Font2);
    d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
    d.setCursor(colX, y);
    d.print(rows[i][0]);
    d.setFont(&fonts::Font4);
    d.setTextColor(COLOR_TEXT_PRIMARY, COLOR_CARD_BG);
    d.setCursor(colX, y + 18);
    d.print(rows[i][1]);
  }

  d.setFont(&fonts::Font2);
  d.setTextColor(COLOR_TEXT_SECONDARY, COLOR_CARD_BG);
  d.setCursor(qr.x, qr.y + qr.h + 20);
  d.print("Scanning joins the Wi-Fi network only --");
  d.setCursor(qr.x, qr.y + qr.h + 40);
  d.print("the NMEA stream still needs its own TCP client.");

  drawChip(wifiToggleRect(), apUp ? "DISABLE AP" : "ENABLE AP", COLOR_TOPBAR_BG, COLOR_TEXT_PRIMARY,
           pressTarget == PressTarget::AP_TOGGLE);
  drawChip(wifiDoneRect(), "DONE", COLOR_TOPBAR_BG, COLOR_TEXT_PRIMARY, pressTarget == PressTarget::AP_DONE);
}

#endif // ENABLE_WIFI_NMEA
