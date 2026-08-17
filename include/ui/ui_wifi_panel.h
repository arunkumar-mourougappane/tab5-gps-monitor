#pragma once
#include "core/config.h"
#include "core/layout.h"

#if ENABLE_WIFI_NMEA

// -------------------------------------------------------- wifi AP QR overlay --
// Modal panel opened from the AP status pill: the AP's SSID/password/address
// as text, a QR code encoding a standard Wi-Fi join payload, live connected-
// client count, and a manual on/off toggle independent of sleep's own AP
// teardown (see io/wifi_nmea.h's wifiUserDisabled). Design writeup:
// docs/wifi-ap-qr-join.md.

extern bool apOpen;
extern bool apDirty;

Rect wifiPanelRect();
Rect wifiQrRect();
Rect wifiToggleRect();
Rect wifiDoneRect();

// Computes the QR module matrix once -- the join string is a compile-time
// constant, so there's no reason to re-encode it every time the overlay
// opens. Call once from setup(), before the panel can ever be reached.
void initWifiPanel();

void drawWifiPanel();

#endif // ENABLE_WIFI_NMEA
