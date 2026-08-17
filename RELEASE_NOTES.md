## tab5-gps-monitor v1.1.0

Adds Wi-Fi AP join by QR code, a hidden diagnostics overlay, and version tracking, on top of v1.0.0's GPS/BDS monitor for the [M5Stack Tab5](https://docs.m5stack.com/en/core/Tab5).

### Highlights

- **Wi-Fi AP QR join**: the top-bar AP badge is now a button. Tapping it opens an overlay with the AP's SSID/password/address as text, a scannable QR code for joining the network directly, live connected-client count, and a manual on/off toggle for the radio that's independent of sleep's own AP teardown — turning it off no longer flips back on the next time the device wakes. Design writeup, including the QR library choice and the AP badge's hit-rect handling: [`docs/wifi-ap-qr-join.md`](docs/wifi-ap-qr-join.md).
- **Hidden diagnostics overlay**: a small on-device panel, reachable through a gesture on the top bar, surfacing build/version information alongside a status readout.
- **Version tracking**: `FIRMWARE_VERSION` (`config.h`) is the project's first version constant, kept in sync with the release tag via `scripts/bump-version.sh` as part of cutting a release.

### Fixed

- The AP overlay's connected-client count and the Wi-Fi AP's IP address now reflect real, confirmed radio state rather than the request to turn it on — opening the overlay right after enabling the AP no longer shows a stale `0.0.0.0`.
- The AP overlay's client count now updates live while it's open, instead of only on the next tap.

### Verified on hardware

- Wi-Fi AP QR join: the panel opens on tap, the QR code scans at a natural distance, and the manual on/off toggle behaves correctly across a sleep/wake cycle.
- The hidden diagnostics overlay's trigger gesture registers reliably.
