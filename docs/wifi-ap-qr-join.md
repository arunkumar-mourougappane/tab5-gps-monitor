# Wi-Fi AP QR join — research and design

**Status: proposed, not implemented.** This is research and a design sketch for turning the top-bar AP
status pill into a button that opens an overlay with the AP's QR join code, connection details, and a
manual on/off toggle. The interaction is prototyped in [`docs/index.html`](index.html) (tap the AP badge,
or **Jump to state → Wi-Fi QR**) so the shape of it can be reviewed before any firmware work starts. See
[`ARCHITECTURE.md`](ARCHITECTURE.md) for how the real touch/render pipeline this would plug into is
structured, and [`DECISIONS.md`](DECISIONS.md) for the project's engineering history.

## Motivation

The Wi-Fi AP (`Tab5-GPS`, WPA2, TCP port 10110 for raw NMEA — see `src/io/wifi_nmea.cpp`) is currently
join-by-typing: SSID and password live in the firmware source, and there's no on-panel way to see them
short of reading the code or knowing them from memory. A phone camera scanning a QR code is faster and
doesn't require the SSID/password to be communicated out of band. The AP pill already sits in the top bar
showing live client count — it's the natural place to hang this off of.

## What the overlay shows

Mocked up in `docs/index.html`'s `drawWifiOverlay()`:

- **SSID / password / address** as plain text (`Tab5-GPS` / `gpstest123` / `192.168.4.1:10110`).
- **A QR code** encoding a standard Wi-Fi join payload (below), scannable by a phone's camera app
  directly — no separate QR scanner needed on current iOS/Android.
- **Connected client count**, reusing the same `nmeaClientCount` the badge itself already shows.
- **A toggle chip** (`DISABLE AP` / `ENABLE AP`) to turn the radio off without going through sleep, and a
  **DONE** chip to close, matching the brightness overlay's `OFF`/`DONE` pair.
- When the AP is off: the QR/detail fields are replaced with a muted "AP is off" placeholder rather than
  showing stale or blank values.

One explicit non-goal, called out in the overlay's own hint text: **scanning only joins the Wi-Fi
network.** It doesn't open a TCP connection to port 10110 — there's no standard URI scheme a phone's camera
app will hand off to an NMEA client for that. The address is shown as text specifically so a user can type
it into whatever TCP client they're using once they've joined.

## QR payload format

The [Wi-Fi network config URI format](https://github.com/zxing/zxing/wiki/Barcode-Contents#wifi-network-config-android)
that Android and iOS's camera apps both recognize:

```
WIFI:T:WPA;S:<ssid>;P:<password>;;
```

For this AP: `WIFI:T:WPA;S:Tab5-GPS;P:gpstest123;;`. Special characters in the SSID/password (`;`, `,`,
`"`, `\`, `:`) need backslash-escaping per the same spec if either ever stops being a fixed compile-time
constant — not a concern today (`WIFI_AP_SSID`/`WIFI_AP_PASS` in `src/io/wifi_nmea.cpp` are both plain
ASCII with no special characters), but worth a comment at the call site if this gets built so a future
SSID/password change doesn't silently produce an unscannable code.

## Generating the QR code

**In the mockup:** there's no QR library loaded (the page is a self-contained static file — see
`README.md`'s note that `docs/index.html` has no external dependencies). Since the join string is a fixed
compile-time constant, the QR module matrix was generated once, offline, with Python + [segno](https://pypi.org/project/segno/)
(version 3, error correction M, 29×29 modules) and embedded directly as a `WIFI_QR_BITS` array of 29
binary strings in `docs/index.html`, rendered as a grid of absolutely-positioned `<i>` elements inside a
white quiet-zone box. This was checked end-to-end by rendering the page, screenshotting the code, and
decoding it back with OpenCV's `QRCodeDetector` — it reproduces `WIFI:T:WPA;S:Tab5-GPS;P:gpstest123;;`
exactly. If the SSID or password ever changes, the embedded matrix needs regenerating the same way.

**In firmware,** the SSID and password are still compile-time constants today, so the same
precompute-once approach would work — but a real QR *library* is the better call there, both because a
future version might want to encode something that isn't fixed at compile time (a random per-boot AP
password, say) and because M5GFX has no QR support of its own to lean on. Two candidates, both MIT-licensed
and both commonly used in ESP32/Arduino projects:

| Library | Footprint | Fit |
|---|---|---|
| [ricmoo/QRCode](https://github.com/ricmoo/QRCode) | ~350 lines, no heap allocation — caller supplies the output buffer | Purpose-built for microcontrollers; the usual choice in M5Stack example code. **Recommended.** |
| [nayuki/QR-Code-generator](https://github.com/nayuki/QR-Code-generator) (C version) | Larger, more configurable (explicit control over version/mask/ECC selection) | Worth it only if the fixed defaults ricmoo's library picks turn out to be wrong for this use (unlikely at this payload size) |

For a 37-character payload like this one, error correction level M lands on a version-3 code (29×29
modules) either way — this was confirmed empirically via segno while building the mockup, not assumed.
Buffer size at that version is small (`qrcode_getBufferSize(3)` = 106 bytes for ricmoo's library); flash
cost for the library itself is on the order of a few KB. Against the current build's 1.19MB / 18.7% flash
usage (see `DECISIONS.md`'s build section), this is noise.

**Rendering the matrix on the M5GFX canvas:** no library support needed — walk the module grid and
`canvas.fillRect()` each set bit at `moduleSize` pixels square, same idea as `qrModulesHTML()` in the
mockup. Needs a `moduleSize` chosen with the quiet zone (4 modules minimum per the spec) and physical
scan distance in mind — see below.

## Open question: will it actually scan on hardware

The Tab5's panel is 1280×720 on a ~5" diagonal (~294 PPI). A 29-module code needs the quiet zone counted
in its footprint (29 + 2×4 = 37 modules total). The mockup renders it at 260px including quiet zone
(≈7px/module), which on real hardware is a ~22mm code — small for a phone camera at a comfortable arm's
length, though generous single-App scanning at 10-15cm is normally fine down to similar sizes. This
wasn't validated against real hardware or a real phone camera as part of this research — **flagged as the
first thing to check before shipping this**, along with whether the panel's glass/coating introduces any
glare that would compound a small code. If it doesn't scan reliably at a natural distance, the fix is a
larger `moduleSize` and a correspondingly larger overlay panel, which the current 680×460 mockup panel
has headroom for (it doesn't fill the 1280×720 screen).

## Design tension: the AP badge isn't a fixed-position hit target

`LIGHT`/`SLEEP` are simple to hit-test because they're fixed: `lightBtnRect()`/`sleepBtnRect()` in
`src/ui/ui_status_bar.cpp` return constants. The AP badge is different — it's drawn by `drawDotBadge()`,
right-aligned and *chained* leftward off whatever's drawn before it (battery %, `RECEIVING`/`NO DATA`),
so its x-position shifts whenever a neighboring pill's label changes width. `ui_status_bar.h` calls this
out explicitly as the reason the LIGHT/SLEEP chips were deliberately kept off that row in the first place:

> Deliberately not chained onto the right-hand badge row: those pills resize with their labels, which
> would shift these buttons' hit rects around under the user's finger.

That objection was written with a *drag* interaction in mind (the brightness slider), where a rect
shifting mid-gesture would be genuinely bad. A single tap-to-open is more forgiving — the badge's rect
only needs to be correct for the frame it was tapped on, since `drawStatusPill()` already redraws it at
~5Hz (the render pipeline's poll rate — see `ARCHITECTURE.md`'s render cycle section), well ahead of any
plausible tap. Recommended approach: have `drawStatusPill()` capture the AP pill's rect (it already computes
the x-position while chaining) into a module-level `Rect apBadgeRect_` the way `lightBtnRect()` is a
constant, and have `hitTestChips()` in `app_input.cpp` read it the same way it reads the fixed rects. The
mockup's equivalent, `apBadgeHit()` in `docs/index.html`, does the analogous thing by reading the *live
DOM* rect of the badge element — not applicable to the canvas firmware directly, but confirms the
underlying idea (measure where it actually landed, don't hardcode where it should be) works in practice.

## Design decision: manual toggle vs. sleep's automatic teardown

`wifiEnabled` (`include/io/wifi_nmea.h`) is currently written from exactly one place outside
`wifi_nmea.cpp` itself: `src/power/power.cpp`, which sets it `false` on `enterSleep()`/`enterBacklightOff()`
and `true` again on wake. Adding a manual toggle means a second writer to the same flag, and the two need
to interact sensibly. Two options:

1. **Naive:** the manual toggle also just writes `wifiEnabled` directly. Simple, but means turning the AP
   off, then sleeping and waking, silently turns it back on — surprising, and arguably worse than not
   having the toggle at all.
2. **Recommended:** add a separate `bool wifiUserDisabled = false;` (mirrors the shape of
   `positionView`/`logExpanded` — plain UI-state booleans already living next to the input/power code) that
   only the toggle chip writes. `wifiEnabled` becomes computed rather than directly assigned:
   `wifiEnabled = !asleep && !backlightOff && !wifiUserDisabled;`, evaluated wherever `power.cpp` currently
   assigns it. A manual "off" then survives a sleep/wake cycle, matching what a user who just turned it off
   would expect.

The mockup's `UI.wifiOn` is the option-2 shape (independent of `UI.asleep`, combined only at display/badge
time via `apUp = UI.wifiOn && !UI.asleep`), specifically to demonstrate this rather than the naive version.

## Security note

`WIFI_AP_SSID`/`WIFI_AP_PASS` are already plaintext constants in `src/io/wifi_nmea.cpp`, and the AP itself
already broadcasts the SSID — anyone within range can already see the network name and attempt to join.
Showing the password as text and as a QR code on-screen doesn't expose anything that wasn't already
recoverable from the firmware source or the binary, but it does add a new *casual* exposure path (anyone
who can see the screen over the user's shoulder now has the password in about one second, versus needing
to go read source). Worth a one-line callout during implementation review; not a blocker — this is a demo
receiver on a fixed default password already, not a deployed secret.

## What's in the mockup vs. what's still open

Implemented in `docs/index.html` (all under `WIFI`/`apOpen`/`wifiOn` — grep for "Proposed, not yet
implemented in firmware" to find every touch point):

- Tappable AP badge, opening a modal overlay in the same style as the brightness overlay.
- A real, scannable QR code (precomputed matrix, verified by decode).
- SSID/password/address/client-count display, with an "AP is off" state.
- A manual on/off toggle chip, decoupled from sleep in the same way recommended above for firmware.
- `DONE` / tap-outside-panel to close, `Jump to state → Wi-Fi QR` in the dev harness for quick access.

Open for the real implementation:

- Validate QR scan distance on actual Tab5 hardware with a real phone camera (see above).
- Land the `apBadgeRect()`-caching approach in `ui_status_bar.cpp`/`app_input.cpp`.
- Land the `wifiUserDisabled` latch in `power.cpp`/`wifi_nmea.cpp`.
- Pick and vendor a QR library (`ricmoo/QRCode` recommended) into `platformio.ini`'s `lib_deps`.
- Decide the overlay's exact on-device pixel geometry (the mockup's 680×460 panel is a starting point, not
  a final answer) and add it to the `computeLayout()`/`layout.h` family alongside `dimmerPanelRect()`.
