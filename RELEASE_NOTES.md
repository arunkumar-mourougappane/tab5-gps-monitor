## tab5-gps-monitor v1.0.0

First release: a GPS/BDS monitor for the [M5Stack Tab5](https://docs.m5stack.com/en/core/Tab5) (ESP32-P4), driving the [GPS/BDS Unit with SMA Antenna](https://docs.m5stack.com/en/unit/Unit-GPS%20SMA) over UART.

### Highlights

- Live position, altitude, speed, course and HDOP, with a 40-sample HDOP trend and per-fix-mode session timing.
- A satellite sky plot (elevation/azimuth/SNR) and a scrollable, filterable raw NMEA log, both touch-driven.
- SD card logging of raw NMEA and decoded track points, and a Wi-Fi AP (`Tab5-GPS`, TCP port 10110) fanning the sentence stream out to up to four clients — both keep running with the screen off.
- Touch controls for a position/trip toggle, satellite detail, brightness, and sleep, tuned against this panel's digitizer (ghost second contact, press-capture semantics).

### Performance

- Ingestion holds `stateMutex` per sentence, not per drain burst, and per-fix accounting (trip stats, HDOP/speed history) runs once per receiver fix rather than once per ~2 ms drain.
- The render path avoids `Arduino String` on the hot paths (NMEA parsing, PMIC reads, per-frame draw) and repaints only the panels whose signature actually changed, in touch-interruptible bands.
- Built at `-O2` (the framework defaults to `-Os`); `-flto` was tried and reverted — this platform's link step invokes `ld` directly, so the LTO plugin never loads. See [`docs/DECISIONS.md`](docs/DECISIONS.md).

### Architecture

- `main.cpp` is wiring only; the firmware is split into ~20 module pairs under `include/<topic>/` and `src/<topic>/` (`core`, `model`, `io`, `ui`, `input`, `render`, `power`).
- Full architecture, concurrency model, and sequence diagrams: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md). Engineering decisions, bugs found on hardware, and why: [`docs/DECISIONS.md`](docs/DECISIONS.md).
- [`docs/index.html`](docs/index.html) is a browser mockup of the panel, reproducing the real layout arithmetic, palette and touch map for reference without hardware.

### Fixed on hardware

- Touch chips now commit on the press that captured them, not a fresh hit-test of wherever the finger drifted to on release.
- The brightness slider tracks the finger continuously instead of losing the grab mid-drag.
- Sleep goes fully dark (previously left a dim residual image), and a press during sleep or backlight-off wakes the panel instead of being swallowed.

### Known limitations

- Parallel builds (`pio run` without `-j 1`) intermittently fail with a library variant-dir race in this PlatformIO platform version's SCons LDF (~1 in 5 clean builds); `-j 1` is required.
- Verified on real Tab5 hardware for the touch, brightness and sleep/wake fixes; the wider module split and `-O2` build are hardware-confirmed but have not had a long-duration field soak.
