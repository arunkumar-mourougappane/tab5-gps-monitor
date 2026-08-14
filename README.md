# tab5-gps-monitor
A esp32-p4 (Tab5) application to integrate m5stack GPS/BDS Unit with SMA Antenna to demonstrate GPS Monitor

## Hardware

- [M5Stack Tab5](https://docs.m5stack.com/en/core/Tab5) — ESP32-P4, dual-core RISC-V @ 360MHz + LP single-core @ 40MHz, 16MB flash, 32MB Octal PSRAM
- [GPS/BDS Unit with SMA Antenna (AT6668)](https://docs.m5stack.com/en/unit/Unit-GPS%20SMA) — connect to the Tab5's HY2.0-4P port (Port.A): GND, 5V, TX → G54, RX → G53. Outputs NMEA0183 at 115200 baud.

## Building

This project targets the ESP32-P4, which isn't yet supported by upstream `platformio/platform-espressif32`, so `platformio.ini` uses the [pioarduino](https://github.com/pioarduino/platform-espressif32) community fork (per M5Stack's own Tab5 docs).

```sh
pio run -j 1         # build (see note below on -j 1)
pio run -t upload    # flash
pio device monitor   # serial monitor
```

`-j 1` (serial build) is intentional: parallel builds on this platform version intermittently fail with `cannot find lib*/M5GFX/.../*.cpp.o` linker errors — a library variant-dir race in the SCons LDF, reproduced locally on ~1 in 5 clean parallel builds. If you hit it, rerunning `pio run -j 1` reliably succeeds.

If `pio run` fails while packaging the firmware image with a `click`/`esptool` `TypeError`, your PlatformIO install is running on a system Python whose globally installed `click` is too new for the bundled esptool. Fix with:

```sh
python3 -m pip install --user "click<8.2"
```

## Architecture

- `src/gps_monitor.cpp` reads and parses NMEA sentences (via [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)) on a dedicated FreeRTOS task pinned to core 1, so serial ingestion never blocks display rendering.
- `src/main.cpp` renders the latest GPS fix (via [M5Unified](https://github.com/M5Stack/M5Unified)) on a throttled ~5Hz UI loop.
