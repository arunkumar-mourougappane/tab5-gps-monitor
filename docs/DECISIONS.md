# Decisions, bugs, and why

An engineering log for the Tab5 GPS monitor firmware: what was found, what was tried, what was kept, and what was deliberately stepped away from. Organized by theme rather than by date; each entry names the commit it landed in where one exists. For how the pieces fit together today, see [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Performance

The firmware started as a single 2,456-line `src/main.cpp`. Before any of the architecture work, four performance issues were found and fixed in the ingestion path — all four still apply verbatim to the split codebase, just in different files now.

### Arduino `String` on the hot path

**Found:** every NMEA sentence — roughly 50 a second — went through `String::operator+=` one character at a time to assemble the line, three `substring()` calls in `handleRawSentence()`, and one heap-allocated `String` per CSV field in `splitCSV()` (up to 24 per sentence). All of it ran inside `stateMutex`.

**Fixed** (`f1d41f0`): fixed `char` buffers end to end. `gpsTask` assembles into a `char[104]`; `splitCSV()` splits in place, writing `NUL` over each comma and returning pointers into the same buffer, so tokenizing a sentence allocates nothing. The raw-log ring became `char[40][104]` instead of `String[40]`, which also turned the snapshot's 40 `String` assignments into one `memcpy` — a copy that runs five times a second while holding the mutex `gpsTask` needs.

### Per-burst work mistaken for per-fix work

**Found** (`bf52956`): the block guarded by `gps.location.isValid()` ran after every UART drain. `isValid()` stays true for as long as a fix exists, so it fired on every *burst*, not on every *fix*. `gpsTask` wakes every 2 ms; a second of sentences arriving over ~50 ms produces on the order of 25 bursts. Consequence: ~25 track-CSV rows and ~25 SD flushes a second where the file was meant to hold one row per fix, and the 40-sample HDOP sparkline covered under two seconds of history instead of the last 40 fixes.

**Fixed:** all per-fix work now runs once per *fix epoch*, keyed on the receiver's own UTC timestamp. `TinyGPSPlus::isUpdated()` would be the obvious signal but isn't usable here — reading a value clears its flag, and `captureSnapshot()` reads every one of them from the render task five times a second, so the flags can't be trusted as an ingestion-side signal.

### Lock held around the whole drain

**Found** (`43d8bf0`): the mutex used to span an entire UART drain — reading the port, parsing every sentence in it, the SD writes for each, the track-CSV row and its flush, and a dozen blocking `Serial.print` calls. `captureSnapshot()` blocks on the same mutex five times a second, so a slow SD card or a stalled USB CDC endpoint could delay rendering.

**Fixed:** reduced to exactly the state that's shared. Reading the UART and assembling a line take no lock at all. The lock is taken once per *completed sentence*, around `TinyGPSPlus::encode()` and `handleRawSentence()`, so the render task can interleave between sentences instead of waiting out a whole burst. `logSentence()` (the SD append and the TCP queue push) runs unlocked, before the lock is even taken. `printFix()` became `formatFix()`: it fills a buffer under the lock (every field is shared state) and the caller writes it to the port after releasing.

One bug caught *while doing the later module split*, not while doing this: the first pass of extracting `nmea_parser.cpp` called `logSentence()` from inside `handleRawSentence()`, which runs under the lock — that would have put the SD write straight back inside the critical section this commit removed it from. Caught before it was committed; `logSentence()` stays a separate call made by `gpsTask` outside the lock. Documented in `87376f9`'s commit message as a reminder that a mechanical refactor can silently reintroduce a fixed bug if the reason for the original shape isn't carried along with the code.

### Two smaller costs

**Found** (`92d29ef`): `M5.Power.getBatteryLevel()`/`isCharging()` are I2C round-trips to the PMIC. Both the status-bar signature (sampled every 200 ms) and the battery badge itself called them — ten I2C transactions a second for a value that moves on the order of minutes. Separately, every panel repaint built throwaway `String`s for its values (`String(lat, 6)`, `"AP " + String(count)`), each one a heap allocation.

**Fixed:** battery reads cached at 1 Hz behind `refreshBattery()`, read by both callers. The four draw helpers (`drawHeroValue`, `drawMiniStat`, `drawBadge`, `drawChip`) take `const char*`; callers `snprintf` into stack buffers instead.

## Bugs found on hardware, and what fixed them

Three touch/power bugs were reported after the panel was actually used, not caught by anything that compiles. Two were understood on the first pass; one took two attempts, and that second attempt is worth reading for how it went wrong before it went right.

### Buttons committed on the wrong position

**Symptom:** buttons were unreliable — the highlight would light up under a finger, and then nothing would happen.

**Cause** (`283b0f3`): `pressTarget` was computed while pressing, purely to draw the highlight. The *release* handler then ran a fresh hit-test against the *lift* coordinates. A target had to be hit twice — once on press to highlight, once again on release to fire — so a finger that drifted a few pixels before lifting (which is every real tap; a fingertip rolls as it leaves the glass) lit the chip and then committed nothing.

**Fixed:** a press now captures its target once (`capturedTarget`), and release commits whatever was captured — standard button semantics, not a re-test. The highlight shows the captured chip for the whole press rather than re-testing the moving contact, so what's lit is always what will fire. See the sequence diagram in [ARCHITECTURE.md § Touch input](ARCHITECTURE.md#touch-input-press-capture).

A second, unrelated fix landed in the same commit: the screen push now goes out in 120-row bands with a touch sample between them, because a full-canvas push moves 1.8 MB out of PSRAM with no DMA and blocks long enough that a tap landing entirely inside that window was never seen at all — the touch driver only reports what it's polled for. (The driver itself was never the bottleneck; `Touch_Class::update()` already polls at ~5 ms.)

### The brightness slider stuck

**Symptom:** dragging the brightness slider would sometimes just stop responding mid-drag.

**Cause** (`65ca210`): the drag handler updated the level only while the current contact position was inside the 26px-tall track rect, re-tested on every sample. The knob riding on that track is drawn at 34px — taller than the band it had to stay inside — so any vertical wander during a horizontal slide (which is every real slide) fell out of the band and the value stopped dead until the finger wandered back in.

**Fixed:** a press that starts on the slider grabs it and holds the grab until release; only the X coordinate matters from then on, so the value follows the finger anywhere on the panel, not just inside the original 26px band. The grab area itself is padded to 62px tall to make starting the grab easier too.

### Sleep wouldn't wake on a tap — two attempts

**Symptom:** tapping a sleeping screen didn't bring it back.

**First attempt** (`0950b51`): the working theory was that the brightness-zero DCS write was the cause. Tab5's panel (ST7123) is an integrated display-and-touch controller, and `enterSleep()` already avoided `M5.Display.sleep()` for exactly this reason — telling the display block to fully idle risked taking the digitizer down with it. So this commit changed two things at once: it stopped writing brightness 0 on sleep (used `BRIGHTNESS_MIN` instead), *and* it fixed the wake path itself, which had required a full press-and-release cycle observed through a 20 ms sampling window rather than waking on the press alone.

**What actually happened:** the brightness change was wrong, but it proved something useful. With the panel held at `BRIGHTNESS_MIN` instead of 0, the backlight was visibly glowing while "asleep" — which confirmed the DCS 0x51 write genuinely controls the backlight, something the code had only ever marked `SPECULATIVE`.

**Second attempt** (`9212bf8`): reverted the brightness change back to writing 0 on sleep (a real backlight-off matters more than a debugging signal), kept the wake-edge fix, and left one variable to actually test on hardware instead of guessing further: a serial line (`"touch while dark: x,y"`) that fires whenever a contact is detected while the screen is dark. If a tap still doesn't wake it, that line either appears (wake logic still wrong) or doesn't (the digitizer really does stop reporting when the backlight goes to zero — the theory that motivated the first attempt, still on the table and still unconfirmed). The next targeted step, if it comes to that, is written into the code as a comment: clear only the `BL` bit in DCS `0x53` (control byte `0x28` instead of `0x2C`) rather than zeroing the brightness value outright.

This one is the clearest example in this project of a fix landing in two pieces because the first pass changed two things to chase one symptom. The lesson kept: when a commit fixes something and you're not fully sure *which* part of the fix mattered, say so in the commit message rather than letting the uncertainty disappear — `0950b51`'s message flags exactly this, and it's what made the second attempt possible without re-deriving the reasoning from scratch.

## UI design iterations

### Where the HDOP trend belongs

The sparkline had one call site, the trip view — so the only accuracy trend anywhere on the panel was behind a tap, and the live view showed HDOP as a bare number with no way to tell whether a fix was settling or degrading. Moved (`3cc9915`) to sit directly under the HDOP value on the live view, in the 18px the PDOP/VDOP caption used to occupy — the fix card had 8px of slack, so something had to give, and the DOP trio (still parsed, still logged, still in the sentence stream) was the most redundant thing on the card. Fixed 0–5 scale rather than autoscaled, matching the trip view: a trace that autoscaled would make a steady poor fix look identical to a steady good one.

Vacating the trip view's sparkline left that face two-thirds empty (content ended at y=356 of a 482px interior) — filled (`bb93a05`) with a session panel the live view structurally can't provide: a speed trend autoscaled to the trip's own maximum, and a stacked "fix quality" bar showing time spent at 3D/2D/no-fix, since a logger's output is only as good as the fix it actually had, and the live view only ever shows the fix it has *right now*.

### Top-bar pill sizing: overshoot, then correction

The status badges started at 32px with 16px text — the smallest type on a 1280×720 panel, meant to be read at arm's length. First pass (`a4eae6e`) went to 42px with the larger Font4 face. That read as oversized once actually on screen. Second pass (`eea9457`) settled at 36px, roughly the midpoint, and deliberately kept the *text* at the smaller Font2 rather than compromising between the two — because there's nothing to compromise to: M5GFX's font ramp has an 8px face and then jumps straight to 26px, with nothing between. An intermediate size would mean pulling in a whole second typeface (`FreeSans9pt7b` or similar) for four labels, which wasn't judged worth it. The tradeoff is written down next to the constant in the source rather than left for the next person to rediscover.

### The sky-plot compass: from four letters to an HSI bezel

The compass around the satellite plot was the least-developed part of the panel: four 8px cardinal letters (the smallest text anywhere on the display), unlabelled elevation rings, no tick marks to read an azimuth against, and a course needle that simply vanished below 1 km/h as though the instrument had broken.

This one went through a proper design pass before implementation — two exploration pages were built and kept in `docs/` rather than thrown away: `sky-compass-options.html` compared five bezel treatments (current, an HSI-style tick ring, a "defects only" minimal fix, a heading tape, and a track-up rotation shown specifically to demonstrate why it doesn't work — rotating the plot would put every GSV azimuth, which is north-referenced, at odds with the drawn positions), and `hsi-needle-studies.html` measured six needle treatments by simulating how much of the satellite plot each one's needle physically covers at any instant.

That measurement is what decided it: a solid arrow needle sits on top of ~1.5 of 14 satellites at any moment; a compass-style needle, ~2.6. **Landed** (`edf3e8d`): an HSI-style bezel — 10°/30° tick marks, cardinals moved outside the ring at the larger face, a fixed north index, elevation rings labelled 30°/60° — baked into the pre-rendered `radarBg` sprite, so none of it costs anything per frame. Course rides the bezel as a chevron instead of crossing the plot as a needle, which sits on *zero* satellites regardless of heading, because nothing about it enters the circle at all. The bezel is paid for out of the plot radius (159px → 148px, capped by the card interior's height), and colour now carries meaning consistently across the whole panel: white is "reference that never moves," accent blue is "live course," and the three signal colours belong to satellites and nothing else.

## Build & tooling

### The partition table was silently wrong

**Found** (`5db5452`): the board JSON declares both a 16MB flash chip and the right partition table (`"arduino": {"partitions": "default_16MB.csv"}`), but PlatformIO doesn't read that field from the board definition — so the build was silently using the 4MB `default.csv` table instead, `app0`/`app1` at 1280K each, the whole table ending at `0x400000`. 12MB of the chip sat unaddressed, and the firmware was already at 90.4% of its undersized app partition.

**Fixed:** `board_build.partitions = default_16MB.csv` set explicitly in `platformio.ini`. Verified by decoding the actual generated `.pio/build/tab5/partitions.bin`, not by trusting the board JSON alone — `app0`/`app1` went to 6400K each, usage dropped to 18.0%. Moves every partition, so a device already flashed with the old table needs a full reflash rather than an OTA.

### `-Os` → `-O2`

The ESP32 Arduino build script hardcodes `-Os` (optimize for size) unconditionally, ahead of anything a project's `platformio.ini` sets — confirmed from the actual verbose compile line rather than assumed from general PlatformIO knowledge. Three levers were on the table: `src_build_flags` to raise only this project's own ~20 modules to `-O2` (smallest blast radius, smallest win, since the libraries doing the heavy lifting — M5GFX, TinyGPSPlus — would stay at `-Os`); `-flto` to recover the cross-translation-unit inlining the module split gave up (see below); or `-O2` project-wide via `build_unflags`/`build_flags`, which recompiles the framework and every library at the new level too.

**Landed** (`7f3e745`): `-O2` project-wide, chosen specifically as the option with the *least* certainty behind it, on the reasoning that it was worth finding out early rather than deferring. Measured with a full clean rebuild: flash 18.1% → 18.7% (+37KB of `.text`, well inside the 81% headroom), RAM unchanged, no new compiler warnings anywhere in the build — framework and libraries included. Confirmed working on physical hardware by the user afterward. What that measurement does *not* claim: a runtime speed improvement, or that nothing timing-sensitive (touch sampling, DMA-adjacent display code) behaves differently at the new level. Code size and a clean compile are what's actually verified from a development machine; only hardware can confirm the rest, and did.

### `-flto`: tried, and reverted before it was ever committed

The module split turned one translation unit into 21. Before the split, the compiler could inline small cross-module functions (`pointInRect`, `drawChip`, `refreshBattery`, and the like) for free, since everything lived in one file; after it, those became real function calls across a translation-unit boundary unless something recovers the inlining. `-flto` is the standard way to recover it — a whole-program optimization pass at link time.

Added to `build_flags` alongside `-O2` and tested with a full clean rebuild. It failed outright:

```
ld: .pio/build/tab5/src/main.cpp.o: plugin needed to handle lto object
ld: ...app_startup.c.obj): undefined reference to `app_main'
collect2: error: ld returned 1 exit status
```

**Cause:** this platform's final link step invokes `ld` directly rather than driving the link through `g++`. LTO's whole-program pass only happens if the *compiler* performs the final link, so it can load the LTO plugin and materialize the intermediate representation into real machine code; a bare `ld` invocation can't consume `-flto` object files at all, which is why every single object failed with "plugin needed" and the link collapsed into a cascade of undefined references. This is a structural mismatch between `-flto` and how this specific ESP-IDF-based build produces its final ELF, not a flag that was passed incorrectly.

**Decision: reverted, not worked around.** `-ffat-lto-objects` would make the linker fall back to plain (non-LTO) codegen it can actually read — but that means paying LTO's compile-time cost for zero cross-TU inlining benefit, which isn't a trade worth making. Actually fixing it would mean the platform driving its final link through `g++` instead of raw `ld`, which lives inside the pinned `pioarduino` platform package's own build script, not this project — patching a pinned third-party platform package is a materially different, larger-scope change than anything else touched here, and wasn't undertaken. `platformio.ini` carries `-O2` alone as a result; if the split's lost inlining turns out to matter in measured practice, the narrower fix on the table is marking the handful of small, hot, cross-module functions `inline` in their headers by hand, not chasing whole-program LTO on a build system that can't drive it.

## Architecture: from one file to a module tree

### Why split it

`src/main.cpp` reached 2,456 lines and 197 `static` declarations. It stayed readable because it was well-commented, but any single edit meant holding the whole ingestion/parsing/rendering/touch/power state machine in your head at once, and every change risked touching an unrelated concern by accident. The file's own banner comments already marked out its conceptual sections (`-- satellites --`, `-- GPS task --`, `-- dimmer overlay --`, and so on) — the split formalized those into real compilation units rather than inventing new boundaries.

### How, and how it was verified

Constraint going in: **pure code motion, zero behavior change.** No logic altered, only moved, with headers added so the pieces could see each other. Delivered as eight sequential commits (`1ea967f` through `e7ebe65`), leaf modules first (palette, config, layout) working up to the ones that depend on everything (touch input, the render scheduler, and finally `main.cpp` itself, which shrank to ~100 lines of wiring). Every single commit was verified with `pio run -j 1` before moving to the next, and flash usage was checked after each one specifically to catch code being duplicated rather than moved — it stayed flat at 18.1% (1,189,238 bytes) across the entire split. The final commit was checked with a full clean rebuild (`.pio/build/tab5` removed, rebuilt from nothing) to rule out incremental compilation masking a stale object file; it produced an identical binary.

Two structural decisions made along the way, not just mechanical extraction:

- `drawStaticChrome()` used to call `drawFilterChip()` internally — a whole-screen chrome function reaching into one specific panel's chip. Moved so `layout.cpp`'s `relayout()` calls both explicitly, back to back: same two draws, same order, but chrome no longer has to know a particular panel exists.
- `stateMutex` itself moved into `render_snapshot.h/.cpp`, since that module's whole job *is* the boundary the mutex protects — everything else takes it as an `extern`.

One real bug was caught mid-extraction and is covered above under *Performance*: the first pass of `nmea_parser.cpp` called the SD-writing `logSentence()` from inside the mutex-holding `handleRawSentence()`, which would have silently reintroduced the exact locking bug `43d8bf0` had already fixed. Caught and corrected before that commit landed.

### Directory layout

The 21 module pairs initially lived flat in `src/` — 42 files with no structure beyond a shared filename prefix. Reorganized (`650ff4d`) into the classic `include/`+`src/` split, with matching topic subdirectories on both sides (`core`, `model`, `io`, `ui`, `input`, `render`, `power` — the same seven groups the module-split commits already used).

Before moving a single file, the two PlatformIO behaviors this depends on were verified empirically rather than assumed: a throwaway header in `include/_probe_a/` and a `.cpp` in a nested `src/_probe_b/` confirmed that `include/` (with its full subtree) is on every compilation unit's search path by default, and that the default `src_filter` compiles every `.cpp` under `src/` recursively. Both probe files were deleted before touching the real tree. Every `#include "name.h"` was rewritten to `#include "topic/name.h"` in the same pass as the physical move, so nothing was left compiling against a stale flat path; git recognized all 42 moves as renames, each with a same-size content diff (the include line, nothing else). Verified again with a full clean rebuild: identical flash usage to before the move.
