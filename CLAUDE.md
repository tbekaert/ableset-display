# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Notes for anyone (human or coding assistant) working on this repo: what the code does, the constraints that shaped it, and the traps already hit once.

## What this is

Two Arduino sketches for Wi-Fi stage displays that follow [AbleSet](https://ableset.app) over OSC. Each shows the current song, the current section in its AbleSet colour with a progress bar, and what comes next. Both run on the ESP32-C5 (2.4 + 5 GHz Wi-Fi).

```
esp32-c5-lcd-1.47/ableset_display_147/        Waveshare ESP32-C5-LCD-1.47 (320x172)
esp32-c5-touch-lcd-3.5/ableset_display_35/    Waveshare ESP32-C5-Touch-LCD-3.5 (480x320, touch)
docs/                                          screenshots used by README.md
```

Each sketch is a single `.ino` file plus `secrets.example.h`.

## Secrets: never commit Wi-Fi credentials

- Networks and passwords live in `secrets.h` next to each sketch. It is git-ignored.
- `secrets.example.h` is the committed template. Keep its placeholder values.
- Before committing, check that no real SSID or password made it into a tracked file: `git diff --cached`, and `git grep` for anything that looks like one.
- When compiling in a scratch copy, copy `secrets.example.h` to `secrets.h` there.

## Building

Toolchain:

- **esp32 core 3.3.10 exactly.** Waveshare tested these boards on it. 3.3.12 failed to flash the 3.5" board.
- **GFX Library for Arduino** (moononournation) 1.6.x.
- **U8g2** (olikraus), 3.5" only, for its Logisoso fonts.

arduino-cli. On this machine there is no standalone install: use the one
bundled with the Arduino IDE, with the IDE's config so it finds the cores.
The esp32 core is installed as `esp32:esp32` (`arduino-cli core list` to check).

```sh
alias arduino-cli='"/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli" --config-file ~/.arduinoIDE/arduino-cli.yaml'

# 1.47" — default partition. The sketch is about 87% of 1.25 MB; if it outgrows
# it, use PartitionScheme=min_spiffs.
arduino-cli compile --fqbn esp32:esp32:esp32c5:CDCOnBoot=cdc \
  esp32-c5-lcd-1.47/ableset_display_147

# 3.5" — needs PSRAM for the framebuffer, and a bigger app partition
arduino-cli compile \
  --fqbn esp32:esp32:esp32c5:CDCOnBoot=cdc,PSRAM=enabled,PartitionScheme=huge_app \
  esp32-c5-touch-lcd-3.5/ableset_display_35
```

Check the option names with `arduino-cli board details -b esp32:esp32:esp32c5`. Some core installs don't offer `huge_app`; `min_spiffs` or a No-OTA scheme also fits.

### Arduino prototype generation (bites every time)

The Arduino builder generates a prototype for every function and inserts them all just before the **first function definition** in the file.

- Any struct or enum used in a function signature (`OscOut`, `OscArg`, `SectProgress`, `FitResult`, `NextContent`, ...) must be declared above the first function. That's why the "OSC TYPES" block sits high up in both sketches.
- Don't use default arguments. The generated prototype repeats them and the build fails. Pass every argument explicitly.
- `BOOT_PIN` is already defined by the core. The BOOT button is `RETRY_BTN_PIN` (GPIO28).

## Hardware

### ESP32-C5-LCD-1.47

- ESP32-C5 with 4 MB flash and **no PSRAM**, so no framebuffer: it draws straight to the panel.
- ST7789 172x320, used in landscape (rotation 1) with offsets 34/0.
- LCD pins: CLK 7, MOSI 6, CS 23, DC 24, RST 26, backlight 10 (PWM).
- WS2812 LED on GPIO8 (off by default, `USE_RGB_LED`).
- BOOT button on GPIO28. No touch.
- Uses the built-in 5x7 GFX font, scaled. It is ASCII only, so `displayText()` folds accents (é → E).

### ESP32-C5-Touch-LCD-3.5

- 8 MB PSRAM, 32 MB flash.
- ST7796 320x480 over SPI: SCK 6, MOSI 7, CS 8, DC 5. Runs at 60 MHz. `ROTATION = 1` is correct (confirmed on hardware).
- I2C on SDA 27 / SCL 26, shared by:
  - CH32V003 IO expander at 0x24 (registers MODE 0x02, OUTPUT 0x03, PWM 0x05). It owns the touch reset (EXIO0), LCD reset (EXIO1) and panel power (EXIO5), plus the backlight PWM. Init order matters: expander first, then LCD reset, then the panel. See `exioInit()`.
  - FT6336 touch at 0x38. Only the touch count is read; any touch means "Retry".
  - AXP2101 PMIC (left at its defaults).
- 300 KB framebuffer (`Arduino_Canvas`) in PSRAM. Only changed rectangles are pushed (`pushRect`).
- U8g2 Logisoso fonts with `setUTF8Print(true)`. Arduino_GFX's `getTextBounds()` reports zero width for non-ASCII glyphs; `measureForm()` swaps accented letters for ASCII look-alikes before measuring.

## Architecture (both sketches)

### Receiving OSC

- One UDP socket (lwIP `socket()`), bound to `LOCAL_PORT` 8000.
- `oscRxTask` (priority 5) blocks in `recvfrom()`. For each packet it takes `stateLock`, parses into `net`, and stamps `lastPacketMs`.
- `loop()` calls `takeSnapshot()` to copy `net` into `st` under the lock, then draws only from `st`.
- `loop()` sends on the same socket. lwIP is built with `LWIP_NETCONN_FULLDUPLEX`, so that is safe. AbleSet always sees requests from port 8000.

### Why it's built this way

Earlier versions polled `WiFiUDP` from `loop()`. A UDP socket's receive mailbox holds only 6 packets, so while a full-screen redraw was running AbleSet's bursts overflowed it and updates were silently lost. Rules that follow:

- Never block or draw in the receive path. Keep work under `stateLock` short.
- Redraw only what changed: compare against `drawn` and the `drawn*` signature strings.
- `Serial.setTxTimeoutMs(0)`: the USB-CDC serial port otherwise blocks up to about 2 s per write when the host isn't reading (for example, powered from a laptop with no serial monitor open).
- Read `lastPacketMs` through `packetAgeMs()`. It reads the timestamp before the clock, so a packet landing in between can't make the age wrap around.
- Avoid `AsyncUDP`. It splits large datagrams, and the setlist reply is large.

### Talking to AbleSet

AbleSet OSC port: 39051. Docs: https://ableset.com/docs/osc

- **`/subscribe port name F filters…`** subscribes. With no IP, AbleSet replies to the sender's address, so a subnet broadcast works before AbleSet has been found. Its address is learnt from the first reply. mDNS (`_osc._udp`) is only a hint.
- **`/getValues port`** returns everything. It is sent once after subscribing.
- **`sendResync()`** sends `/getValues addr…` every 3 s for the small values only (`RESYNC_VALUES`). This heals missed updates.
- **Not in the resync:**
  - `beatsPosition`: AbleSet streams it constantly.
  - `remainingTimeInSet`.
  - `setlist/songs` and `setlist/songDescriptions`: large, so loaded at subscribe and when they change.
- **`/getHeartbeat`** every 2 s. AbleSet counts as gone after 10 s of silence (`ABLESET_TIMEOUT_MS`).
- **Values used:**
  - Song and section: song name and index, section name and colour, next section, next song.
  - Loop state: loop flag, counter, loop count.
  - Progress: `activeSectionStart`/`End`, `global/beatsPosition`, `global/timeSignature`.
  - Laptop power: `computer/onBattery`.
  - 3.5" only: `setlist/songs`, `setlist/songDescriptions`, `activeSongDescription` and `remainingTimeInSet`. Descriptions keep their case and are cut to their first line (`cleanDesc()`); `SHOW_SONG_DESCRIPTIONS` switches the setlist between 5 two-line rows and 9 one-line rows.
- **Colours** are matched by name against a hand-picked palette (`PALETTE`), with an HSV fallback from the hex value. Colour logs are printed only when the resolved colour changes.
- **No "stop" value.** AbleSet doesn't expose setlist stop markers over OSC (`activeSongStopDescription` is null), and project scripts can only read OSC values, so they can't provide one either.
- **Name tags.** Song and section names can carry tags in braces (`Song {Drop}`). `cleanName()` strips them for display.

### Network handling

- **Network list.** The sketch tries the configured networks in order (`wifiRotate`). Each network costs one try, whether it fails to connect, is out of range, or connects but has no AbleSet within 30 s (`ABLESET_HUNT_MS`).
- **Giving up.** After `WIFI_ROUNDS × N_WIFI` tries, `giveUp()` turns the radio off and shows the Retry screen. BOOT (or a touch on the 3.5") calls `retryNow()`.
- **Dropouts.** A dropped network is retried first, before moving to the next one. Finding AbleSet resets the try budget.

## Testing

- **Compile both sketches** with arduino-cli (see Building) after any change. Watch the flash percentage on the 1.47".
- **Layout preview without hardware.** Compile the drawing part of a sketch on the host against Arduino_GFX's `Arduino_Canvas`, using minimal `Arduino.h` / `String` / `Print` stubs. Dump the framebuffer (RGB565) and convert it to PNG. The images in `docs/` were made that way. The harness is not checked in; rebuild it in the scratchpad when needed.
- **No automated tests.** There is no test runner, linter or formatter; a clean compile of both sketches is the check.
- **Real test:** run AbleSet on the same network, play through a set with loops, and watch the Serial Monitor at 115200 baud. It logs scans, joins, AbleSet discovery, colour changes and section boundaries.

## Flashing tips

- **Download mode.** If the upload can't connect: hold BOOT, tap RST, release BOOT.
- **Avoid deep USB hub chains.** Monitor hubs and some docks make the C5's native USB (USB-Serial-JTAG, GPIO13/14) show up as a generic CDC device that won't sync. Plug the board into the computer or a simple hub.

## Style

- Keep each sketch a single self-contained `.ino`. Arduino IDE users should only need the libraries above.
- Comments explain *why*: the hardware or AbleSet quirk behind the code.
- Keep the two sketches' networking code in step. They share the same receive task, resync, retry and give-up logic. Port fixes to both.
