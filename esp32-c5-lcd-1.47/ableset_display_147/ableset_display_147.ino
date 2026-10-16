/*
 * AbleSet stage display — Waveshare ESP32-C5-LCD-1.47
 * -----------------------------------------------------
 * Connects to a WiFi network (2.4 or 5 GHz), subscribes to AbleSet's OSC
 * feed, and shows the current song, section and what's coming next on the
 * 1.47" LCD.
 *
 * Board:    Waveshare ESP32-C5-LCD-1.47 (ST7789, 172x320)
 * Requires: arduino-esp32 core 3.3.10  (Boards Manager: "esp32" by Espressif)
 *           GFX Library for Arduino 1.6.x (moononournation)
 *
 * Tools menu: Board = "ESP32C5 Dev Module", USB CDC On Boot = Enabled
 *
 * Shares its networking with the 3.5" version: OSC is received in its own
 * task so drawing never makes it drop packets, a light resync heals missed
 * updates, and after WIFI_ROUNDS passes through the network list it stops on
 * a Retry screen (press BOOT). Also shows a section progress strip and the
 * AbleSet laptop's power state.
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <lwip/sockets.h>
#include <math.h>
#include <Arduino_GFX_Library.h>

// ============================================================================
// CONFIG — edit this block
// ============================================================================

struct WifiNetwork { const char *ssid; const char *pass; };

// Your networks and passwords live in secrets.h (not committed to git).
// Copy secrets.example.h to secrets.h and fill it in before compiling.
#include "secrets.h"
static const int N_WIFI = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

static const uint32_t WIFI_TRY_MS   = 10000;  // per-network connect timeout
static const uint32_t WIFI_RETRY_MS = 10000;  // wait between reconnect attempts

// How many times to go through the whole network list looking for AbleSet
// before giving up and showing a Retry screen (press BOOT to try again).
static const int WIFI_ROUNDS = 2;

// Leave empty ("") to auto-discover AbleSet. Set a fixed IP if needed.
static const char *ABLESET_HOST = "";

static const uint16_t ABLESET_PORT = 39051;
static const uint16_t LOCAL_PORT   = 8000;

static const char *DEVICE_NAME = "AbleSet Display";   // shown in AbleSet's device list

// If your first pass through a loop reads "LOOP 0/4", set this to 1.
static const int LOOP_COUNTER_OFFSET = 0;

// Section band colour treatment (HSV-based, hue is never altered).
// BAND_SATURATION: multiplier on the colour's own saturation. 2.5 makes all
//                  of AbleSet's palette fully vivid.
// BAND_BRIGHTNESS: value channel, 0-255. 255 = as bright as the panel goes.
static const float   BAND_SATURATION = 2.5f;
static const uint8_t BAND_BRIGHTNESS = 255;

static const uint8_t  BACKLIGHT_PCT = 100;
static const bool     USE_RGB_LED   = false;
static const bool     SHOW_NEXT     = true;

// Section progress strip under the section name: true = the current bar
// fills up beat by beat, false = whole bars light up as they start.
// SHOW_PROGRESS = false hides the strip and gives the name the full band.
static const bool     SHOW_PROGRESS    = true;
static const bool     PROGRESS_BY_BEAT = true;

// ============================================================================
// DISPLAY
// ============================================================================

#define LCD_CS   23
#define LCD_DC   24
#define LCD_CLK   7
#define LCD_MOSI  6
#define LCD_RST  26
#define LCD_BL   10
#define RGB_LED   8
#define RETRY_BTN_PIN 28   // the BOOT button (BOOT_PIN is taken by the core)

// rotation 1 = landscape, 320 wide x 172 tall
// The `true` below is the IPS/invert flag. If black and white swap, flip it.
Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_CLK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 1 /* rotation */, true /* IPS */,
                                      172, 320, 34, 0, 34, 0);

static const int SCR_W = 320;
static const int SCR_H = 172;

// Zones
static const int Z_STATUS_Y = 0,   Z_STATUS_H = 20;
static const int Z_SONG_Y   = 22,  Z_SONG_H   = 74;
static const int Z_SECT_Y   = 98,  Z_SECT_H   = 50;
static const int Z_NEXT_Y   = 150, Z_NEXT_H   = 22;

// Progress strip along the bottom of the section band.
static const int PB_H = 5, PB_BOTTOM = 3, PB_X = 6, PB_W = SCR_W - 12, PB_GAP = 2;
static const int PB_Y = Z_SECT_Y + Z_SECT_H - PB_BOTTOM - PB_H;
static const int PB_MAX_SEGMENTS = 32;   // longer sections get one continuous bar
// Section name box: the band minus the strip.
static const int SECT_TEXT_H = SHOW_PROGRESS ? (PB_Y - 2 - Z_SECT_Y) : Z_SECT_H;

static const uint16_t COL_BG     = RGB565_BLACK;
static const uint16_t COL_TEXT   = RGB565_WHITE;
static const uint16_t COL_DIM    = 0x8410;
static const uint16_t COL_OK     = 0x07E0;
static const uint16_t COL_WARN   = 0xFD20;
static const uint16_t COL_ERR    = 0xF800;
static const uint16_t COL_LOOP   = 0xFFE0;  // loop row background (yellow)

// ============================================================================
// STATE
// ============================================================================

// Networking runs in its own task (see NETWORK RECEIVE TASK): incoming OSC is
// parsed the moment it arrives into `net`, and loop() renders from a snapshot
// (`st`) taken under a short lock. Drawing can't make us drop packets.
// One UDP socket bound to LOCAL_PORT: the receive task reads it, loop() sends
// from it (lwIP is built full-duplex, so that's safe).
int oscSock = -1;
SemaphoreHandle_t stateLock = nullptr;
IPAddress abletonIp;             // written under stateLock (see setAblesetIp)
volatile bool haveAblesetIp = false;
String connectedSsid = "";

struct State {
  String song      = "";
  String section   = "";
  String nextSect  = "";
  String nextSong  = "";
  String nextLine  = "";
  bool   inLoop    = false;
  int    loopCount = 0;
  int    loopTarget = 0;
  uint16_t sectCol = 0x2104;
  uint16_t nextSectCol = 0x2104;
  int songIndex    = -1;
  int songCount    = 0;
  bool playing     = false;
  bool liveLinked  = false;
  float beatPos    = -1;      // playhead, in beats (whole beats while playing)
  float sectStart  = 0;       // current section, in beats
  float sectEnd    = 0;
  int   tsNum      = 4;       // time signature
  int   tsDen      = 4;
  int   onBattery  = -1;      // AbleSet laptop on battery: 1 yes, 0 no, -1 unknown
} net, st, drawn;   // net: written by the receive task; st: snapshot for drawing

uint16_t drawnNextBg = 0xFFFF;
String drawnProgress = "";   // signature of what the progress strip shows

bool wifiUp = false;
bool absetUp = false;
bool mdnsUp = false;
bool drawnWifiUp = false, drawnAbsetUp = false;
bool forceRedraw = true;

volatile uint32_t lastPacketMs = 0;
uint32_t lastSubscribe  = 0;
uint32_t lastWifiCheck  = 0;
uint32_t lastWifiRetry  = 0;
uint32_t lastKeepalive  = 0;
uint32_t lastResync     = 0;

static const uint32_t ABLESET_TIMEOUT_MS = 10000;
static const uint32_t RESUBSCRIBE_MS     = 5000;
static const uint32_t KEEPALIVE_MS       = 2000;
static const uint32_t RESYNC_MS          = 3000;

// If we're on a network but AbleSet hasn't answered within this window, assume
// it lives on a different configured network and rotate to the next one.
static const uint32_t ABLESET_HUNT_MS    = 30000;
uint32_t wifiConnectedAt = 0;
int wifiRotate = 0;   // index of the next network to try (mod N_WIFI)

// Give-up logic: every network tried counts once (failed to connect, not in
// range, or connected but no AbleSet within ABLESET_HUNT_MS). After
// WIFI_ROUNDS passes through the list, stop and wait for the user.
static const int MAX_NETWORK_TRIES = WIFI_ROUNDS * N_WIFI;
int networkTries = 0;
bool anyWifiThisRun = false;   // did any network accept us since the last reset?
bool gaveUp = false;

// ============================================================================
// OSC TYPES
//
// These MUST stay above the first function definition in the file. The Arduino
// IDE auto-generates prototypes for every function and inserts them just before
// the first one — so any type used in a function signature has to be declared
// before that point, or the generated prototypes won't compile.
// ============================================================================

struct OscOut {
  uint8_t d[512];
  size_t n = 0;
  void reset() { n = 0; }
  void str(const char *s) {
    size_t l = strlen(s);
    if (n + l + 4 > sizeof(d)) return;
    memcpy(d + n, s, l); n += l; d[n++] = 0;
    while (n % 4) d[n++] = 0;
  }
  void i32(int32_t v) {
    if (n + 4 > sizeof(d)) return;
    d[n++] = (v >> 24) & 0xFF; d[n++] = (v >> 16) & 0xFF;
    d[n++] = (v >> 8) & 0xFF;  d[n++] = v & 0xFF;
  }
};

struct OscArg {
  char type = 0;
  const char *s = nullptr;
  int32_t i = 0;
  float f = 0;
};

// Where the playhead is inside the current section, for the progress bar.
struct SectProgress {
  bool  show;
  int   bars;        // section length in bars (rounded up)
  float beatsPerBar; // in quarter-note beats, e.g. 4 for 4/4, 3 for 6/8
  float len;         // section length in beats
  float lit;         // beats elapsed, counting the current one
};

// ============================================================================
// TEXT RENDERING (built-in 5x7 font, scaled)
// ============================================================================

static int wrapText(const String &txt, int charsPerLine, String *out, int maxLines) {
  int n = 0, i = 0, len = txt.length();
  while (i < len && n < maxLines) {
    while (i < len && txt[i] == ' ') i++;
    if (i >= len) break;
    if (len - i <= charsPerLine) { out[n++] = txt.substring(i); i = len; break; }
    int brk = -1;
    for (int j = i; j <= i + charsPerLine && j < len; j++) if (txt[j] == ' ') brk = j;
    if (brk < 0) { out[n++] = txt.substring(i, i + charsPerLine); i += charsPerLine; }
    else         { out[n++] = txt.substring(i, brk); i = brk + 1; }
  }
  return (i < len) ? -1 : n;
}

static void drawTextBlock(int x, int y, int w, int h, const String &txt,
                          uint16_t fg, uint16_t bg, int maxSize, int minSize) {
  gfx->fillRect(x, y, w, h, bg);
  if (txt.length() == 0) return;
  gfx->setTextWrap(false);
  gfx->setTextColor(fg);

  String lines[5];
  int nLines = 0, size = minSize;

  for (int s = maxSize; s >= minSize; s--) {
    int cpl = w / (6 * s);
    int maxLines = h / (9 * s);
    if (cpl < 1 || maxLines < 1) continue;
    if (maxLines > 5) maxLines = 5;
    int n = wrapText(txt, cpl, lines, maxLines);
    if (n > 0) { nLines = n; size = s; break; }
  }
  if (nLines == 0) {
    int cpl = w / (6 * minSize);
    lines[0] = txt.substring(0, max(1, cpl - 1)) + ">";
    nLines = 1; size = minSize;
  }

  gfx->setTextSize(size);
  int lineH  = 9 * size;
  int totalH = nLines * lineH - size;
  int ty = y + (h - totalH) / 2;
  for (int i = 0; i < nLines; i++) {
    int tw = lines[i].length() * 6 * size - size;
    gfx->setCursor(x + (w - tw) / 2, ty);
    gfx->print(lines[i]);
    ty += lineH;
  }
}

// Pick black or white text for a given background, whichever contrasts more.
// WCAG relative luminance; crossover at L = 0.179.
static float srgbLin(int v) {
  float c = v / 255.0f;
  return (c <= 0.03928f) ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}

static uint16_t textOn(uint16_t bg) {
  int r = ((bg >> 11) & 0x1F) * 255 / 31;
  int g = ((bg >> 5) & 0x3F) * 255 / 63;
  int b = (bg & 0x1F) * 255 / 31;
  float L = 0.2126f * srgbLin(r) + 0.7152f * srgbLin(g) + 0.0722f * srgbLin(b);
  return (L > 0.1791f) ? COL_BG : COL_TEXT;
}

// Song names can carry tags in braces, e.g. "Paper Moon {Drop}". They're
// for AbleSet, not the stage: strip them before display.
static String cleanName(const String &in) {
  String name;
  int i = 0, len = in.length();
  while (i < len) {
    if (in[i] == '{') {
      int j = in.indexOf('}', i + 1);
      i = (j < 0) ? len : j + 1;
    } else {
      name += in[i++];
    }
  }
  name.trim();
  while (name.indexOf("  ") >= 0) name.replace("  ", " ");
  return name;
}

// Decode one UTF-8 character starting at s[i]; advances i.
static uint32_t nextCodepoint(const String &s, unsigned &i) {
  uint8_t c = (uint8_t)s[i++];
  if (c < 0x80) return c;
  int extra = (c >= 0xF0) ? 3 : (c >= 0xE0) ? 2 : (c >= 0xC0) ? 1 : 0;
  uint32_t cp = c & (0x3F >> extra);
  while (extra-- > 0 && i < s.length()) cp = (cp << 6) | ((uint8_t)s[i++] & 0x3F);
  return cp;
}

// The built-in 5x7 font is ASCII only, so names are uppercased and folded to
// ASCII for display: é -> E, ß -> SS, œ -> OE, typographic quotes and dashes
// -> plain ones. Anything else (emoji, other scripts) is dropped.
static String displayText(const String &in) {
  // Latin-1 U+00C0..U+00FF, uppercased and stripped of accents.
  static const char FOLD[] =
    "AAAAAAACEEEEIIII" "DNOOOOOxOUUUUYTS"
    "AAAAAAACEEEEIIII" "DNOOOOO/OUUUUYTY";
  String s = cleanName(in);
  String out;
  unsigned i = 0;
  while (i < s.length()) {
    uint32_t cp = nextCodepoint(s, i);
    if (cp >= 'a' && cp <= 'z')                 out += (char)(cp - 32);
    else if (cp >= 0x20 && cp < 0x7F)           out += (char)cp;
    else if (cp == 0xC6 || cp == 0xE6)          out += "AE";
    else if (cp == 0xDF)                        out += "SS";
    else if (cp == 0x152 || cp == 0x153)        out += "OE";
    else if (cp >= 0xC0 && cp <= 0xFF)          out += FOLD[cp - 0xC0];
    else if (cp == 0x2013 || cp == 0x2014 || cp == 0x2212) out += '-';
    else if (cp >= 0x2018 && cp <= 0x201B)      out += '\'';
    else if (cp >= 0x201C && cp <= 0x201F)      out += '"';
    else if (cp == 0x2026)                      out += "...";
    else if (cp == 0xA0)                        out += ' ';
  }
  out.trim();
  return out;
}

static void drawStatusBar() {
  gfx->fillRect(0, Z_STATUS_Y, SCR_W, Z_STATUS_H, COL_BG);
  gfx->setTextWrap(false);
  gfx->setTextSize(1);

  uint16_t wifiCol  = wifiUp ? COL_OK : COL_ERR;
  uint16_t absetCol = absetUp ? (st.liveLinked ? COL_OK : COL_WARN) : COL_ERR;
  gfx->fillCircle(10, Z_STATUS_Y + 10, 4, wifiCol);
  gfx->fillCircle(26, Z_STATUS_Y + 10, 4, absetCol);

  // Power state of the computer running AbleSet.
  if (absetUp && st.onBattery == 1) {
    const char *msg = "ON BATTERY";
    int tw = strlen(msg) * 6 - 1;
    gfx->fillRoundRect(36, Z_STATUS_Y + 3, tw + 10, 14, 3, COL_ERR);
    gfx->setTextColor(COL_TEXT);
    gfx->setCursor(41, Z_STATUS_Y + 7);
    gfx->print(msg);
  } else if (absetUp && st.onBattery == 0) {
    gfx->setTextColor(COL_DIM);
    gfx->setCursor(38, Z_STATUS_Y + 7);
    gfx->print("AC POWER");
  }

  if (st.songCount > 0 && st.songIndex >= 0) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%d/%d", st.songIndex + 1, st.songCount);
    int tw = strlen(buf) * 6 - 1;
    gfx->setTextColor(COL_DIM);
    gfx->setCursor((SCR_W - tw) / 2, Z_STATUS_Y + 7);
    gfx->print(buf);
  }

  const char *tp = st.playing ? "PLAY" : "STOP";
  gfx->setTextColor(st.playing ? COL_OK : COL_DIM);
  gfx->setCursor(SCR_W - 4 - (int)strlen(tp) * 6, Z_STATUS_Y + 7);
  gfx->print(tp);
}

static void setLed(uint8_t r, uint8_t g, uint8_t b) {
  static uint8_t lr = 255, lg = 255, lb = 255;
  if (!USE_RGB_LED) return;
  if (r == lr && g == lg && b == lb) return;
  lr = r; lg = g; lb = b;
  rgbLedWrite(RGB_LED, r, g, b);
}

// Mix two RGB565 colours: t = 0 gives a, t = 1 gives b.
static uint16_t blend565(uint16_t a, uint16_t b, float t) {
  int ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
  int br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
  int r = ar + (int)((br - ar) * t + 0.5f);
  int gr = ag + (int)((bg - ag) * t + 0.5f);
  int bl = ab + (int)((bb - ab) * t + 0.5f);
  return (r << 11) | (gr << 5) | bl;
}

static SectProgress sectionProgress() {
  SectProgress p = { false, 0, 4, 0, 0 };
  p.len = st.sectEnd - st.sectStart;
  if (!SHOW_PROGRESS || !absetUp || st.beatPos < 0 || p.len < 0.5f) return p;

  p.beatsPerBar = st.tsNum * 4.0f / (st.tsDen > 0 ? st.tsDen : 4);
  if (p.beatsPerBar <= 0) p.beatsPerBar = 4;
  p.bars = max(1, (int)ceilf(p.len / p.beatsPerBar - 0.01f));

  float rel = st.beatPos - st.sectStart;
  if (rel < 0) rel = 0;
  if (PROGRESS_BY_BEAT) {
    // The beat we're on counts as lit while playing (like a metronome light).
    p.lit = floorf(rel + 0.001f) + (st.playing ? 1 : 0);
  } else {
    // The bar we're in counts as lit while playing; redraws once per bar.
    int barIdx = (int)floorf(rel / p.beatsPerBar + 0.001f);
    p.lit = (barIdx + (st.playing ? 1 : 0)) * p.beatsPerBar;
  }
  if (p.lit > p.len) p.lit = p.len;
  p.show = true;
  return p;
}

static String progressSig() {
  SectProgress p = sectionProgress();
  if (!p.show) return String("-");
  return String(p.bars) + "|" + String((int)(p.lit * 4)) + "|" + String((int)(p.len * 4)) +
         "|" + String(st.sectCol);
}

// Fill [x, x+w) of the strip: the first `fw` pixels lit, the rest dim. Each
// pixel is written once (no clear-then-draw), so beat updates don't flicker.
static void stripSpan(int x, int w, int fw, uint16_t on, uint16_t off) {
  if (fw > 0)     gfx->fillRect(x, PB_Y, fw, PB_H, on);
  if (fw < w)     gfx->fillRect(x + fw, PB_Y, w - fw, PB_H, off);
}

static void drawProgress() {
  if (!SHOW_PROGRESS) return;
  SectProgress p = sectionProgress();
  if (!p.show) {
    gfx->fillRect(0, PB_Y, SCR_W, PB_H, st.sectCol);
    return;
  }
  uint16_t on  = textOn(st.sectCol);
  uint16_t off = blend565(st.sectCol, on, 0.28f);

  if (p.bars > PB_MAX_SEGMENTS) {
    stripSpan(PB_X, PB_W, (int)(PB_W * (p.lit / p.len) + 0.5f), on, off);
    return;
  }

  // One segment per bar; it fills beat by beat, or all at once (PROGRESS_BY_BEAT).
  float segW = (PB_W - PB_GAP * (p.bars - 1)) / (float)p.bars;
  int prevEnd = PB_X;
  for (int i = 0; i < p.bars; i++) {
    int x0 = PB_X + (int)(i * (segW + PB_GAP) + 0.5f);
    int x1 = PB_X + (int)(i * (segW + PB_GAP) + segW + 0.5f);
    if (x0 > prevEnd) gfx->fillRect(prevEnd, PB_Y, x0 - prevEnd, PB_H, st.sectCol);  // gap
    float barStart = i * p.beatsPerBar;
    float barLen = min(p.beatsPerBar, p.len - barStart);
    float f = (p.lit - barStart) / barLen;
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    stripSpan(x0, x1 - x0, (int)((x1 - x0) * f + 0.5f), on, off);
    prevEnd = x1;
  }
}

static void render() {
  if (forceRedraw || wifiUp != drawnWifiUp || absetUp != drawnAbsetUp ||
      st.playing != drawn.playing || st.songIndex != drawn.songIndex ||
      st.songCount != drawn.songCount || st.liveLinked != drawn.liveLinked ||
      st.onBattery != drawn.onBattery) {
    drawStatusBar();
    drawnWifiUp = wifiUp; drawnAbsetUp = absetUp;
    drawn.playing = st.playing; drawn.songIndex = st.songIndex;
    drawn.songCount = st.songCount; drawn.liveLinked = st.liveLinked;
    drawn.onBattery = st.onBattery;
  }

  if (forceRedraw || st.song != drawn.song) {
    String s = displayText(st.song);
    uint16_t c = COL_TEXT;
    if (!s.length()) {
      s = !wifiUp ? "NO WIFI" : (absetUp ? "--" : "NO ABLESET");
      c = COL_ERR;
    }
    drawTextBlock(4, Z_SONG_Y, SCR_W - 8, Z_SONG_H, s, c, COL_BG, 3, 2);
    drawn.song = st.song;
  }

  bool sectChanged = forceRedraw || st.section != drawn.section || st.sectCol != drawn.sectCol;
  if (sectChanged) {
    String s = displayText(st.section);
    if (!s.length()) s = "--";
    drawTextBlock(0, Z_SECT_Y, SCR_W, SECT_TEXT_H, s, textOn(st.sectCol), st.sectCol, 4, 2);
    // Band below the name box, around the strip.
    gfx->fillRect(0, Z_SECT_Y + SECT_TEXT_H, SCR_W, Z_SECT_H - SECT_TEXT_H, st.sectCol);
    drawn.section = st.section;
    drawn.sectCol = st.sectCol;
  }

  String ps = progressSig();
  if (sectChanged || ps != drawnProgress) {
    drawProgress();
    drawnProgress = ps;
  }

  // Bottom row, in priority order: loop > next section > next song.
  // Text colour is derived from the background here too (textOn).
  if (SHOW_NEXT) {
    uint16_t bg = COL_BG;
    if (st.inLoop) {
      char buf[24];
      int n = st.loopCount + LOOP_COUNTER_OFFSET;
      if (st.loopTarget > 0) snprintf(buf, sizeof(buf), "LOOP %d/%d", n, st.loopTarget);
      else                   snprintf(buf, sizeof(buf), "LOOP %d", n);
      st.nextLine = buf;
      bg = COL_LOOP;
    } else if (displayText(st.nextSect).length()) {
      st.nextLine = "NEXT: " + displayText(st.nextSect);
      bg = st.nextSectCol;
    } else if (displayText(st.nextSong).length()) {
      st.nextLine = "NEXT: " + displayText(st.nextSong);
    } else {
      st.nextLine = "";
    }

    if (forceRedraw || st.nextLine != drawn.nextLine || bg != drawnNextBg) {
      drawTextBlock(0, Z_NEXT_Y, SCR_W, Z_NEXT_H, st.nextLine, textOn(bg), bg, 2, 1);
      drawn.nextLine = st.nextLine;
      drawnNextBg = bg;
    }
  }

  forceRedraw = false;

  if (!wifiUp)            setLed(20, 0, 0);
  else if (!absetUp)      setLed(20, 8, 0);
  else if (st.playing)    setLed(0, 12, 0);
  else                    setLed(0, 0, 10);
}

// ============================================================================
// OSC — minimal encoder / decoder (no external library)
// ============================================================================

// Subnet broadcast, e.g. 192.168.1.255 — used before we know AbleSet's address.
static IPAddress broadcastIp() {
  IPAddress ip = WiFi.localIP(), mask = WiFi.subnetMask();
  IPAddress b;
  for (int i = 0; i < 4; i++) b[i] = ip[i] | (~mask[i] & 0xFF);
  return b;
}

// AbleSet fills in the sender's address when /subscribe omits the IP field, so
// a broadcast subscribe works even when we have no idea where AbleSet is. We
// learn its address from the first packet it sends back.
static void oscSend(OscOut &m) {
  if (!wifiUp || oscSock < 0) return;
  IPAddress dst = broadcastIp();
  if (haveAblesetIp && (uint32_t)abletonIp != 0) dst = abletonIp;
  struct sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(ABLESET_PORT);
  to.sin_addr.s_addr = (uint32_t)dst;
  sendto(oscSock, m.d, m.n, 0, (struct sockaddr *)&to, sizeof(to));
}

static const char *FILTERS[] = {
  "/setlist/activeSongName",
  "/setlist/activeSongIndex",
  "/setlist/activeSectionName",
  "/setlist/activeSectionColor",
  "/setlist/nextSectionName",
  "/setlist/nextSectionColor",
  "/setlist/nextSongName",
  "/setlist/isInActiveSectionLoop",
  "/setlist/loopCounter",
  "/setlist/activeSectionLoopCount",
  "/setlist/songCount",
  "/setlist/activeSectionStart",
  "/setlist/activeSectionEnd",
  "/global/beatsPosition",
  "/global/timeSignature",
  "/computer/onBattery",
  "/global/isPlaying",
  "/global/isConnected",
  "/heartbeat",
};
static const int N_FILTERS = sizeof(FILTERS) / sizeof(FILTERS[0]);

static const bool USE_FILTERS = true;

// DNS hostnames can't contain spaces or uppercase reliably.
static String hostnameFromDeviceName() {
  String h;
  for (size_t i = 0; DEVICE_NAME[i]; i++) {
    char c = DEVICE_NAME[i];
    if (c == ' ' || c == '_' || c == '.') c = '-';
    else if (c >= 'A' && c <= 'Z') c += 32;
    else if (!isalnum((int)c) && c != '-') continue;
    h += c;
  }
  if (h.length() == 0) h = "ableset-display";
  return h;
}

// Full state. Sent right after subscribing.
static void sendGetValues() {
  OscOut g;
  g.str("/getValues");
  g.str(",i");
  g.i32(LOCAL_PORT);
  oscSend(g);
}

// Periodic self-heal: only the small values. Skips the playhead, which
// AbleSet pushes continuously anyway.
static const char *RESYNC_VALUES[] = {
  "/setlist/activeSongName",
  "/setlist/activeSongIndex",
  "/setlist/activeSectionName",
  "/setlist/activeSectionColor",
  "/setlist/nextSectionName",
  "/setlist/nextSectionColor",
  "/setlist/nextSongName",
  "/setlist/isInActiveSectionLoop",
  "/setlist/loopCounter",
  "/setlist/activeSectionLoopCount",
  "/setlist/activeSectionStart",
  "/setlist/activeSectionEnd",
  "/global/timeSignature",
  "/global/isPlaying",
  "/global/isConnected",
  "/computer/onBattery",
};
static const int N_RESYNC = sizeof(RESYNC_VALUES) / sizeof(RESYNC_VALUES[0]);

static void sendResync() {
  OscOut g;
  g.str("/getValues");
  String tt = ",";
  for (int i = 0; i < N_RESYNC; i++) tt += "s";
  g.str(tt.c_str());
  for (int i = 0; i < N_RESYNC; i++) g.str(RESYNC_VALUES[i]);
  oscSend(g);
}

static void sendSubscribe() {
  OscOut m;
  m.str("/subscribe");
  // typetags: int (port), string (name), False (no fine updates), then filters
  String tt = ",isF";
  if (USE_FILTERS) for (int i = 0; i < N_FILTERS; i++) tt += "s";
  m.str(tt.c_str());
  m.i32(LOCAL_PORT);
  m.str(DEVICE_NAME);
  if (USE_FILTERS) for (int i = 0; i < N_FILTERS; i++) m.str(FILTERS[i]);
  oscSend(m);

  sendGetValues();
}

// --- decoding ---------------------------------------------------------------

static float argNum(const OscArg &a) {
  if (a.type == 'i') return (float)a.i;
  if (a.type == 'f') return a.f;
  if (a.type == 'T') return 1;
  if (a.type == 'F') return 0;
  if (a.type == 's' && a.s) return atof(a.s);
  return 0;
}

// --- section band colours ---------------------------------------------------
//
// Colours are matched BY NAME against the palette below — hand-picked, fully
// saturated RGB565 values. Nothing is derived from AbleSet's hex strings for
// known names, so there are no assumptions about what its palette "should"
// compute to. Unknown names fall back to HSV conversion of the hex argument.

struct NamedColor { const char *name; uint16_t rgb565; };

static const NamedColor PALETTE[] = {
  { "red",    0xF800 },  // 255,   0,   0
  { "orange", 0xFC00 },  // 255, 128,   0
  { "yellow", 0xFFE0 },  // 255, 255,   0
  { "green",  0x07E0 },  //   0, 255,   0
  { "cyan",   0x07FF },  //   0, 255, 255
  { "blue",   0x001F },  //   0,   0, 255
  { "purple", 0xA81F },  // 168,   0, 255
  { "pink",   0xF81F },  // 255,   0, 255
  { "white",  0xFFFF },  // 255, 255, 255
  { "black",  0x94B2 },  // 150, 150, 150 — a band can't be black and visible
  { "gray",   0x94B2 },
  { "grey",   0x94B2 },
};
static const int N_PALETTE = sizeof(PALETTE) / sizeof(PALETTE[0]);

static bool nameEq(const char *a, const char *b) {
  while (*a && *b) {
    char ca = (*a >= 'A' && *a <= 'Z') ? *a + 32 : *a;
    char cb = (*b >= 'A' && *b <= 'Z') ? *b + 32 : *b;
    if (ca != cb) return false;
    a++; b++;
  }
  return *a == *b;
}

// HSV fallback for colour names not in the palette: hue preserved, saturation
// multiplied, brightness fixed. Near-neutral inputs stay neutral.
static uint16_t hexToRgb565(const char *hex) {
  if (!hex) return 0x2104;
  if (*hex == '#') hex++;
  if (strlen(hex) < 6) return 0x2104;
  auto hx = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
  };
  int r = hx(hex[0]) * 16 + hx(hex[1]);
  int g = hx(hex[2]) * 16 + hx(hex[3]);
  int b = hx(hex[4]) * 16 + hx(hex[5]);

  int mx = max(r, max(g, b)), mn = min(r, min(g, b));
  int d = mx - mn;

  if (mx < 40) return 0x94B2;
  float s = (float)d / mx;
  if (s < 0.15f) return 0xFFFF;

  float h;
  if (mx == r)      h = fmodf((float)(g - b) / d + 6.0f, 6.0f);
  else if (mx == g) h = (float)(b - r) / d + 2.0f;
  else              h = (float)(r - g) / d + 4.0f;

  float sOut = s * BAND_SATURATION;
  if (sOut > 1) sOut = 1;
  float v = (float)BAND_BRIGHTNESS;

  float c = v * sOut;
  float x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));
  float m = v - c;
  float rf, gf, bf;
  switch ((int)h) {
    case 0:  rf = c; gf = x; bf = 0; break;
    case 1:  rf = x; gf = c; bf = 0; break;
    case 2:  rf = 0; gf = c; bf = x; break;
    case 3:  rf = 0; gf = x; bf = c; break;
    case 4:  rf = x; gf = 0; bf = c; break;
    default: rf = c; gf = 0; bf = x; break;
  }
  int ri = (int)(rf + m + 0.5f), gi = (int)(gf + m + 0.5f), bi = (int)(bf + m + 0.5f);
  if (ri > 255) ri = 255;
  if (gi > 255) gi = 255;
  if (bi > 255) bi = 255;

  return ((ri & 0xF8) << 8) | ((gi & 0xFC) << 3) | (bi >> 3);
}

// Resolve a colour message's arguments to a band colour. The raw input is
// logged only when the resulting colour changes, so the periodic resync
// doesn't flood the serial port.
static uint16_t colorFromArgs(const char *addr, OscArg *args, int nArgs, uint16_t prev) {
  const char *name = nullptr, *hex = nullptr;
  for (int i = 0; i < nArgs; i++) {
    if (args[i].type == 's' && args[i].s) {
      if (args[i].s[0] == '#') { if (!hex) hex = args[i].s; }
      else                     { if (!name) name = args[i].s; }
    }
  }

  uint16_t c = 0;
  const char *how = "hsv fallback";
  bool found = false;
  if (name) {
    for (int i = 0; i < N_PALETTE && !found; i++) {
      if (nameEq(name, PALETTE[i].name)) { c = PALETTE[i].rgb565; how = PALETTE[i].name; found = true; }
    }
  }
  if (!found) c = hexToRgb565(hex);

  if (c != prev) {
    Serial.printf("[color] %s: \"%s\" \"%s\" -> %s = %04X\n", addr,
                  name ? name : "", hex ? hex : "", how, c);
  }
  return c;
}

static void handleMessage(const char *addr, OscArg *args, int nArgs) {
  if (!strcmp(addr, "/heartbeat")) return;

  if (!strcmp(addr, "/setlist/activeSongName")) {
    net.song = (nArgs > 0 && args[0].type == 's') ? String(args[0].s) : String("");
  } else if (!strcmp(addr, "/setlist/activeSectionName")) {
    net.section = (nArgs > 0 && args[0].type == 's') ? String(args[0].s) : String("");
  } else if (!strcmp(addr, "/setlist/nextSectionName")) {
    net.nextSect = (nArgs > 0 && args[0].type == 's') ? String(args[0].s) : String("");
  } else if (!strcmp(addr, "/setlist/nextSongName")) {
    net.nextSong = (nArgs > 0 && args[0].type == 's') ? String(args[0].s) : String("");
  } else if (!strcmp(addr, "/setlist/isInActiveSectionLoop")) {
    net.inLoop = nArgs > 0 && argNum(args[0]) > 0.5f;
  } else if (!strcmp(addr, "/setlist/loopCounter")) {
    net.loopCount = nArgs > 0 ? (int)argNum(args[0]) : 0;
  } else if (!strcmp(addr, "/setlist/activeSectionLoopCount")) {
    net.loopTarget = nArgs > 0 ? (int)argNum(args[0]) : 0;
  } else if (!strcmp(addr, "/setlist/activeSectionColor")) {
    net.sectCol = colorFromArgs(addr, args, nArgs, net.sectCol);
  } else if (!strcmp(addr, "/setlist/nextSectionColor")) {
    net.nextSectCol = colorFromArgs(addr, args, nArgs, net.nextSectCol);
  } else if (!strcmp(addr, "/setlist/activeSongIndex")) {
    net.songIndex = nArgs > 0 ? (int)argNum(args[0]) : -1;
  } else if (!strcmp(addr, "/computer/onBattery")) {
    int v = nArgs > 0 ? (argNum(args[0]) > 0.5f ? 1 : 0) : -1;
    if (v != net.onBattery) Serial.printf("AbleSet computer %s\n", v == 1 ? "ON BATTERY" : "on AC power");
    net.onBattery = v;
  } else if (!strcmp(addr, "/global/beatsPosition")) {
    static bool loggedPos = false;
    net.beatPos = nArgs > 0 ? argNum(args[0]) : -1;
    if (!loggedPos) { loggedPos = true; Serial.printf("[progress] first playhead position %.1f beats\n", net.beatPos); }
  } else if (!strcmp(addr, "/setlist/activeSectionStart")) {
    float v = nArgs > 0 ? argNum(args[0]) : 0;
    if (v != net.sectStart) Serial.printf("[progress] section start %.1f beats\n", v);
    net.sectStart = v;
  } else if (!strcmp(addr, "/setlist/activeSectionEnd")) {
    float v = nArgs > 0 ? argNum(args[0]) : 0;
    if (v != net.sectEnd) Serial.printf("[progress] section end %.1f beats\n", v);
    net.sectEnd = v;
  } else if (!strcmp(addr, "/global/timeSignature")) {
    if (nArgs >= 2) {
      int n = (int)argNum(args[0]), d = (int)argNum(args[1]);
      if (n > 0 && d > 0 && (n != net.tsNum || d != net.tsDen))
        Serial.printf("[progress] time signature %d/%d\n", n, d);
      if (n > 0 && d > 0) { net.tsNum = n; net.tsDen = d; }
    }
  } else if (!strcmp(addr, "/setlist/songCount")) {
    net.songCount = nArgs > 0 ? (int)argNum(args[0]) : 0;
  } else if (!strcmp(addr, "/global/isPlaying")) {
    net.playing = nArgs > 0 && argNum(args[0]) > 0.5f;
  } else if (!strcmp(addr, "/global/isConnected")) {
    net.liveLinked = nArgs > 0 && argNum(args[0]) > 0.5f;
  }
}

static void parsePacket(const uint8_t *p, size_t len);

static void parseElement(const uint8_t *p, size_t len) {
  if (len < 4) return;

  const char *addr = (const char *)p;
  size_t alen = strnlen(addr, len);
  if (alen >= len) return;
  size_t off = (alen + 4) & ~3u;
  if (off >= len) { handleMessage(addr, (OscArg *)nullptr, 0); return; }

  const char *tt = (const char *)(p + off);
  if (tt[0] != ',') return;
  size_t tlen = strnlen(tt, len - off);
  if (off + tlen >= len) return;
  off += (tlen + 4) & ~3u;

  OscArg args[8];
  int nArgs = 0;
  for (size_t i = 1; i < tlen && nArgs < 8; i++) {
    char t = tt[i];
    OscArg a; a.type = t;
    if (t == 's' || t == 'S') {
      if (off >= len) break;
      a.s = (const char *)(p + off);
      size_t sl = strnlen(a.s, len - off);
      off += (sl + 4) & ~3u;
    } else if (t == 'i' || t == 'f' || t == 'c' || t == 'r' || t == 'm') {
      if (off + 4 > len) break;
      uint32_t v = ((uint32_t)p[off] << 24) | ((uint32_t)p[off + 1] << 16) |
                   ((uint32_t)p[off + 2] << 8) | (uint32_t)p[off + 3];
      if (t == 'f') memcpy(&a.f, &v, 4); else a.i = (int32_t)v;
      off += 4;
    } else if (t == 'h' || t == 'd' || t == 't') {
      if (off + 8 > len) break;
      off += 8;
    } else if (t == 'b') {
      if (off + 4 > len) break;
      uint32_t bl = ((uint32_t)p[off] << 24) | ((uint32_t)p[off + 1] << 16) |
                    ((uint32_t)p[off + 2] << 8) | (uint32_t)p[off + 3];
      off += 4 + ((bl + 3) & ~3u);
    }
    args[nArgs++] = a;
  }
  handleMessage(addr, args, nArgs);
}

static void parsePacket(const uint8_t *p, size_t len) {
  if (len >= 8 && !memcmp(p, "#bundle", 7)) {
    size_t off = 16;
    while (off + 4 <= len) {
      uint32_t sz = ((uint32_t)p[off] << 24) | ((uint32_t)p[off + 1] << 16) |
                    ((uint32_t)p[off + 2] << 8) | (uint32_t)p[off + 3];
      off += 4;
      if (sz == 0 || off + sz > len) break;
      parsePacket(p + off, sz);
      off += sz;
    }
  } else {
    parseElement(p, len);
  }
}

// ============================================================================
// NETWORK
// ============================================================================

// ============================================================================
// RETRY SCREEN
// ============================================================================

// True once per press of the BOOT button.
static bool wasPressed = true;
static bool retryPressed() {
  bool down = digitalRead(RETRY_BTN_PIN) == LOW;
  bool fired = down && !wasPressed;
  wasPressed = down;
  return fired;
}

static void drawRetryScreen() {
  gfx->fillScreen(COL_BG);

  String title = anyWifiThisRun ? "ABLESET NOT FOUND" : "NO WIFI";
  String names;
  for (int i = 0; i < N_WIFI; i++) {
    if (i) names += (i == N_WIFI - 1) ? " & " : ", ";
    names += WIFI_NETWORKS[i].ssid;
  }
  String detail = String("Tried ") + names + (WIFI_ROUNDS == 2 ? " twice" : "");
  detail.toUpperCase();

  drawTextBlock(4, 8, SCR_W - 8, 36, title, COL_ERR, COL_BG, 3, 2);
  drawTextBlock(4, 46, SCR_W - 8, 30, displayText(detail), COL_DIM, COL_BG, 2, 1);

  const int bw = 170, bh = 50, bx = (SCR_W - bw) / 2, by = 88;
  gfx->fillRoundRect(bx, by, bw, bh, 10, COL_TEXT);
  drawTextBlock(bx + 8, by + 6, bw - 16, bh - 12, "RETRY", COL_BG, COL_TEXT, 4, 3);

  drawTextBlock(4, 148, SCR_W - 8, 18, "PRESS BOOT TO TRY AGAIN", COL_DIM, COL_BG, 1, 1);
}

static void giveUp() {
  Serial.printf("Gave up after %d network tries — waiting for Retry\n", networkTries);
  gaveUp = true;
  wifiUp = false;
  haveAblesetIp = false;
  connectedSsid = "";
  if (mdnsUp) { MDNS.end(); mdnsUp = false; }
  WiFi.disconnect(true);       // radio off until the user asks again
  wasPressed = true;           // ignore a button that's already held
  setLed(20, 0, 0);
  drawRetryScreen();
}

static void splash(const String &line1, const String &line2, uint16_t col) {
  gfx->fillScreen(COL_BG);
  drawTextBlock(4, 40, SCR_W - 8, 50, displayText(line1), col, COL_BG, 3, 1);
  drawTextBlock(4, 95, SCR_W - 8, 40, displayText(line2), COL_DIM, COL_BG, 2, 1);
}

// How long since AbleSet last sent anything. Reads the timestamp first, then
// the clock, so a packet landing in between can't make the age wrap around.
static uint32_t packetAgeMs() {
  uint32_t last = lastPacketMs;
  return millis() - last;
}

// The receive task also updates the AbleSet address, so writes go through the lock.
static void setAblesetIp(const IPAddress &ip) {
  xSemaphoreTake(stateLock, portMAX_DELAY);
  abletonIp = ip;
  haveAblesetIp = true;
  xSemaphoreGive(stateLock);
}

// mDNS is only a hint — the broadcast subscribe is what actually finds
// AbleSet. Kept because a direct hit saves a few seconds on boot.
static bool discoverAbleset() {
  if (strlen(ABLESET_HOST) > 0) {
    IPAddress fixed;
    if (fixed.fromString(ABLESET_HOST)) {
      setAblesetIp(fixed);
      Serial.printf("Using configured AbleSet host %s\n", ABLESET_HOST);
      return true;
    }
    IPAddress resolved;
    if (WiFi.hostByName(ABLESET_HOST, resolved)) {
      setAblesetIp(resolved);
      Serial.printf("Resolved %s to %s\n", ABLESET_HOST, resolved.toString().c_str());
      return true;
    }
    Serial.printf("Could not resolve ABLESET_HOST \"%s\"\n", ABLESET_HOST);
    return false;
  }

  int n = MDNS.queryService("osc", "udp");
  Serial.printf("mDNS _osc._udp: %d result(s)\n", n);
  for (int i = 0; i < n; i++) {
    IPAddress a = MDNS.address(i);
    String hn = MDNS.hostname(i);
    Serial.printf("  %s %s:%u\n", hn.c_str(), a.toString().c_str(), MDNS.port(i));
    if (MDNS.port(i) != ABLESET_PORT) continue;

    // The SRV record often arrives without a resolved A record (0.0.0.0).
    if ((uint32_t)a == 0 && hn.length() > 0) {
      Serial.printf("  no address in SRV, resolving \"%s.local\"...\n", hn.c_str());
      IPAddress r = MDNS.queryHost(hn.c_str(), 3000);
      Serial.printf("  -> %s\n", r.toString().c_str());
      a = r;
    }

    if ((uint32_t)a != 0) {
      setAblesetIp(a);
      Serial.printf("Found AbleSet at %s\n", a.toString().c_str());
      return true;
    }
    Serial.println("  no usable IPv4 address — falling back to broadcast");
  }
  return false;
}

// --- WiFi diagnostics --------------------------------------------------------

volatile uint8_t lastDisconnectReason = 0;

static const char *wifiStatusName(int s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    case WL_STOPPED:         return "STOPPED";
    default:                 return "UNKNOWN";
  }
}

static const char *disconnectReasonName(uint8_t r) {
  switch (r) {
    case 1:   return "UNSPECIFIED";
    case 2:   return "AUTH_EXPIRE";
    case 4:   return "ASSOC_EXPIRE";
    case 8:   return "ASSOC_LEAVE";
    case 15:  return "4WAY_HANDSHAKE_TIMEOUT (wrong password?)";
    case 23:  return "802_1X_AUTH_FAILED";
    case 200: return "BEACON_TIMEOUT";
    case 201: return "NO_AP_FOUND (out of range, or wrong band)";
    case 202: return "AUTH_FAIL (wrong password?)";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    case 205: return "CONNECTION_FAIL";
    case 206: return "AP_TSF_RESET";
    case 207: return "ROAMING";
    default:  return "see esp_wifi_types.h";
  }
}

static const char *encryptionName(wifi_auth_mode_t m) {
  switch (m) {
    case WIFI_AUTH_OPEN:            return "open";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-ENT";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "?";
  }
}

static void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("  [event] associated with AP");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("  [event] got IP %s\n", WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("  [event] lost IP");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
      uint8_t r = info.wifi_sta_disconnected.reason;
      lastDisconnectReason = r;
      Serial.printf("  [event] disconnected, reason %u: %s\n", r, disconnectReasonName(r));
      break;
    }
    default:
      break;
  }
}

// Try one configured network. Returns true once we have an IP.
static bool tryNetwork(int idx) {
  const WifiNetwork &net = WIFI_NETWORKS[idx];
  Serial.printf("Trying \"%s\" (password %d chars)...\n", net.ssid, (int)strlen(net.pass));
  splash("CONNECTING", net.ssid, COL_WARN);

  lastDisconnectReason = 0;
  WiFi.begin(net.ssid, net.pass);
  uint32_t t0 = millis();
  int lastStatus = -1;
  while (millis() - t0 < WIFI_TRY_MS) {
    int s = WiFi.status();
    if (s != lastStatus) {
      Serial.printf("  status: %s\n", wifiStatusName(s));
      lastStatus = s;
    }
    if (s == WL_CONNECTED) {
      connectedSsid = net.ssid;
      Serial.printf("Connected to \"%s\"\n", net.ssid);
      Serial.printf("  IP      %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("  gateway %s\n", WiFi.gatewayIP().toString().c_str());
      Serial.printf("  channel %d (%s), RSSI %d dBm\n",
                    WiFi.channel(), WiFi.channel() > 14 ? "5 GHz" : "2.4 GHz", WiFi.RSSI());
      return true;
    }
    delay(100);
  }
  Serial.printf("  ...\"%s\" failed after %lu ms, last status %s",
                net.ssid, (unsigned long)WIFI_TRY_MS, wifiStatusName(WiFi.status()));
  if (lastDisconnectReason)
    Serial.printf(", reason %u: %s", lastDisconnectReason,
                  disconnectReasonName(lastDisconnectReason));
  Serial.println();
  WiFi.disconnect(false, true);
  delay(100);
  return false;
}

// Scan and mark which configured networks are in range. If none are (hidden
// SSIDs, or a scan glitch), every network is treated as worth trying.
static void scanVisible(bool *visible) {
  splash("SCANNING", "for known networks", COL_WARN);
  int found = WiFi.scanNetworks();
  Serial.printf("\n--- scan: %d networks ---\n", found);
  for (int j = 0; j < found; j++) {
    Serial.printf("  %2d. %-24s ch %-3d %4d dBm  %s\n",
                  j + 1, WiFi.SSID(j).c_str(), WiFi.channel(j),
                  WiFi.RSSI(j), encryptionName(WiFi.encryptionType(j)));
  }
  int nVisible = 0;
  for (int i = 0; i < N_WIFI; i++) {
    int hit = -1;
    for (int j = 0; j < found; j++) {
      if (WiFi.SSID(j) == WIFI_NETWORKS[i].ssid) { hit = j; break; }
    }
    visible[i] = hit >= 0;
    if (hit >= 0) {
      nVisible++;
      Serial.printf("configured \"%s\": FOUND, ch %d (%s), %d dBm\n",
                    WIFI_NETWORKS[i].ssid, WiFi.channel(hit),
                    WiFi.channel(hit) > 14 ? "5 GHz" : "2.4 GHz", WiFi.RSSI(hit));
    } else {
      Serial.printf("configured \"%s\": not in scan\n", WIFI_NETWORKS[i].ssid);
    }
  }
  WiFi.scanDelete();
  if (nVisible == 0) {
    Serial.println("No configured network visible; trying all anyway");
    for (int i = 0; i < N_WIFI; i++) visible[i] = true;
  }
}

// Try the configured networks one at a time, starting where the last attempt
// left off, until one connects or the try budget runs out (then: Retry
// screen). Each network tried — reachable or not — uses one try; a fresh scan
// is done at the start of every pass through the list.
static bool connectWifi() {
  connectedSsid = "";
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  String host = hostnameFromDeviceName();
  WiFi.setHostname(host.c_str());
  Serial.printf("MAC %s, hostname %s\n", WiFi.macAddress().c_str(), host.c_str());

  bool visible[N_WIFI];
  bool scanned = false;

  while (networkTries < MAX_NETWORK_TRIES) {
    if (!scanned || wifiRotate % N_WIFI == 0) {
      scanVisible(visible);
      scanned = true;
    }
    int idx = wifiRotate % N_WIFI;
    wifiRotate++;
    networkTries++;
    Serial.printf("Network try %d/%d: \"%s\"\n", networkTries, MAX_NETWORK_TRIES,
                  WIFI_NETWORKS[idx].ssid);
    if (!visible[idx]) continue;

    if (tryNetwork(idx)) {
      anyWifiThisRun = true;
      wifiUp = true;
      wifiConnectedAt = millis();
      if (mdnsUp) { MDNS.end(); mdnsUp = false; }
      MDNS.begin(host.c_str());
      mdnsUp = true;
      Serial.printf("Hostname: %s.local\n", host.c_str());
      startNetworking();
      splash("LOOKING FOR", "ABLESET", COL_WARN);
      forceRedraw = true;
      return true;
    }
  }

  wifiUp = false;
  giveUp();
  return false;
}

// Called when the user presses BOOT on the Retry screen: start a fresh budget from the first network.
static void retryNow() {
  Serial.println("Retry requested");
  gaveUp = false;
  networkTries = 0;
  anyWifiThisRun = false;
  wifiRotate = 0;
  if (connectWifi()) {
    discoverAbleset();
    sendSubscribe();
    lastSubscribe = millis();
  }
  lastWifiRetry = millis();
  lastWifiCheck = millis();
  if (!gaveUp) {
    gfx->fillScreen(COL_BG);
    forceRedraw = true;
  }
}

// ============================================================================
// NETWORK RECEIVE TASK
// ============================================================================

static void oscRxTask(void *) {
  static uint8_t buf[4096];   // the full /getValues reply is one big bundle
  for (;;) {
    struct sockaddr_in from;
    socklen_t fl = sizeof(from);
    int len = recvfrom(oscSock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fl);
    if (len <= 0) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    IPAddress src((uint32_t)from.sin_addr.s_addr);

    xSemaphoreTake(stateLock, portMAX_DELAY);
    if (!haveAblesetIp || src != abletonIp) {
      abletonIp = src;
      haveAblesetIp = true;
      Serial.printf("AbleSet is at %s\n", src.toString().c_str());
    }
    parsePacket(buf, len);
    lastPacketMs = millis();
    xSemaphoreGive(stateLock);
  }
}

// Open the OSC socket (bound to LOCAL_PORT) and start the receive task.
// Done once; the socket survives WiFi reconnects.
static void startNetworking() {
  if (oscSock >= 0) return;
  oscSock = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(LOCAL_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (oscSock < 0 || bind(oscSock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    Serial.println("ERROR: could not open the OSC socket");
    if (oscSock >= 0) { close(oscSock); oscSock = -1; }
    return;
  }
  int yes = 1;
  setsockopt(oscSock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
  // Priority above loop() so packets are handled even mid-redraw.
  xTaskCreate(oscRxTask, "osc_rx", 8192, nullptr, 5, nullptr);
  Serial.println("OSC receive task started");
}

// Copy what the receive task has gathered into the drawing snapshot. Kept
// short so the receive task is never held up.
static void takeSnapshot() {
  xSemaphoreTake(stateLock, portMAX_DELAY);
  State s = net;
  xSemaphoreGive(stateLock);
  st = s;
}

// ============================================================================

void setup() {
  Serial.begin(115200);
  // Never wait on the USB serial port. If the board is powered from a computer
  // that isn't reading the log, a full buffer would otherwise stall every log
  // line for up to ~2 s and freeze the display.
  Serial.setTxTimeoutMs(0);
  delay(300);
  Serial.println("\n\n=== AbleSet display (1.47\") booting ===");
  stateLock = xSemaphoreCreateMutex();
  WiFi.onEvent(onWifiEvent);
  pinMode(RETRY_BTN_PIN, INPUT_PULLUP);

  pinMode(LCD_BL, OUTPUT);
  ledcAttach(LCD_BL, 5000, 8);
  ledcWrite(LCD_BL, map(BACKLIGHT_PCT, 0, 100, 0, 255));

  gfx->begin();
  gfx->fillScreen(COL_BG);
  splash("ABLESET", "starting up", COL_TEXT);
  setLed(0, 0, 10);

  if (connectWifi()) {
    discoverAbleset();
    sendSubscribe();
    lastSubscribe = millis();
  }
  lastWifiRetry = millis();

  if (!gaveUp) {
    gfx->fillScreen(COL_BG);
    forceRedraw = true;
  }
}

void loop() {
  // Gave up: everything network-related is paused until Retry.
  if (gaveUp) {
    if (retryPressed()) retryNow();
    delay(20);
    return;
  }

  uint32_t now = millis();

  // Incoming OSC is handled by the receive task (oscRxTask).

  // --- WiFi watchdog ---
  if (now - lastWifiCheck > 2000) {
    lastWifiCheck = now;
    bool up = (WiFi.status() == WL_CONNECTED);
    if (up != wifiUp) {
      wifiUp = up;
      forceRedraw = true;
      if (!up) {
        haveAblesetIp = false;
        connectedSsid = "";
        // Reconnect to the network we just lost first, not the next one.
        wifiRotate = (wifiRotate + N_WIFI - 1) % N_WIFI;
        Serial.println("WiFi lost");
      }
    }
    if (!wifiUp && now - lastWifiRetry > WIFI_RETRY_MS) {
      connectWifi();
      lastWifiRetry = millis();
      if (gaveUp) return;
      gfx->fillScreen(COL_BG);
      forceRedraw = true;
      if (wifiUp) {
        discoverAbleset();
        sendSubscribe();
        lastSubscribe = millis();
      }
      now = millis();
      lastWifiCheck = now;
    }
  }

  // --- AbleSet watchdog ---
  bool up = wifiUp && haveAblesetIp && (packetAgeMs() < ABLESET_TIMEOUT_MS);
  if (up != absetUp) {
    absetUp = up;
    forceRedraw = true;
    if (up) {
      networkTries = 0;          // found it: a later dropout gets a full budget again
      anyWifiThisRun = true;
    }
    if (!up) {
      xSemaphoreTake(stateLock, portMAX_DELAY);
      net.song = ""; net.section = ""; net.nextSect = ""; net.nextSong = "";
      net.inLoop = false; net.loopCount = 0; net.loopTarget = 0;
      net.nextSectCol = 0x2104;
      net.beatPos = -1; net.sectStart = 0; net.sectEnd = 0;
      net.onBattery = -1;
      xSemaphoreGive(stateLock);
      drawnNextBg = 0xFFFF; drawnProgress = "";
    }
  }

  // Ping AbleSet so we notice a dead link even when nothing is changing.
  if (haveAblesetIp && now - lastKeepalive > KEEPALIVE_MS) {
    lastKeepalive = now;
    OscOut hb;
    hb.str("/getHeartbeat");
    hb.str(",");
    oscSend(hb);
  }

  // AbleSet only pushes values when they change, so one lost packet means a
  // stale display until the next change. Re-pull the full state periodically
  // so any missed update self-heals within a few seconds.
  if (absetUp && now - lastResync > RESYNC_MS) {
    lastResync = now;
    sendResync();
  }

  // --- (re)subscribe ---
  if (wifiUp && !absetUp && now - lastSubscribe > RESUBSCRIBE_MS) {
    lastSubscribe = now;
    if (!haveAblesetIp || (uint32_t)abletonIp == 0) {
      haveAblesetIp = false;
      discoverAbleset();
      if (!haveAblesetIp)
        Serial.printf("No AbleSet yet, broadcasting /subscribe to %s:%u\n",
                      broadcastIp().toString().c_str(), ABLESET_PORT);
    }
    sendSubscribe();
  }

  // --- AbleSet hunt across networks ---
  // Connected, but AbleSet has never answered here (or has been silent well
  // past its timeout): it probably lives on another configured network.
  if (wifiUp && !absetUp &&
      now - wifiConnectedAt > ABLESET_HUNT_MS &&
      packetAgeMs() > ABLESET_HUNT_MS) {
    Serial.printf("No AbleSet on \"%s\" after %lus — trying the next network\n",
                  connectedSsid.c_str(), (unsigned long)(ABLESET_HUNT_MS / 1000));
    wifiUp = false;
    haveAblesetIp = false;
    connectedSsid = "";
    WiFi.disconnect(false, true);
    delay(100);
    connectWifi();
    lastWifiRetry = millis();
    if (gaveUp) return;
    gfx->fillScreen(COL_BG);
    forceRedraw = true;
    if (wifiUp) {
      discoverAbleset();
      sendSubscribe();
      lastSubscribe = millis();
    }
    now = millis();
    lastWifiCheck = now;
  }

  takeSnapshot();
  render();
  delay(10);
}
