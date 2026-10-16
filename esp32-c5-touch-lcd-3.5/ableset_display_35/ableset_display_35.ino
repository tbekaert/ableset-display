/*
 * AbleSet stage display — Waveshare ESP32-C5-Touch-LCD-3.5
 * ---------------------------------------------------------
 * Connects to a WiFi network (2.4 or 5 GHz), subscribes to AbleSet's OSC
 * feed, and shows the current song, section, and what's coming next on the
 * 3.5" 480x320 LCD.
 *
 * Board:    Waveshare ESP32-C5-Touch-LCD-3.5 (ST7796 320x480, CH32V003 IO expander)
 * Requires: arduino-esp32 core 3.3.10  (Boards Manager: "esp32" by Espressif)
 *           GFX Library for Arduino 1.6.x (moononournation)
 *           U8g2 (olikraus) — only used for its fonts
 *
 * Tools menu: Board            = "ESP32C5 Dev Module"
 *             USB CDC On Boot  = Enabled
 *             PSRAM            = Enabled        (flicker-free framebuffer)
 *             Partition Scheme = "Huge APP (3MB No OTA/1MB SPIFFS)" if the
 *                                default one reports the sketch is too big
 *
 * Changes from the 1.47" version:
 *  - LCD reset, backlight and panel power go through the CH32V003 IO expander
 *    on I2C (0x24), not GPIOs.
 *  - Everything is drawn into a framebuffer in PSRAM, then only the changed
 *    rows are pushed to the panel, so updates don't flicker.
 *  - Text uses scalable U8g2 fonts (Logisoso, UTF-8 so accents render) and is
 *    auto-fitted to each zone.
 *  - New layout: left column with song / current section (with a per-bar
 *    progress bar) / next section in three equal rows, and a setlist sidebar on the right with the time left
 *    in the set.
 *  - No RGB LED on this board, so the LED code is gone.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <lwip/sockets.h>
#include <math.h>
#include <U8g2lib.h>              // fonts only; must come before Arduino_GFX
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
// before giving up and showing a Retry screen (tap anywhere / press BOOT).
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

// Section progress bar: true = the current bar fills up beat by beat,
// false = whole bars light up as they start (one redraw per bar).
static const bool     PROGRESS_BY_BEAT = true;

// Song descriptions from AbleSet (the song's "description" field), shown in
// a smaller dim font under the song name and under every name in the
// setlist. false = names only, which also fits 9 setlist rows instead of 5.
static const bool     SHOW_SONG_DESCRIPTIONS = true;

// Screen orientation. 1 is correct for this board (confirmed on hardware);
// 3 is the same landscape view upside down.
static const uint8_t  ROTATION = 1;

// Waveshare's own example runs this panel at 60 MHz. If you ever see
// corrupted pixels, drop it back to 40000000.
static const uint32_t LCD_SPI_HZ = 60000000;

// ============================================================================
// HARDWARE — Waveshare ESP32-C5-Touch-LCD-3.5
// ============================================================================

#define LCD_SCK   6
#define LCD_MOSI  7
#define LCD_CS    8
#define LCD_DC    5

#define I2C_SDA  27
#define I2C_SCL  26

#define TOUCH_ADDR  0x38    // FT6336 capacitive touch, same I2C bus
#define RETRY_BTN_PIN    28      // BOOT button, active low — doubles as "retry"

// CH32V003 IO expander (register map from Waveshare's Board_IO example)
#define EXIO_ADDR        0x24
#define EXIO_REG_MODE    0x02   // direction, bit = 1 -> output
#define EXIO_REG_OUTPUT  0x03
#define EXIO_REG_PWM     0x05   // backlight PWM, 0-255
#define EXIO_TP_RST      0
#define EXIO_LCD_RST     1
#define EXIO_POWER       5      // panel power enable — must be high

static const int SCR_W = 480;
static const int SCR_H = 320;

Arduino_DataBus *bus   = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_ST7796  *panel = new Arduino_ST7796(bus, GFX_NOT_DEFINED /* reset via expander */,
                                            ROTATION, true /* IPS: inversion on */, 320, 480);
Arduino_Canvas  *canvas = new Arduino_Canvas(SCR_W, SCR_H, panel);

// Everything draws through `g`: the PSRAM framebuffer when it could be
// allocated, otherwise straight to the panel (works, but redraws flicker).
Arduino_GFX *g = panel;
bool useCanvas = false;

// ============================================================================
// LAYOUT (480 x 320)
// ============================================================================

// Left column: thin status bar, then song / section / next in equal rows.
// Right column: the setlist.
static const int GAP        = 4;
static const int LEFT_W     = 296;
static const int SIDE_X     = LEFT_W + GAP;          // 300
static const int SIDE_W     = SCR_W - SIDE_X;        // 180, fits "VELVET HOUR" at F22
static const int Z_STATUS_Y = 0,   Z_STATUS_H = 30;
static const int ROW_H      = (SCR_H - Z_STATUS_H - 3 * GAP) / 3;   // 94
static const int Z_SONG_Y   = Z_STATUS_H + GAP;                     // 34
static const int Z_SECT_Y   = Z_SONG_Y + ROW_H + GAP;               // 132
static const int Z_NEXT_Y   = Z_SECT_Y + ROW_H + GAP;               // 230
static const int Z_NEXT_H   = SCR_H - Z_NEXT_Y;                     // 90 (+ rounding)

// Setlist sidebar
static const int SL_HEAD_H  = Z_STATUS_H;
static const int SL_NAME_H  = 30;                    // name line of a setlist row
static const int SL_ROW_H   = SHOW_SONG_DESCRIPTIONS ? 54 : SL_NAME_H;
static const int SL_PITCH   = SL_ROW_H + 2;
static const int SL_ROWS    = (SCR_H - SL_HEAD_H - GAP + 2) / SL_PITCH;   // 5 with descriptions, 9 without
static const int MAX_SONGS  = 64;

static const uint16_t COL_BG     = RGB565_BLACK;
static const uint16_t COL_TEXT   = RGB565_WHITE;
static const uint16_t COL_DIM    = 0x8410;
static const uint16_t COL_BAR    = 0x10A2;  // status bar background
static const uint16_t COL_CELL   = 0x2124;  // "next song" cell background
static const uint16_t COL_OK     = 0x07E0;
static const uint16_t COL_WARN   = 0xFD20;
static const uint16_t COL_ERR    = 0xF800;
static const uint16_t COL_LOOP   = 0xFFE0;  // loop cell background (yellow)
static const uint16_t COL_SIDE   = 0x0861;  // setlist background
static const uint16_t COL_CURROW = 0x3189;  // current song row
static const uint16_t COL_PAST   = 0x5AEB;  // songs already played
static const uint16_t COL_FUTURE = 0xCE59;  // songs still to come

// Logisoso (condensed bold, very legible at a distance), largest first.
// _tf = full Latin-1 set, so accented names render.
static const uint8_t *const FONTS[] = {
  u8g2_font_logisoso58_tf, u8g2_font_logisoso54_tf, u8g2_font_logisoso50_tf,
  u8g2_font_logisoso46_tf, u8g2_font_logisoso42_tf, u8g2_font_logisoso38_tf,
  u8g2_font_logisoso34_tf, u8g2_font_logisoso32_tf, u8g2_font_logisoso30_tf,
  u8g2_font_logisoso28_tf, u8g2_font_logisoso26_tf, u8g2_font_logisoso24_tf,
  u8g2_font_logisoso22_tf, u8g2_font_logisoso20_tf, u8g2_font_logisoso18_tf,
  u8g2_font_logisoso16_tf,
};
static const int N_FONTS = sizeof(FONTS) / sizeof(FONTS[0]);
enum : int { F58 = 0, F54, F50, F46, F42, F38, F34, F32, F30, F28, F26, F24, F22, F20, F18, F16 };

// Fixed text sizes (indices into FONTS). Text that doesn't fit on one line is
// truncated with "...". F38 is one step bigger, F30 one step smaller.
static const int LEFT_FONT = F34;   // song / section / next, all three rows
static const int SIDE_FONT = F22;   // every name in the setlist
static const int SONG_DESC_FONT = F18;   // description under the song name
static const int SIDE_DESC_FONT = F16;   // description under each setlist name

static const uint8_t *const FONT_UI    = u8g2_font_helvB12_tf;  // status bar
static const uint8_t *const FONT_LABEL = u8g2_font_helvB10_tf;  // small cell labels

// ============================================================================
// STATE
// ============================================================================

// Networking runs in its own task (see NETWORK RECEIVE TASK): incoming OSC is
// parsed the moment it arrives into `net`, and loop() renders from a snapshot
// (`st`) taken under a short lock. Drawing can't make us drop packets anymore.
// One UDP socket bound to LOCAL_PORT: the receive task reads it, loop() sends
// from it (lwIP is built full-duplex, so that's safe), and AbleSet always
// sees our requests coming from LOCAL_PORT, as with the previous version.
int oscSock = -1;
SemaphoreHandle_t stateLock = nullptr;
IPAddress abletonIp;             // written under stateLock (see setAblesetIp)
volatile bool haveAblesetIp = false;
String connectedSsid = "";

struct State {
  String song      = "";
  String songDesc  = "";
  String section   = "";
  String nextSect  = "";
  String nextSong  = "";
  bool   inLoop    = false;
  int    loopCount = 0;
  int    loopTarget = 0;
  uint16_t sectCol = 0x2104;
  uint16_t nextSectCol = 0x2104;
  int songIndex    = -1;
  int songCount    = 0;
  bool playing     = false;
  bool liveLinked  = false;
  float setLeft    = -1;      // seconds left in the set, -1 = unknown
  float beatPos    = -1;      // playhead, in beats (whole beats while playing)
  float sectStart  = 0;       // current section, in beats
  float sectEnd    = 0;
  int   tsNum      = 4;       // time signature
  int   tsDen      = 4;
  int   onBattery  = -1;      // AbleSet laptop on battery: 1 yes, 0 no, -1 unknown
} net, st, drawn;   // net: written by the receive task; st: snapshot for drawing

// Setlist. AbleSet sends it at subscribe time and again only if it changes;
// the periodic resync doesn't ask for it.
String netSongs[MAX_SONGS];   // written by the receive task
String netSongDescs[MAX_SONGS];
int netNSongs = 0;
int netNSongDescs = 0;
uint32_t netSongsVersion = 0; // bumped when either list changes
String songs[MAX_SONGS];      // snapshot used for drawing
String songDescs[MAX_SONGS];
int nSongs = 0;
uint32_t songsVersion = 0;    // bumped whenever the list changes

String drawnNext = "";     // signature of what the next cell currently shows
String drawnSide = "";     // signature of what the setlist rows currently show
String drawnSideHead = ""; // signature of the setlist header (time left)
String drawnProgress = ""; // signature of the section progress bar

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
bool touchOk = false;

// ============================================================================
// OSC TYPES
//
// These MUST stay above the first function definition in the file. The Arduino
// IDE auto-generates prototypes for every function and inserts them just before
// the first one — so any type used in a function signature has to be declared
// before that point, or the generated prototypes won't compile.
// ============================================================================

struct OscOut {
  uint8_t d[1024];
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

// Result of fitting a string into a box (see fitText). Declared up here for
// the same prototype reason as the OSC types.
struct FitResult {
  String lines[3];
  int nLines = 0;
  int font = 0;      // index into FONTS
  int capH = 0;      // cap height of that font, px
  int gap = 0;       // vertical gap between lines, px
};

enum Align : uint8_t { ALIGN_CENTER, ALIGN_LEFT };

// What the third left row shows (next section / loop / next song).
struct NextContent { bool show; const char *label; String val; uint16_t bg; };

// Where the playhead is inside the current section, for the progress bar.
struct SectProgress {
  bool  show;
  int   bars;        // section length in bars (rounded up)
  float beatsPerBar; // in quarter-note beats, e.g. 4 for 4/4, 3 for 6/8
  float len;         // section length in beats
  float lit;         // beats elapsed, counting the current one
};

// ============================================================================
// BOARD — CH32V003 IO expander (LCD reset, backlight, panel power)
// ============================================================================

static uint8_t exioOut = 0;

static bool exioWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(EXIO_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Same sequence as Waveshare's Board_IO_Init: all pins briefly output-low,
// then pulse both resets and switch panel power on.
static bool exioInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  bool ok = exioWrite(EXIO_REG_MODE, 0xFF) &&
            exioWrite(EXIO_REG_OUTPUT, 0x00) &&
            exioWrite(EXIO_REG_PWM, 0x00) &&
            exioWrite(EXIO_REG_MODE, (1 << EXIO_TP_RST) | (1 << EXIO_LCD_RST) | (1 << EXIO_POWER));
  if (!ok) return false;
  exioOut = (1 << EXIO_TP_RST) | (1 << EXIO_LCD_RST);
  exioWrite(EXIO_REG_OUTPUT, exioOut);
  delay(50);
  exioOut = 0;
  exioWrite(EXIO_REG_OUTPUT, exioOut);
  delay(50);
  exioOut = (1 << EXIO_TP_RST) | (1 << EXIO_LCD_RST) | (1 << EXIO_POWER);
  return exioWrite(EXIO_REG_OUTPUT, exioOut);
}

static void exioSet(uint8_t pin, bool level) {
  if (level) exioOut |= (1 << pin); else exioOut &= ~(1 << pin);
  exioWrite(EXIO_REG_OUTPUT, exioOut);
}

static void lcdReset() {
  exioSet(EXIO_LCD_RST, true);  delay(10);
  exioSet(EXIO_LCD_RST, false); delay(20);
  exioSet(EXIO_LCD_RST, true);  delay(120);
}

static void setBacklight(uint8_t pct) {
  if (pct > 100) pct = 100;
  exioWrite(EXIO_REG_PWM, (uint8_t)(pct * 255U / 100U));
}

// ============================================================================
// FRAMEBUFFER
// ============================================================================

// Copy rows [y, y+h) of the framebuffer to the panel. Zones are full width,
// so each zone is one contiguous block of the buffer.
static void pushRows(int y, int h) {
  if (!useCanvas) return;
  if (y < 0) { h += y; y = 0; }
  if (y + h > SCR_H) h = SCR_H - y;
  if (h <= 0) return;
  panel->draw16bitRGBBitmap(0, y, canvas->getFramebuffer() + (size_t)y * SCR_W, SCR_W, h);
}

// Copy any rectangle of the framebuffer to the panel. Zones that are narrower
// than the screen go through a scratch buffer so we only send their own pixels.
static uint16_t *pushBuf = nullptr;
static void pushRect(int x, int y, int w, int h) {
  if (!useCanvas) return;
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > SCR_W) w = SCR_W - x;
  if (y + h > SCR_H) h = SCR_H - y;
  if (w <= 0 || h <= 0) return;
  if (w == SCR_W) { pushRows(y, h); return; }
  if (!pushBuf) pushBuf = (uint16_t *)ps_malloc((size_t)SCR_W * SCR_H * 2);
  if (!pushBuf) { pushRows(y, h); return; }
  const uint16_t *fb = canvas->getFramebuffer();
  for (int r = 0; r < h; r++)
    memcpy(pushBuf + (size_t)r * w, fb + (size_t)(y + r) * SCR_W + x, (size_t)w * 2);
  panel->draw16bitRGBBitmap(x, y, pushBuf, w, h);
}

static void clearScreen() {
  g->fillScreen(COL_BG);
  pushRows(0, SCR_H);
}

// ============================================================================
// TEXT
// ============================================================================

// Decode one UTF-8 character starting at s[i]; advances i.
static uint32_t nextCodepoint(const String &s, unsigned &i) {
  uint8_t c = (uint8_t)s[i++];
  if (c < 0x80) return c;
  int extra = (c >= 0xF0) ? 3 : (c >= 0xE0) ? 2 : (c >= 0xC0) ? 1 : 0;
  uint32_t cp = c & (0x3F >> extra);
  while (extra-- > 0 && i < s.length()) cp = (cp << 6) | ((uint8_t)s[i++] & 0x3F);
  return cp;
}

static void appendCodepoint(String &out, uint32_t cp) {
  if (cp < 0x80) { out += (char)cp; return; }
  out += (char)(0xC0 | (cp >> 6));       // the fonts only cover up to U+00FF
  out += (char)(0x80 | (cp & 0x3F));
}

// Keep only what the fonts can draw (ASCII + Latin-1): œ -> OE, typographic
// dashes/quotes -> ASCII, anything else (emoji, other scripts) is dropped.
// With upper, also uppercase (é -> É). Strings are UTF-8.
static String foldUtf8(const String &in, bool upper) {
  String out;
  unsigned i = 0;
  while (i < in.length()) {
    uint32_t cp = nextCodepoint(in, i);
    if (!upper)                                         { /* keep case */ }
    else if (cp >= 'a' && cp <= 'z')                    cp -= 32;
    else if (cp >= 0xE0 && cp <= 0xFE && cp != 0xF7)    cp -= 0x20;
    else if (cp == 0xFF)                                cp = 'Y';
    if (cp == 0x152 || cp == 0x153)                   { out += upper ? "OE" : "oe"; continue; }
    else if (cp == 0x2013 || cp == 0x2014 || cp == 0x2212) cp = '-';
    else if (cp >= 0x2018 && cp <= 0x201B)              cp = '\'';
    else if (cp >= 0x201C && cp <= 0x201F)              cp = '"';
    else if (cp == 0x2026)                            { out += "..."; continue; }
    if (cp > 0xFF) continue;
    appendCodepoint(out, cp);
  }
  return out;
}

static String upperUtf8(const String &in) { return foldUtf8(in, true); }

// Drop the last UTF-8 character.
static void popUtf8(String &s) {
  int i = s.length() - 1;
  while (i > 0 && ((uint8_t)s[i] & 0xC0) == 0x80) i--;
  s.remove(i < 0 ? 0 : i);
}

// Arduino_GFX's getTextBounds() gives accented (non-ASCII) glyphs zero width
// with U8g2 fonts, even though it draws them fine. Measure with each Latin-1
// letter swapped for its unaccented base letter, which has the same advance
// in these fonts.
static String measureForm(const String &s) {
  static const char BASE[] = "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYPsaaaaaaaceeeeiiiidnooooo/ouuuuypy";
  String out;
  unsigned i = 0;
  while (i < s.length()) {
    uint32_t cp = nextCodepoint(s, i);
    if (cp < 0x80)                    out += (char)cp;
    else if (cp >= 0xC0 && cp <= 0xFF) out += BASE[cp - 0xC0];
    else                               out += 'O';   // other Latin-1 symbols: approximate
  }
  return out;
}

static int textWidth(const String &s, int16_t *xOff) {
  int16_t x1, y1; uint16_t w, h;
  g->getTextBounds(measureForm(s), 0, 0, &x1, &y1, &w, &h);
  if (xOff) *xOff = x1;
  return w;
}

static int capHeight() {
  int16_t x1, y1; uint16_t w, h;
  g->getTextBounds("H", 0, 0, &x1, &y1, &w, &h);
  return h;
}

// Greedy word wrap by pixel width with the current font.
// Returns the number of lines, or -1 if it doesn't fit in maxLines (or a
// single word is wider than the box).
static int wrapPx(const String &txt, int maxW, String *out, int maxLines) {
  int n = 0, i = 0, len = txt.length();
  String line = "";
  while (i < len) {
    while (i < len && txt[i] == ' ') i++;
    if (i >= len) break;
    int j = i;
    while (j < len && txt[j] != ' ') j++;
    String word = txt.substring(i, j);
    i = j;
    if (textWidth(word, nullptr) > maxW) return -1;
    if (line.length() == 0) {
      line = word;
    } else if (textWidth(line + " " + word, nullptr) <= maxW) {
      line += " ";
      line += word;
    } else {
      if (n >= maxLines) return -1;
      out[n++] = line;
      line = word;
    }
  }
  if (line.length()) {
    if (n >= maxLines) return -1;
    out[n++] = line;
  }
  return n;
}

// Find the largest font (between fMax and fMin, FONTS indices) at which txt
// fits in a w x h box on at most maxLines lines. Falls back to one truncated
// line in the smallest font.
static FitResult fitText(const String &txt, int w, int h, int fMax, int fMin, int maxLines) {
  FitResult r;
  if (maxLines > 3) maxLines = 3;
  for (int f = fMax; f <= fMin && f < N_FONTS; f++) {
    g->setFont(FONTS[f]);
    int cap = capHeight();
    int gap = max(6, cap * 2 / 5);
    int lines = min(maxLines, (h + gap) / (cap + gap));
    if (lines < 1) continue;
    int n = wrapPx(txt, w, r.lines, lines);
    if (n > 0) { r.nLines = n; r.font = f; r.capH = cap; r.gap = gap; return r; }
  }
  r.font = min(fMin, N_FONTS - 1);
  g->setFont(FONTS[r.font]);
  r.capH = capHeight();
  r.gap = 0;
  String s = txt;
  while (s.length() && textWidth(s + "...", nullptr) > w) popUtf8(s);
  s.trim();
  r.lines[0] = s + "...";
  r.nLines = 1;
  return r;
}

static void drawFit(int x, int y, int w, int h, const FitResult &r, uint16_t fg, Align align) {
  if (r.nLines == 0) return;
  g->setFont(FONTS[r.font]);
  g->setTextColor(fg);
  int total = r.nLines * r.capH + (r.nLines - 1) * r.gap;
  int base = y + (h - total) / 2 + r.capH;
  for (int i = 0; i < r.nLines; i++) {
    int16_t xOff;
    int tw = textWidth(r.lines[i], &xOff);
    int cx = (align == ALIGN_CENTER) ? x + (w - tw) / 2 - xOff : x - xOff;
    g->setCursor(cx, base);
    g->print(r.lines[i]);
    base += r.capH + r.gap;
  }
}

static void drawText(int x, int y, int w, int h, const String &txt, uint16_t fg,
                     int fMax, int fMin, int maxLines, Align align) {
  if (txt.length() == 0) return;
  FitResult r = fitText(txt, w, h, fMax, fMin, maxLines);
  drawFit(x, y, w, h, r, fg, align);
}

// Pick black or white text for a given background, whichever contrasts more.
// WCAG relative luminance; crossover at L = 0.179.
static float srgbLin(int v) {
  float c = v / 255.0f;
  return (c <= 0.03928f) ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}

static uint16_t textOn(uint16_t bg) {
  int r = ((bg >> 11) & 0x1F) * 255 / 31;
  int gr = ((bg >> 5) & 0x3F) * 255 / 63;
  int b = (bg & 0x1F) * 255 / 31;
  float L = 0.2126f * srgbLin(r) + 0.7152f * srgbLin(gr) + 0.0722f * srgbLin(b);
  return (L > 0.1791f) ? COL_BG : COL_TEXT;
}

// ============================================================================
// ZONES
// ============================================================================

static void drawStatusBar() {
  const int y = Z_STATUS_Y, h = Z_STATUS_H, mid = y + h / 2;
  g->fillRect(0, y, LEFT_W, h, COL_BAR);
  g->setFont(FONT_UI);
  int base = mid + capHeight() / 2;

  uint16_t wifiCol  = wifiUp ? COL_OK : COL_ERR;
  uint16_t absetCol = absetUp ? (st.liveLinked ? COL_OK : COL_WARN) : COL_ERR;
  g->fillCircle(15, mid, 6, wifiCol);
  g->fillCircle(34, mid, 6, absetCol);

  // Power state of the computer running AbleSet.
  if (absetUp && st.onBattery == 1) {
    const char *msg = "ON BATTERY";
    int16_t xOff;
    int tw = textWidth(msg, &xOff);
    int bx = 48, bh = h - 8;
    g->fillRoundRect(bx, mid - bh / 2, tw + 16, bh, 4, COL_ERR);
    g->setTextColor(COL_TEXT);
    g->setCursor(bx + 8 - xOff, base);
    g->print(msg);
  } else if (absetUp && st.onBattery == 0) {
    g->setTextColor(COL_DIM);
    g->setCursor(50, base);
    g->print("AC POWER");
  }

  const char *tp = st.playing ? "PLAY" : "STOP";
  uint16_t tc = st.playing ? COL_OK : COL_DIM;
  int16_t xOff;
  int tw = textWidth(tp, &xOff);
  int tx = LEFT_W - 10 - tw;
  int ix = tx - 20;   // icon left edge
  if (st.playing) g->fillTriangle(ix, mid - 7, ix, mid + 7, ix + 12, mid, tc);
  else            g->fillRect(ix, mid - 6, 12, 12, tc);
  g->setTextColor(tc);
  g->setCursor(tx - xOff, base);
  g->print(tp);

  pushRect(0, y, LEFT_W, h);
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

// Descriptions can be several lines in AbleSet; the display has room for one.
// Keep the first non-empty line, as typed (not uppercased: that keeps it
// visually secondary to the name).
static String cleanDesc(const String &in) {
  if (!SHOW_SONG_DESCRIPTIONS) return String("");
  String d = in;
  d.replace("\r", "\n");
  int start = 0, len = d.length();
  while (start < len) {
    int nl = d.indexOf('\n', start);
    String line = (nl < 0) ? d.substring(start) : d.substring(start, nl);
    line.trim();
    while (line.indexOf("  ") >= 0) line.replace("  ", " ");
    if (line.length()) return foldUtf8(line, false);
    if (nl < 0) break;
    start = nl + 1;
  }
  return String("");
}

// ---- left column content -----------------------------------------------------

// Text boxes of the three left rows (the next row has a small label on top).
static const int LX = 10, LW = LEFT_W - 20;
static const int SONG_BOX_Y = Z_SONG_Y,      SONG_BOX_H = ROW_H;
// Section progress bar along the bottom of the section row.
static const int PB_H       = 10;
static const int PB_MARGIN  = 9;
static const int PB_Y       = Z_SECT_Y + ROW_H - PB_MARGIN - PB_H;
static const int PB_GAP     = 3;     // gap between bar segments
static const int PB_MAX_SEGMENTS = 32;   // longer sections get one continuous bar
static const int SECT_BOX_Y = Z_SECT_Y + 2,  SECT_BOX_H = PB_Y - Z_SECT_Y - 4;
static const int NEXT_BOX_Y = Z_NEXT_Y + 22, NEXT_BOX_H = Z_NEXT_H - 26;

static String songText() {
  String name = cleanName(st.song);
  if (name.length()) return upperUtf8(name);
  return !wifiUp ? String("NO WIFI") : (absetUp ? String("--") : String("NO ABLESET"));
}

static String sectionText() {
  return upperUtf8(st.section.length() ? st.section : String("--"));
}

// Third row: the loop counter while looping, otherwise the next section in its
// own colour, otherwise (last section) the next song.
static NextContent nextContent() {
  NextContent n = { true, "", "", COL_BG };
  if (st.inLoop) {
    char buf[24];
    int c = st.loopCount + LOOP_COUNTER_OFFSET;
    if (st.loopTarget > 0) snprintf(buf, sizeof(buf), "%d / %d", c, st.loopTarget);
    else                   snprintf(buf, sizeof(buf), "%d", c);
    n.label = "LOOP"; n.val = buf; n.bg = COL_LOOP;
  } else if (st.nextSect.length()) {
    n.label = "NEXT"; n.val = st.nextSect; n.bg = st.nextSectCol;
  } else if (st.nextSong.length()) {
    n.label = "NEXT SONG"; n.val = cleanName(st.nextSong); n.bg = COL_CELL;
  } else {
    n.show = false;
  }
  n.val = upperUtf8(n.val);
  return n;
}

static void drawSong() {
  g->fillRect(0, Z_SONG_Y, LEFT_W, ROW_H, COL_BG);
  bool haveName = cleanName(st.song).length() > 0;
  uint16_t c = haveName ? COL_TEXT : COL_ERR;
  String desc = haveName ? cleanDesc(st.songDesc) : String("");
  if (desc.length()) {
    // Name in the upper part of the row, description under it, smaller and dim.
    drawText(LX, SONG_BOX_Y + 4, LW, 50, songText(), c, LEFT_FONT, LEFT_FONT, 1, ALIGN_LEFT);
    drawText(LX, SONG_BOX_Y + 58, LW, 30, desc, COL_DIM, SONG_DESC_FONT, SONG_DESC_FONT, 1, ALIGN_LEFT);
  } else {
    drawText(LX, SONG_BOX_Y, LW, SONG_BOX_H, songText(), c, LEFT_FONT, LEFT_FONT, 1, ALIGN_LEFT);
  }
  pushRect(0, Z_SONG_Y, LEFT_W, ROW_H);
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
  if (!absetUp || st.beatPos < 0 || p.len < 0.5f) return p;

  p.beatsPerBar = st.tsNum * 4.0f / (st.tsDen > 0 ? st.tsDen : 4);
  if (p.beatsPerBar <= 0) p.beatsPerBar = 4;
  p.bars = max(1, (int)ceilf(p.len / p.beatsPerBar - 0.01f));

  float rel = st.beatPos - st.sectStart;
  if (rel < 0) rel = 0;
  if (PROGRESS_BY_BEAT) {
    // The beat we're on counts as lit while playing (like a metronome light).
    // Redraws a small strip once per beat, which is cheap.
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

// Draws into the framebuffer only; the caller pushes the rows.
static void drawProgress() {
  g->fillRect(0, PB_Y, LEFT_W, PB_H, st.sectCol);
  SectProgress p = sectionProgress();
  if (!p.show) return;

  uint16_t on  = textOn(st.sectCol);
  uint16_t off = blend565(st.sectCol, on, 0.28f);

  if (p.bars > PB_MAX_SEGMENTS) {
    int fw = (int)(LW * (p.lit / p.len) + 0.5f);
    g->fillRect(LX, PB_Y, LW, PB_H, off);
    if (fw > 0) g->fillRect(LX, PB_Y, fw, PB_H, on);
    return;
  }

  // One segment per bar; it fills beat by beat, or all at once (PROGRESS_BY_BEAT).
  float segW = (LW - PB_GAP * (p.bars - 1)) / (float)p.bars;
  for (int i = 0; i < p.bars; i++) {
    int x0 = LX + (int)(i * (segW + PB_GAP) + 0.5f);
    int x1 = LX + (int)(i * (segW + PB_GAP) + segW + 0.5f);
    int w = x1 - x0;
    float barStart = i * p.beatsPerBar;
    float barLen = min(p.beatsPerBar, p.len - barStart);
    float f = (p.lit - barStart) / barLen;
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    int fw = (int)(w * f + 0.5f);
    g->fillRect(x0, PB_Y, w, PB_H, off);
    if (fw > 0) g->fillRect(x0, PB_Y, fw, PB_H, on);
  }
}

static void drawSection() {
  g->fillRect(0, Z_SECT_Y, LEFT_W, ROW_H, st.sectCol);
  drawText(LX, SECT_BOX_Y, LW, SECT_BOX_H, sectionText(), textOn(st.sectCol),
           LEFT_FONT, LEFT_FONT, 1, ALIGN_LEFT);
  drawProgress();
  pushRect(0, Z_SECT_Y, LEFT_W, ROW_H);
}

static void drawNext(bool force) {
  NextContent n = nextContent();
  String sig = String(n.show) + n.label + "|" + n.val + "|" + String(n.bg);
  if (!force && sig == drawnNext) return;
  drawnNext = sig;

  const int y = Z_NEXT_Y, h = Z_NEXT_H;
  g->fillRect(0, y, LEFT_W, h, COL_BG);
  if (n.show) {
    uint16_t fg = textOn(n.bg);
    g->fillRect(0, y, LEFT_W, h, n.bg);
    g->setFont(FONT_LABEL);
    g->setTextColor(fg);
    g->setCursor(LX, y + 6 + capHeight());
    g->print(n.label);
    drawText(LX, NEXT_BOX_Y, LW, NEXT_BOX_H, n.val, fg, LEFT_FONT, LEFT_FONT, 1, ALIGN_LEFT);
  }
  pushRect(0, y, LEFT_W, h);
}

// ---- setlist -----------------------------------------------------------------

static const int SL_NAME_X = SIDE_X + 8;
static const int SL_NAME_W = SIDE_W - 14;

static String formatDuration(float secs) {
  if (secs < 0) return String("");
  long t = (long)(secs + 0.5f);
  char buf[16];
  if (t >= 3600) snprintf(buf, sizeof(buf), "%ld:%02ld:%02ld", t / 3600, (t / 60) % 60, t % 60);
  else           snprintf(buf, sizeof(buf), "%ld:%02ld", t / 60, t % 60);
  return String(buf);
}

// Right column, part 1: header with the time left in the set. Redrawn on its
// own (a small strip) because the time changes every second.
static void drawSetlistHeader(bool force) {
  String left = absetUp ? formatDuration(st.setLeft) : String("");
  if (!force && left == drawnSideHead) return;
  drawnSideHead = left;

  g->fillRect(SIDE_X, 0, SIDE_W, SL_HEAD_H, COL_BAR);
  g->setFont(FONT_LABEL);
  int base = SL_HEAD_H / 2 + capHeight() / 2;
  g->setTextColor(COL_DIM);
  g->setCursor(SIDE_X + 8, base);
  g->print("SETLIST");
  if (left.length()) {
    String t = left + " LEFT";
    int16_t xOff;
    int tw = textWidth(t, &xOff);
    g->setTextColor(COL_TEXT);
    g->setCursor(SCR_W - 8 - tw - xOff, base);
    g->print(t);
  }
  pushRect(SIDE_X, 0, SIDE_W, SL_HEAD_H);
}

// Right column, part 2: the setlist around the current song (one played song
// above it, then what's next). Only redrawn when the song or the list changes.
static void drawSetlist(bool force) {
  String sig = String(absetUp) + "|" + String(songsVersion) + "|" + String(st.songIndex);
  if (!force && sig == drawnSide) return;
  drawnSide = sig;

  const int top = SL_HEAD_H;
  g->fillRect(SIDE_X, top, SIDE_W, SCR_H - top, COL_SIDE);

  if (absetUp && nSongs > 0) {
    int cur = st.songIndex;
    int start = (cur > 0) ? cur - 1 : 0;
    if (start + SL_ROWS > nSongs) start = max(0, nSongs - SL_ROWS);
    int y = SL_HEAD_H + GAP;
    for (int i = start; i < nSongs && i < start + SL_ROWS; i++, y += SL_PITCH) {
      bool isCur = (i == cur);
      uint16_t bg = isCur ? COL_CURROW : COL_SIDE;
      uint16_t fg = isCur ? COL_TEXT : (i < cur ? COL_PAST : COL_FUTURE);
      if (isCur) g->fillRect(SIDE_X, y, SIDE_W, SL_ROW_H, bg);

      drawText(SL_NAME_X, y, SL_NAME_W, SL_NAME_H, upperUtf8(cleanName(songs[i])), fg,
               SIDE_FONT, SIDE_FONT, 1, ALIGN_LEFT);
      // Description under the name, in the name's colour faded towards the
      // row background so it stays secondary whatever the row's state.
      String desc = cleanDesc(songDescs[i]);
      if (desc.length())
        drawText(SL_NAME_X, y + SL_NAME_H, SL_NAME_W, SL_ROW_H - SL_NAME_H, desc,
                 blend565(fg, bg, 0.45f), SIDE_DESC_FONT, SIDE_DESC_FONT, 1, ALIGN_LEFT);
    }
  }
  pushRect(SIDE_X, top, SIDE_W, SCR_H - top);
}

static void render() {
  if (forceRedraw || wifiUp != drawnWifiUp || absetUp != drawnAbsetUp ||
      st.onBattery != drawn.onBattery ||
      st.playing != drawn.playing || st.liveLinked != drawn.liveLinked) {
    drawStatusBar();
    drawnWifiUp = wifiUp; drawnAbsetUp = absetUp; drawn.onBattery = st.onBattery;
    drawn.playing = st.playing; drawn.liveLinked = st.liveLinked;
  }

  if (forceRedraw || st.song != drawn.song || st.songDesc != drawn.songDesc) {
    drawSong();
    drawn.song = st.song;
    drawn.songDesc = st.songDesc;
  }

  String ps = progressSig();
  if (forceRedraw || st.section != drawn.section || st.sectCol != drawn.sectCol) {
    drawSection();                 // includes the progress bar
    drawn.section = st.section;
    drawn.sectCol = st.sectCol;
    drawnProgress = ps;
  } else if (ps != drawnProgress) {
    drawProgress();                // only the bar strip changes (per beat or per bar)
    pushRect(0, PB_Y, LEFT_W, PB_H);
    drawnProgress = ps;
  }

  drawNext(forceRedraw);
  drawSetlistHeader(forceRedraw);
  drawSetlist(forceRedraw);

  forceRedraw = false;
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
  "/setlist/activeSongDescription",
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
  "/setlist/songs",
  "/setlist/songDescriptions",
  "/setlist/remainingTimeInSet",
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

// Full state, including the setlist. Sent right after subscribing.
static void sendGetValues() {
  OscOut g;
  g.str("/getValues");
  g.str(",i");
  g.i32(LOCAL_PORT);
  oscSend(g);
}

// Periodic self-heal: only the small values. Skips the setlist (loaded once)
// and the values AbleSet pushes continuously anyway.
static const char *RESYNC_VALUES[] = {
  "/setlist/activeSongName",
  "/setlist/activeSongDescription",
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

// Resolve a colour message's arguments to a band colour, logging the raw
// input so bad renders can be diagnosed from serial output instead of guessed.
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
  } else if (!strcmp(addr, "/setlist/activeSongDescription")) {
    net.songDesc = (nArgs > 0 && args[0].type == 's') ? String(args[0].s) : String("");
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
  } else if (!strcmp(addr, "/setlist/songs")) {
    int n = 0;
    bool changed = false;
    for (int i = 0; i < nArgs && n < MAX_SONGS; i++) {
      if (args[i].type != 's' || !args[i].s) continue;
      if (n >= netNSongs || netSongs[n] != args[i].s) changed = true;
      netSongs[n++] = args[i].s;
    }
    if (n != netNSongs) changed = true;
    netNSongs = n;
    if (changed) {
      netSongsVersion++;
      Serial.printf("Setlist received: %d songs\n", n);
    }
  } else if (!strcmp(addr, "/setlist/songDescriptions")) {
    // Same shape as /setlist/songs: one string per song, same order.
    int n = 0;
    bool changed = false;
    for (int i = 0; i < nArgs && n < MAX_SONGS; i++) {
      const char *d = (args[i].type == 's' && args[i].s) ? args[i].s : "";
      if (n >= netNSongDescs || netSongDescs[n] != d) changed = true;
      netSongDescs[n++] = d;
    }
    if (n != netNSongDescs) changed = true;
    netNSongDescs = n;
    if (changed) netSongsVersion++;
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
  } else if (!strcmp(addr, "/setlist/remainingTimeInSet")) {
    net.setLeft = nArgs > 0 ? argNum(args[0]) : -1;
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

  static OscArg args[MAX_SONGS + 4];   // /setlist/songs sends one string per song
  int nArgs = 0;
  for (size_t i = 1; i < tlen && nArgs < MAX_SONGS + 4; i++) {
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
// TOUCH + RETRY SCREEN
// ============================================================================

static bool touchInit() {
  Wire.beginTransmission(TOUCH_ADDR);
  bool ok = Wire.endTransmission() == 0;
  Serial.println(ok ? "Touch FT6336 found" : "Touch FT6336 not found at 0x38 (BOOT still works)");
  return ok;
}

// Number of fingers on the screen (0 if none or touch unavailable).
static int touchCount() {
  if (!touchOk) return 0;
  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(TOUCH_ADDR, 1) != 1) return 0;
  return Wire.read() & 0x0F;
}

// True once per press (touch anywhere, or the BOOT button).
static bool wasPressed = true;
static bool retryPressed() {
  bool down = touchCount() > 0 || digitalRead(RETRY_BTN_PIN) == LOW;
  bool fired = down && !wasPressed;
  wasPressed = down;
  return fired;
}

static void drawRetryScreen() {
  g->fillScreen(COL_BG);

  String title = anyWifiThisRun ? "ABLESET NOT FOUND" : "NO WIFI";
  String names;
  for (int i = 0; i < N_WIFI; i++) {
    if (i) names += (i == N_WIFI - 1) ? " & " : ", ";
    names += WIFI_NETWORKS[i].ssid;
  }
  String detail = String("Tried ") + names + (WIFI_ROUNDS == 2 ? " twice" : "");

  drawText(20, 30, SCR_W - 40, 60, upperUtf8(title), COL_ERR, F34, F22, 1, ALIGN_CENTER);
  drawText(20, 92, SCR_W - 40, 34, upperUtf8(detail), COL_DIM, F20, F16, 1, ALIGN_CENTER);

  const int bw = 240, bh = 90, bx = (SCR_W - bw) / 2, by = 150;
  g->fillRoundRect(bx, by, bw, bh, 14, COL_TEXT);
  drawText(bx, by, bw, bh, "RETRY", COL_BG, F46, F34, 1, ALIGN_CENTER);

  g->setFont(FONT_LABEL);
  String hint = "TAP ANYWHERE OR PRESS BOOT";
  int16_t xOff;
  int tw = textWidth(hint, &xOff);
  g->setTextColor(COL_DIM);
  g->setCursor((SCR_W - tw) / 2 - xOff, 285);
  g->print(hint);

  pushRows(0, SCR_H);
}

static void giveUp() {
  Serial.printf("Gave up after %d network tries — waiting for Retry\n", networkTries);
  gaveUp = true;
  wifiUp = false;
  haveAblesetIp = false;
  connectedSsid = "";
  if (mdnsUp) { MDNS.end(); mdnsUp = false; }
  WiFi.disconnect(true);       // radio off until the user asks again
  wasPressed = true;           // ignore a finger that's already on the glass
  drawRetryScreen();
}

static void splash(const String &line1, const String &line2, uint16_t col) {
  g->fillScreen(COL_BG);
  drawText(20, 70, SCR_W - 40, 110, upperUtf8(line1), col, F50, F22, 2, ALIGN_CENTER);
  drawText(20, 190, SCR_W - 40, 60, upperUtf8(line2), COL_DIM, F28, F16, 1, ALIGN_CENTER);
  pushRows(0, SCR_H);
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

// Scan once, then try the configured networks in list order.
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

// Called when the user taps Retry: start a fresh budget from the first network.
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
    clearScreen();
    forceRedraw = true;
  }
}

// ============================================================================
// NETWORK RECEIVE TASK
// ============================================================================

static void oscRxTask(void *) {
  static uint8_t buf[8192];   // /setlist/songs, songDescriptions and bundles can be big
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
  bool listChanged = (netSongsVersion != songsVersion);
  if (listChanged) {
    for (int i = 0; i < netNSongs; i++) songs[i] = netSongs[i];
    for (int i = 0; i < MAX_SONGS; i++) songDescs[i] = (i < netNSongDescs) ? netSongDescs[i] : String("");
    nSongs = netNSongs;
    songsVersion = netSongsVersion;
  }
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
  Serial.println("\n\n=== AbleSet display (3.5\") booting ===");
  stateLock = xSemaphoreCreateMutex();
  WiFi.onEvent(onWifiEvent);

  // Panel power, resets and backlight all live on the IO expander, so it has
  // to come up before the display.
  if (!exioInit()) {
    Serial.println("ERROR: IO expander not found at 0x24 (I2C on GPIO27/26)");
  }
  pinMode(RETRY_BTN_PIN, INPUT_PULLUP);
  setBacklight(0);
  lcdReset();

  if (!panel->begin(LCD_SPI_HZ, SPI_MODE0)) {
    Serial.println("ERROR: display init failed");
  }
  panel->fillScreen(COL_BG);

  // Full-screen framebuffer (480 x 320 x 2 = 300 KB) — needs PSRAM enabled.
  if (canvas->begin(GFX_SKIP_OUTPUT_BEGIN)) {
    g = canvas;
    useCanvas = true;
    Serial.println("Framebuffer OK (flicker-free mode)");
  } else {
    Serial.println("No framebuffer (is PSRAM enabled?) — drawing directly");
  }
  g->setUTF8Print(true);
  g->setTextWrap(false);

  splash("ABLESET", "starting up", COL_TEXT);
  setBacklight(BACKLIGHT_PCT);
  touchOk = touchInit();

  if (connectWifi()) {
    discoverAbleset();
    sendSubscribe();
    lastSubscribe = millis();
  }
  lastWifiRetry = millis();

  if (!gaveUp) {
    clearScreen();
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
      clearScreen();
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
      net.nextSectCol = 0x2104; net.setLeft = -1;
      net.beatPos = -1; net.sectStart = 0; net.sectEnd = 0;
      net.onBattery = -1;
      xSemaphoreGive(stateLock);
      drawnNext = ""; drawnSide = ""; drawnSideHead = ""; drawnProgress = "";
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
    clearScreen();
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
