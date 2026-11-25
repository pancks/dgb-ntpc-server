// ESP32-S3 N16R8V + W5500 + GNSS1 (ME31GR01) + GNSS2 (NEO-M8N) + DS3231
// Time priority: GNSS1 -> GNSS2 -> RTC. Status page on http://<assigned IP>/
//
// platformio.ini lib_deps:
//   mikalhart/TinyGPSPlus@^1.0.3
//   (RTClib + Adafruit_BusIO: already in lib/)

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ETH.h>
#include <WiFi.h>
#include <Preferences.h>
#include "mbedtls/base64.h"
#include <TinyGPSPlus.h>
#include <RTClib.h>
#include "esp_timer.h"

// ===================== PINS =====================
// W5500
#define W5500_MOSI_PIN   12
#define W5500_MISO_PIN   13
#define W5500_SCLK_PIN   14
#define W5500_RST_PIN    21
#define W5500_INT_PIN    47
#define W5500_CS_PIN     48

// GNSS1: per the KiCad schematic, net G_TX = module TXD -> GPIO6, net G_RX = module RXD -> GPIO7.
// So the ESP32 UART RX is GPIO6 and the ESP32 UART TX is GPIO7 (the net names are module-side).
#define GNSS1_RX_PIN    6
#define GNSS1_TX_PIN    7
#define GNSS1_PPS_PIN   8
#define GNSS1_RST_PIN   9

// GNSS2 on header J3: net G2_TX (module TX) -> GPIO40, net G2_RX (module RX) -> GPIO41.
// ESP32 UART RX = GPIO40, ESP32 UART TX = GPIO41.
#define GNSS2_RX_PIN   40
#define GNSS2_TX_PIN   41
#define GNSS2_PPS_PIN  42

// DS3231
#define RTC_SDA_PIN    10
#define RTC_SCL_PIN    11

// ===================== CONFIG =====================
static const uint32_t BAUDS[] = {9600, 115200, 38400, 57600, 4800};   // scanned in this order
static const uint8_t  N_BAUDS = sizeof(BAUDS) / sizeof(BAUDS[0]);
// Network, time-source and security options live in flash now: edit them at http://<ip>/settings
static const uint32_t GNSS1_BAUD = 115200;       // fixed baud for GNSS1 (ME31GR01)
static const uint32_t GNSS2_BAUD = 9600;         // fixed baud for GNSS2 (NEO-M8N)
static const bool     GNSS_AUTOBAUD = false;     // true = scan BAUDS[] if no valid NMEA arrives
static const uint32_t GNSS_HOLD_MS = 3000;       // no fresh fix-time for this long -> source lost
static const bool     GNSS1_RESET_ON_BOOT = true; // assumes ACTIVE-LOW reset, see note below

// ===================== SETTINGS (stored in flash, edited at /settings) =====================
// First boot uses these defaults. Everything below can be changed from the web page without reflashing.
struct Settings {
  char     hostname[33];
  bool     ethEnabled, ethDhcp;
  uint32_t ethIp, ethMask, ethGw, ethDns;      // IPv4 stored as (uint32_t)IPAddress, 0 = unset
  bool     wifiEnabled, wifiDhcp;
  char     wifiSsid[33], wifiPass[65];
  uint32_t wifiIp, wifiMask, wifiGw, wifiDns;
  bool     srcEnabled[3];                      // 0 = GNSS1, 1 = GNSS2, 2 = RTC
  uint8_t  srcPrio[3];                         // 1 = tried first
  char     webPass[33];                        // empty = web UI is open (user name is always "admin")
  char     allow[129];                         // allowed NTP clients, e.g. "192.168.1.0/24, 10.0.0.5"; empty = everyone
};
Settings cfg;       // active settings
Settings draft;     // form values being validated
Preferences prefs;

uint8_t  srcOrder[3] = {0, 1, 2};              // source ids, best first (derived from cfg.srcPrio)
uint32_t allowNet[8], allowMask[8];            // parsed allow list, host byte order
uint8_t  allowCount = 0;

void settingsDefaults() {
  memset(&cfg, 0, sizeof cfg);
  strlcpy(cfg.hostname, "zaimu-ntps", sizeof cfg.hostname);
  cfg.ethEnabled = true;  cfg.ethDhcp = true;
  cfg.wifiEnabled = true; cfg.wifiDhcp = true;
  strlcpy(cfg.wifiSsid, "Digiboy", sizeof cfg.wifiSsid);
  strlcpy(cfg.wifiPass, "DGBsemangat", sizeof cfg.wifiPass);
  for (int i = 0; i < 3; i++) { cfg.srcEnabled[i] = true; cfg.srcPrio[i] = i + 1; }
}

// Parses "1.2.3.4, 10.0.0.0/8 ..." into net/mask pairs. Returns false on any malformed entry.
bool parseAllowList(const char *text, uint32_t *net, uint32_t *mask, uint8_t &count) {
  count = 0;
  char buf[130];
  strlcpy(buf, text, sizeof buf);
  char *save = nullptr;
  for (char *tok = strtok_r(buf, ", ;\t\r\n", &save); tok; tok = strtok_r(nullptr, ", ;\t\r\n", &save)) {
    if (count >= 8) return false;
    int prefix = 32;
    char *slash = strchr(tok, '/');
    if (slash) {
      *slash = 0;
      char *end = nullptr;
      long p = strtol(slash + 1, &end, 10);
      if (end == slash + 1 || *end != 0 || p < 0 || p > 32) return false;
      prefix = (int)p;
    }
    IPAddress ip;
    if (!ip.fromString(tok)) return false;
    uint32_t m = (prefix == 0) ? 0 : (0xFFFFFFFFu << (32 - prefix));
    net[count] = __builtin_bswap32((uint32_t)ip) & m;
    mask[count] = m;
    count++;
  }
  return true;
}

bool ntpAllowed(uint32_t ipMem) {
  if (allowCount == 0) return true;
  uint32_t h = __builtin_bswap32(ipMem);
  for (uint8_t i = 0; i < allowCount; i++) {
    if ((h & allowMask[i]) == allowNet[i]) return true;
  }
  return false;
}

void settingsApplyDerived() {
  uint8_t ids[3] = {0, 1, 2};
  for (int a = 0; a < 3; a++) {
    for (int b = a + 1; b < 3; b++) {
      if (cfg.srcPrio[ids[b]] < cfg.srcPrio[ids[a]]) { uint8_t t = ids[a]; ids[a] = ids[b]; ids[b] = t; }
    }
  }
  memcpy(srcOrder, ids, 3);
  if (!parseAllowList(cfg.allow, allowNet, allowMask, allowCount)) allowCount = 0;
}

void settingsLoad() {
  settingsDefaults();
  if (!prefs.begin("ntps", false)) return;
  String s;
  s = prefs.getString("host", cfg.hostname);   strlcpy(cfg.hostname, s.c_str(), sizeof cfg.hostname);
  cfg.ethEnabled = prefs.getBool("eEn", cfg.ethEnabled);
  cfg.ethDhcp    = prefs.getBool("eDhcp", cfg.ethDhcp);
  cfg.ethIp      = prefs.getUInt("eIp", 0);
  cfg.ethMask    = prefs.getUInt("eMask", 0);
  cfg.ethGw      = prefs.getUInt("eGw", 0);
  cfg.ethDns     = prefs.getUInt("eDns", 0);
  cfg.wifiEnabled = prefs.getBool("wEn", cfg.wifiEnabled);
  cfg.wifiDhcp    = prefs.getBool("wDhcp", cfg.wifiDhcp);
  s = prefs.getString("wSsid", cfg.wifiSsid);  strlcpy(cfg.wifiSsid, s.c_str(), sizeof cfg.wifiSsid);
  s = prefs.getString("wPass", cfg.wifiPass);  strlcpy(cfg.wifiPass, s.c_str(), sizeof cfg.wifiPass);
  cfg.wifiIp   = prefs.getUInt("wIp", 0);
  cfg.wifiMask = prefs.getUInt("wMask", 0);
  cfg.wifiGw   = prefs.getUInt("wGw", 0);
  cfg.wifiDns  = prefs.getUInt("wDns", 0);
  uint8_t en = prefs.getUChar("sEn", 7);
  uint32_t pk = prefs.getUInt("sPrio", 0x030201);
  for (int i = 0; i < 3; i++) {
    cfg.srcEnabled[i] = (en >> i) & 1;
    uint8_t p = (pk >> (8 * i)) & 0xFF;
    cfg.srcPrio[i] = (p >= 1 && p <= 3) ? p : (i + 1);
  }
  s = prefs.getString("webPass", "");          strlcpy(cfg.webPass, s.c_str(), sizeof cfg.webPass);
  s = prefs.getString("allow", "");            strlcpy(cfg.allow, s.c_str(), sizeof cfg.allow);
  prefs.end();
}

void settingsSave() {
  prefs.begin("ntps", false);
  prefs.putString("host", cfg.hostname);
  prefs.putBool("eEn", cfg.ethEnabled);   prefs.putBool("eDhcp", cfg.ethDhcp);
  prefs.putUInt("eIp", cfg.ethIp);        prefs.putUInt("eMask", cfg.ethMask);
  prefs.putUInt("eGw", cfg.ethGw);        prefs.putUInt("eDns", cfg.ethDns);
  prefs.putBool("wEn", cfg.wifiEnabled);  prefs.putBool("wDhcp", cfg.wifiDhcp);
  prefs.putString("wSsid", cfg.wifiSsid); prefs.putString("wPass", cfg.wifiPass);
  prefs.putUInt("wIp", cfg.wifiIp);       prefs.putUInt("wMask", cfg.wifiMask);
  prefs.putUInt("wGw", cfg.wifiGw);       prefs.putUInt("wDns", cfg.wifiDns);
  uint8_t en = 0;
  for (int i = 0; i < 3; i++) if (cfg.srcEnabled[i]) en |= (1 << i);
  prefs.putUChar("sEn", en);
  prefs.putUInt("sPrio", (uint32_t)cfg.srcPrio[0] | ((uint32_t)cfg.srcPrio[1] << 8) | ((uint32_t)cfg.srcPrio[2] << 16));
  prefs.putString("webPass", cfg.webPass);
  prefs.putString("allow", cfg.allow);
  prefs.end();
}

void settingsErase() {
  prefs.begin("ntps", false);
  prefs.clear();
  prefs.end();
}

// ===================== PPS (interrupts) =====================
portMUX_TYPE ppsMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timeMux = portMUX_INITIALIZER_UNLOCKED;   // guards the time anchors shared with the NTP task
volatile uint32_t ppsCount[2] = {0, 0};
volatile int64_t  ppsUs[2]    = {0, 0};

void IRAM_ATTR ppsIsr0() {
  portENTER_CRITICAL_ISR(&ppsMux);
  ppsUs[0] = esp_timer_get_time();
  ppsCount[0] = ppsCount[0] + 1;
  portEXIT_CRITICAL_ISR(&ppsMux);
}
void IRAM_ATTR ppsIsr1() {
  portENTER_CRITICAL_ISR(&ppsMux);
  ppsUs[1] = esp_timer_get_time();
  ppsCount[1] = ppsCount[1] + 1;
  portEXIT_CRITICAL_ISR(&ppsMux);
}
void readPps(int i, uint32_t &count, int64_t &us) {
  portENTER_CRITICAL(&ppsMux);
  count = ppsCount[i];
  us = ppsUs[i];
  portEXIT_CRITICAL(&ppsMux);
}

// ===================== GNSS =====================
struct Gnss {
  const char *name = "";
  HardwareSerial *port = nullptr;
  int rx = -1, tx = -1, pps = -1;
  TinyGPSPlus gps;

  uint32_t baud = 9600;          // current UART baud
  uint8_t  baudIdx = 0;          // position in BAUDS[] while scanning
  bool     locked = false;       // valid NMEA seen at current baud
  uint32_t baudSince = 0;        // millis() when current baud was selected
  uint32_t lastGoodMs = 0;       // millis() of last valid sentence
  uint32_t lastPassed = 0;

  bool     anchorValid = false;  // have a time anchor from a fixed RMC
  uint32_t anchorEpoch = 0;      // UTC unix seconds at anchorUs
  int64_t  anchorUs = 0;         // esp_timer time of that second's start (PPS) or sentence arrival
  uint32_t anchorWallMs = 0;     // millis() when anchor was last refreshed
  bool     ppsAligned = false;
};
Gnss gn[2];

void gnssBegin(int i, const char *name, HardwareSerial *port, int rx, int tx, int pps, uint32_t baud, void (*isr)()) {
  Gnss &g = gn[i];
  g.name = name; g.port = port; g.rx = rx; g.tx = tx; g.pps = pps; g.baud = baud;
  port->setRxBufferSize(2048);
  port->begin(baud, SERIAL_8N1, rx, tx);
  pinMode(pps, INPUT);
  attachInterrupt(digitalPinToInterrupt(pps), isr, RISING);
}

bool gnssUsable(int i) {
  return gn[i].anchorValid && (millis() - gn[i].anchorWallMs) < GNSS_HOLD_MS;
}

// Called when a complete, checksum-valid sentence was just decoded.
void gnssOnSentence(int i) {
  Gnss &g = gn[i];
  // Only trust time from an RMC that carries a valid fix (date, time and location all just committed).
  if (g.gps.location.isValid() && g.gps.location.age() < 50 &&
      g.gps.date.isValid() && g.gps.date.age() < 50 &&
      g.gps.time.isValid() && g.gps.time.age() < 50) {
    int year = g.gps.date.year();
    if (year >= 2024) {
      DateTime dt(year, g.gps.date.month(), g.gps.date.day(),
                  g.gps.time.hour(), g.gps.time.minute(), g.gps.time.second());
      int64_t now = esp_timer_get_time();
      uint32_t cnt; int64_t pus;
      readPps(i, cnt, pus);
      bool aligned = cnt > 0 && (now - pus) >= 0 && (now - pus) < 900000;  // PPS edge just before this sentence
      uint32_t ep = dt.unixtime();
      uint32_t wall = millis();
      portENTER_CRITICAL(&timeMux);
      g.anchorEpoch = ep;
      g.anchorUs = aligned ? pus : now;
      g.ppsAligned = aligned;
      g.anchorValid = true;
      g.anchorWallMs = wall;
      portEXIT_CRITICAL(&timeMux);
    }
  }
}

void gnssAutoBaud(Gnss &g) {
  uint32_t now = millis();
  uint32_t passed = g.gps.passedChecksum();
  if (passed != g.lastPassed) {
    g.lastPassed = passed;
    g.lastGoodMs = now;
    g.locked = true;
    return;
  }
  if (!GNSS_AUTOBAUD) return;
  uint32_t ref = g.lastGoodMs > g.baudSince ? g.lastGoodMs : g.baudSince;
  uint32_t limit = g.locked ? 30000UL : 4000UL;
  if (now - ref > limit) {
    g.locked = false;
    g.baudIdx = (g.baudIdx + 1) % N_BAUDS;
    g.baud = BAUDS[g.baudIdx];
    g.port->updateBaudRate(g.baud);
    g.baudSince = now;
  }
}

void gnssPoll(int i) {
  Gnss &g = gn[i];
  while (g.port->available()) {
    if (g.gps.encode((char)g.port->read())) gnssOnSentence(i);
  }
  gnssAutoBaud(g);
}

// 0 = no bytes, 1 = bytes but no valid NMEA, 2 = NMEA but no fix, 3 = fix but time not usable, 4 = OK
int gnssLevel(int i) {
  Gnss &g = gn[i];
  if (g.gps.charsProcessed() == 0) return 0;
  if (g.gps.passedChecksum() == 0) return 1;
  bool fix = g.gps.location.isValid() && g.gps.location.age() < 3000;
  if (!fix) return 2;
  if (!gnssUsable(i)) return 3;
  return 4;
}
const char *gnssLevelText(int i, const char **cls) {
  switch (gnssLevel(i)) {
    case 0:  *cls = "bad";  return "NO DATA - check wiring / TX-RX swap";
    case 1:  *cls = "bad";  return "BYTES BUT NO VALID NMEA - wrong baud or format";
    case 2:  *cls = "warn"; return "NMEA OK - NO FIX";
    case 3:  *cls = "warn"; return "FIX - TIME NOT YET VALID";
    default: *cls = "ok";   return "FIX - TIME OK";
  }
}

// ===================== RTC =====================
RTC_DS3231 rtc;
bool     rtcInitOk = false;
bool     rtcPresent = false;
bool     rtcLost = true;
bool     rtcValid = false;
uint32_t rtcEpoch = 0;
int64_t  rtcReadUs = 0;
float    rtcTemp = NAN;
uint32_t lastRtcSyncMs = 0;
uint32_t rtcSyncCount = 0;

void pollRtc() {
  Wire.beginTransmission(0x68);
  rtcPresent = (Wire.endTransmission() == 0);
  if (!rtcPresent) { rtcValid = false; return; }
  if (!rtcInitOk) rtcInitOk = rtc.begin(&Wire);
  bool lost = rtc.lostPower();
  DateTime n = rtc.now();
  uint32_t ep = n.unixtime();
  int64_t rus = esp_timer_get_time();
  float temp = rtc.getTemperature();
  bool valid = !lost && n.year() >= 2024;
  portENTER_CRITICAL(&timeMux);
  rtcLost = lost;
  rtcEpoch = ep;
  rtcReadUs = rus;
  rtcValid = valid;
  portEXIT_CRITICAL(&timeMux);
  rtcTemp = temp;
}
uint32_t rtcNowEpoch() {
  return rtcEpoch + (uint32_t)((esp_timer_get_time() - rtcReadUs) / 1000000);
}

// ===================== TIME SELECTION =====================
enum Src { SRC_NONE, SRC_GNSS1, SRC_GNSS2, SRC_RTC };
const char *SRC_NAME[] = {"NONE", "GNSS1", "GNSS2", "RTC"};

Src selectSource() {
  for (int k = 0; k < 3; k++) {
    int id = srcOrder[k];
    if (!cfg.srcEnabled[id]) continue;
    if (id < 2) { if (gnssUsable(id)) return id == 0 ? SRC_GNSS1 : SRC_GNSS2; }
    else if (rtcPresent && rtcValid) return SRC_RTC;
  }
  return SRC_NONE;
}

bool getTime(Src s, uint32_t &epoch, uint32_t &ms) {
  int64_t now = esp_timer_get_time();
  if (s == SRC_GNSS1 || s == SRC_GNSS2) {
    Gnss &g = gn[s == SRC_GNSS1 ? 0 : 1];
    int64_t el = now - g.anchorUs;
    epoch = g.anchorEpoch + (uint32_t)(el / 1000000);
    ms = (uint32_t)((el % 1000000) / 1000);
    return true;
  }
  if (s == SRC_RTC) {
    epoch = rtcNowEpoch();
    ms = 0;
    return true;
  }
  return false;
}

void fmtTime(uint32_t epoch, char *out, size_t n) {
  DateTime d(epoch);
  snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d UTC",
           (int)d.year(), (int)d.month(), (int)d.day(), (int)d.hour(), (int)d.minute(), (int)d.second());
}

// Keep the RTC disciplined by GNSS so the fallback is actually good.
void maybeSyncRtc(Src s) {
  if ((s != SRC_GNSS1 && s != SRC_GNSS2) || !rtcPresent) return;
  uint32_t ep, ms;
  getTime(s, ep, ms);
  if (ms > 100) return;                                   // write right after a second boundary
  int32_t drift = (int32_t)(rtcNowEpoch() - ep);
  if (rtcValid && abs(drift) <= 1) return;                // RTC read-out is 1 s granular
  uint32_t interval = rtcValid ? 600000UL : 10000UL;
  if (lastRtcSyncMs != 0 && millis() - lastRtcSyncMs < interval) return;
  rtc.adjust(DateTime(ep));
  lastRtcSyncMs = millis();
  rtcSyncCount++;
  pollRtc();
}

// ===================== ETHERNET + WEB =====================
NetworkServer server(80);
volatile bool ethGotIp = false;
volatile bool wifiGotIp = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:        Serial.println("ETH started"); break;
    case ARDUINO_EVENT_ETH_CONNECTED:    Serial.println("ETH link up"); break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH IP: "); Serial.println(ETH.localIP());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH link down");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi disconnected");
      break;
    default: break;
  }
}

void row(String &h, const char *k, const char *v, const char *cls = "");
void row(String &h, const char *k, const char *v, const char *cls) {
  h += F("<tr><td>"); h += k; h += F("</td><td class='"); h += cls; h += F("'>"); h += v; h += F("</td></tr>");
}

// ===================== NTP SERVER (UDP 123) =====================
// Runs in its own task so web rendering in loop() cannot delay the timestamps.
// One socket per interface, each bound to that interface's IP, so replies always
// carry the address the client asked.
static const uint32_t NTP_UNIX_OFFSET = 2208988800UL;   // seconds from 1900 to 1970

volatile uint32_t ntpReqEth = 0, ntpReqWifi = 0, ntpIgnored = 0, ntpDenied = 0, ntpLastClient = 0, ntpLastReqMs = 0;
WiFiUDP udpEth, udpWifi;
volatile uint32_t udpEthIp = 0, udpWifiIp = 0;

// Atomic snapshot of the best available UTC time. Returns false if no source is valid.
bool timeSnapshot(Src &src, uint32_t &epoch, uint32_t &us, bool &aligned) {
  portENTER_CRITICAL(&timeMux);
  int64_t nowUs = esp_timer_get_time();
  uint32_t nowMs = (uint32_t)(nowUs / 1000);
  src = SRC_NONE; epoch = 0; us = 0; aligned = false;
  for (int k = 0; k < 3 && src == SRC_NONE; k++) {
    int id = srcOrder[k];
    if (!cfg.srcEnabled[id]) continue;
    if (id < 2) {
      if (gn[id].anchorValid && (uint32_t)(nowMs - gn[id].anchorWallMs) < GNSS_HOLD_MS) {
        int64_t el = nowUs - gn[id].anchorUs;
        epoch = gn[id].anchorEpoch + (uint32_t)(el / 1000000);
        us = (uint32_t)(el % 1000000);
        aligned = gn[id].ppsAligned;
        src = (id == 0) ? SRC_GNSS1 : SRC_GNSS2;
      }
    } else if (rtcPresent && rtcValid) {
      int64_t el = nowUs - rtcReadUs;
      epoch = rtcEpoch + (uint32_t)(el / 1000000);
      us = (uint32_t)(el % 1000000);          // DS3231 read-out is whole seconds, sub-second phase is arbitrary
      src = SRC_RTC;
    }
  }
  portEXIT_CRITICAL(&timeMux);
  return src != SRC_NONE;
}

static inline void put32(uint8_t *p, uint32_t v) {
  p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}
static inline uint32_t ntpFrac(uint32_t us) {
  return (uint32_t)(((uint64_t)us << 32) / 1000000ULL);
}

void ntpServe(WiFiUDP &u, bool isEth) {
  int sz = u.parsePacket();
  if (sz <= 0) return;
  Src s; uint32_t rxSec, rxUs; bool al;
  bool ok = timeSnapshot(s, rxSec, rxUs, al);                 // receive timestamp, taken as early as possible
  uint8_t req[48];
  if (sz < 48 || u.read(req, 48) < 48 || (req[0] & 7) != 3) { // only plain client requests (mode 3)
    ntpIgnored = ntpIgnored + 1;
    return;
  }
  IPAddress cip = u.remoteIP();
  uint16_t cport = u.remotePort();
  if (!ntpAllowed((uint32_t)cip)) { ntpDenied = ntpDenied + 1; return; }   // not on the allow list: stay silent

  uint8_t vn = (req[0] >> 3) & 7;
  if (vn < 1 || vn > 4) vn = 4;
  uint8_t li = 0, stratum = 16;
  int8_t prec = -6;
  uint32_t rootDisp = 0;
  uint8_t refid[4] = {0, 0, 0, 0};
  if (!ok) {
    li = 3;                                                   // alarm: clock not synchronised
  } else if (s == SRC_RTC) {
    stratum = 4; prec = 0; rootDisp = 131072;                 // ~2 s (16.16 fixed point seconds)
  } else {
    stratum = 1; refid[0] = 'G'; refid[1] = 'P'; refid[2] = 'S';
    prec = al ? -20 : -10;
    rootDisp = al ? 66 : 32768;                               // ~1 ms with PPS, ~0.5 s without
  }

  uint8_t resp[48];
  memset(resp, 0, sizeof resp);
  resp[0] = (li << 6) | (vn << 3) | 4;                        // server mode
  resp[1] = stratum;
  resp[2] = req[2];                                           // echo poll
  resp[3] = (uint8_t)prec;
  put32(resp + 8, rootDisp);
  memcpy(resp + 12, refid, 4);
  memcpy(resp + 24, req + 40, 8);                             // origin = client's transmit timestamp
  if (ok) {
    put32(resp + 16, rxSec + NTP_UNIX_OFFSET);                // reference timestamp
    put32(resp + 32, rxSec + NTP_UNIX_OFFSET);                // receive timestamp
    put32(resp + 36, ntpFrac(rxUs));
    Src s2; uint32_t txSec, txUs; bool al2;
    if (timeSnapshot(s2, txSec, txUs, al2)) {                 // transmit timestamp, right before sending
      put32(resp + 40, txSec + NTP_UNIX_OFFSET);
      put32(resp + 44, ntpFrac(txUs));
    }
  }
  u.beginPacket(cip, cport);
  u.write(resp, 48);
  u.endPacket();

  if (isEth) ntpReqEth = ntpReqEth + 1; else ntpReqWifi = ntpReqWifi + 1;
  ntpLastClient = (uint32_t)cip;
  ntpLastReqMs = millis();
}

// (Re)binds each socket whenever its interface gets, loses or changes its IP.
void ntpManageSockets() {
  static uint32_t nextTry = 0;
  if ((int32_t)(millis() - nextTry) < 0) return;
  uint32_t e = ethGotIp ? (uint32_t)ETH.localIP() : 0;
  if (e != udpEthIp) {
    if (udpEthIp) udpEth.stop();
    udpEthIp = e;
    if (e && !udpEth.begin(IPAddress(e), 123)) { udpEthIp = 0; nextTry = millis() + 2000; }
  }
  uint32_t w = wifiGotIp ? (uint32_t)WiFi.localIP() : 0;
  if (w != udpWifiIp) {
    if (udpWifiIp) udpWifi.stop();
    udpWifiIp = w;
    if (w && !udpWifi.begin(IPAddress(w), 123)) { udpWifiIp = 0; nextTry = millis() + 2000; }
  }
}

void ntpTask(void *) {
  for (;;) {
    ntpManageSockets();
    if (udpEthIp) ntpServe(udpEth, true);
    if (udpWifiIp) ntpServe(udpWifi, false);
    vTaskDelay(1);
  }
}

void addNtp(String &h) {
  char b[112];
  h += F("<h2>NTP server (UDP 123)</h2><table>");
  Src s; uint32_t sec, us; bool al;
  if (!timeSnapshot(s, sec, us, al)) {
    row(h, "Serving", "NOT SYNCED - replies flagged unsynchronised", "bad");
  } else if (s == SRC_RTC) {
    row(h, "Serving", "stratum 4 from RTC (accuracy about +-1 s)", "warn");
  } else {
    snprintf(b, sizeof b, "stratum 1 from %s, %s", SRC_NAME[s], al ? "PPS-aligned" : "no PPS alignment");
    row(h, "Serving", b, al ? "ok" : "warn");
  }
  String ei = udpEthIp ? IPAddress((uint32_t)udpEthIp).toString() : String("-");
  String wi = udpWifiIp ? IPAddress((uint32_t)udpWifiIp).toString() : String("-");
  snprintf(b, sizeof b, "ETH %s, WiFi %s", ei.c_str(), wi.c_str());
  row(h, "Listening on", b, (udpEthIp || udpWifiIp) ? "ok" : "warn");
  snprintf(b, sizeof b, "%lu via ETH, %lu via WiFi, %lu ignored, %lu denied",
           (unsigned long)ntpReqEth, (unsigned long)ntpReqWifi, (unsigned long)ntpIgnored, (unsigned long)ntpDenied);
  row(h, "Requests", b);
  if (ntpLastClient) {
    String c = IPAddress((uint32_t)ntpLastClient).toString();
    snprintf(b, sizeof b, "%s, %lu s ago", c.c_str(), (unsigned long)((millis() - ntpLastReqMs) / 1000));
    row(h, "Last client", b);
  }
  h += F("</table>");
}

void addGnss(String &h, int i) {
  Gnss &g = gn[i];
  char b[128];
  const char *cls;
  const char *st = gnssLevelText(i, &cls);
  h += F("<h2>"); h += g.name; h += F("</h2><table>");
  row(h, "Status", st, cls);

  snprintf(b, sizeof b, "%lu baud (%s)", (unsigned long)g.baud, g.locked ? "locked" : (GNSS_AUTOBAUD ? "scanning" : "no valid NMEA"));
  row(h, "UART", b, g.locked ? "ok" : "warn");

  uint32_t passed = g.gps.passedChecksum();
  snprintf(b, sizeof b, "%lu bytes, %lu NMEA ok, %lu bad checksum",
           (unsigned long)g.gps.charsProcessed(), (unsigned long)passed, (unsigned long)g.gps.failedChecksum());
  row(h, "Data", b, passed ? "ok" : "bad");

  if (g.gps.satellites.isValid()) snprintf(b, sizeof b, "%lu", (unsigned long)g.gps.satellites.value());
  else snprintf(b, sizeof b, "n/a");
  row(h, "Satellites", b);

  if (g.gps.hdop.isValid()) snprintf(b, sizeof b, "%.2f", g.gps.hdop.hdop());
  else snprintf(b, sizeof b, "n/a");
  row(h, "HDOP", b);

  bool fix = g.gps.location.isValid() && g.gps.location.age() < 3000;
  if (fix) {
    snprintf(b, sizeof b, "%.6f, %.6f, alt %.1f m",
             g.gps.location.lat(), g.gps.location.lng(), g.gps.altitude.meters());
    row(h, "Position", b, "ok");
  } else {
    row(h, "Position", "no fix", "warn");
  }

  uint32_t cnt; int64_t us;
  readPps(i, cnt, us);
  if (cnt == 0) {
    row(h, "PPS", "no pulses yet (often starts only after fix)", "warn");
  } else {
    long age = (long)((esp_timer_get_time() - us) / 1000);
    snprintf(b, sizeof b, "%s - %lu pulses, last %ld ms ago", age < 2000 ? "ACTIVE" : "STALE", (unsigned long)cnt, age);
    row(h, "PPS", b, age < 2000 ? "ok" : "bad");
  }

  if (g.anchorValid) {
    snprintf(b, sizeof b, "refreshed %lu ms ago, %s", (unsigned long)(millis() - g.anchorWallMs),
             g.ppsAligned ? "PPS-aligned" : "NMEA only (no PPS alignment)");
    row(h, "Time anchor", b, gnssUsable(i) ? "ok" : "warn");
  } else {
    row(h, "Time anchor", "none yet", "warn");
  }
  h += F("</table>");
}

void addRtc(String &h, Src sel) {
  char b[96];
  h += F("<h2>RTC (DS3231)</h2><table>");
  if (!rtcPresent) {
    row(h, "Device", "NOT FOUND - check SDA/SCL", "bad");
    h += F("</table>");
    return;
  }
  row(h, "Device", "present", "ok");
  row(h, "Oscillator", rtcLost ? "LOST POWER - time invalid until set" : "OK", rtcLost ? "bad" : "ok");
  char t[32];
  fmtTime(rtcNowEpoch(), t, sizeof t);
  row(h, "RTC time", t, rtcValid ? "ok" : "warn");
  snprintf(b, sizeof b, "%.2f C", rtcTemp);
  row(h, "Temperature", b);
  if (rtcValid && (sel == SRC_GNSS1 || sel == SRC_GNSS2)) {
    uint32_t ep, ms;
    getTime(sel, ep, ms);
    snprintf(b, sizeof b, "%+ld s vs %s", (long)(int32_t)(rtcNowEpoch() - ep), SRC_NAME[sel]);
    row(h, "Drift", b);
  }
  snprintf(b, sizeof b, "%lu syncs from GNSS%s", (unsigned long)rtcSyncCount,
           lastRtcSyncMs ? "" : " (none yet)");
  row(h, "Discipline", b);
  h += F("</table>");
}

void addEth(String &h) {
  char b[96];
  h += F("<h2>Ethernet (W5500)</h2><table>");
  if (!cfg.ethEnabled) { row(h, "Link", "DISABLED in settings", "warn"); h += F("</table>"); return; }
  row(h, "Link", ETH.linkUp() ? "UP" : "DOWN", ETH.linkUp() ? "ok" : "bad");
  row(h, "Addressing", cfg.ethDhcp ? "DHCP" : "static");
  String ip = ethGotIp ? ETH.localIP().toString() : String("no lease yet");
  row(h, "IP", ip.c_str(), ethGotIp ? "ok" : "warn");
  snprintf(b, sizeof b, "%u Mbps, %s duplex", (unsigned)ETH.linkSpeed(), ETH.fullDuplex() ? "full" : "half");
  row(h, "Speed", b);
  String mac = ETH.macAddress();
  row(h, "MAC", mac.c_str());
  snprintf(b, sizeof b, "%lu s", (unsigned long)(millis() / 1000));
  row(h, "Uptime", b);
  h += F("</table>");
}

void addWifi(String &h) {
  char b[96];
  h += F("<h2>WiFi</h2><table>");
  bool up = (WiFi.status() == WL_CONNECTED);
  row(h, "Link", up ? "CONNECTED" : (cfg.wifiEnabled ? "DISCONNECTED" : "DISABLED in settings"), up ? "ok" : (cfg.wifiEnabled ? "bad" : "warn"));
  row(h, "SSID", cfg.wifiSsid);
  row(h, "Addressing", cfg.wifiDhcp ? "DHCP" : "static");
  String ip = wifiGotIp ? WiFi.localIP().toString() : String("no lease yet");
  row(h, "IP", ip.c_str(), wifiGotIp ? "ok" : "warn");
  if (up) {
    int rssi = WiFi.RSSI();
    snprintf(b, sizeof b, "%d dBm", rssi);
    row(h, "Signal", b, rssi > -75 ? "ok" : "warn");
  }
  String mac = WiFi.macAddress();
  row(h, "MAC", mac.c_str());
  h += F("</table>");
}

String buildPage() {
  Src sel = selectSource();
  uint32_t ep = 0, ms = 0;
  bool have = getTime(sel, ep, ms);

  String h;
  h.reserve(6000);
  h += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta http-equiv='refresh' content='1'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'><title>Time status</title><style>"
         "body{font-family:sans-serif;max-width:720px;margin:1em auto;padding:0 1em}"
         "table{border-collapse:collapse;width:100%}td{border-bottom:1px solid #8884;padding:4px 6px}"
         "td:first-child{width:34%;opacity:.7}.ok{color:#1a9c3c}.warn{color:#d98a00}.bad{color:#d62828}"
         ".clock{font-size:2em;font-weight:600}h2{margin:1em 0 .3em}</style></head><body><div><a href='/settings'>Settings</a></div>");
  if (have) {
    char t[40];
    fmtTime(ep, t, sizeof t);
    h += F("<div class='clock'>"); h += t; h += F("</div><div>Source: <b>"); h += SRC_NAME[sel]; h += F("</b></div>");
  } else {
    h += F("<div class='clock bad'>NO VALID TIME SOURCE</div>");
  }
  addNtp(h);
  addGnss(h, 0);
  addGnss(h, 1);
  addRtc(h, sel);
  addEth(h);
  addWifi(h);
  h += F("</body></html>");
  return h;
}

// ===================== WEB: HTTP, PASSWORD, SETTINGS PAGE =====================
struct HttpReq {
  String method, path, auth, body;
  int contentLen = 0;
};

String base64Encode(const String &in) {
  unsigned char out[128];
  size_t olen = 0;
  if (mbedtls_base64_encode(out, sizeof out, &olen, (const unsigned char *)in.c_str(), in.length()) != 0) return String();
  String s;
  s.reserve(olen);
  for (size_t i = 0; i < olen; i++) s += (char)out[i];
  return s;
}

// HTTP Basic auth, user "admin". Credentials travel unencrypted, so this protects against casual access only.
bool authOk(const String &hdr) {
  if (!cfg.webPass[0]) return true;
  String expect = String("Basic ") + base64Encode(String("admin:") + cfg.webPass);
  if (hdr.length() != expect.length()) return false;
  uint8_t d = 0;
  for (unsigned i = 0; i < expect.length(); i++) d |= (uint8_t)(hdr[i] ^ expect[i]);
  return d == 0;
}

bool readHttpRequest(NetworkClient &c, HttpReq &r) {
  uint32_t t0 = millis();
  String line;
  bool first = true, headersDone = false;
  while (c.connected() || c.available()) {
    if (millis() - t0 > 3000) return false;
    if (!c.available()) { gnssPoll(0); gnssPoll(1); delay(1); continue; }
    char ch = (char)c.read();
    if (!headersDone) {
      if (ch == '\r') continue;
      if (ch != '\n') { if (line.length() < 300) line += ch; continue; }
      if (line.length() == 0) {
        headersDone = true;
        if (r.contentLen <= 0) return true;
        continue;
      }
      if (first) {
        first = false;
        int a = line.indexOf(' ');
        int b = line.indexOf(' ', a + 1);
        if (a < 0 || b < 0) return false;
        r.method = line.substring(0, a);
        r.path = line.substring(a + 1, b);
      } else {
        String lower = line;
        lower.toLowerCase();
        if (lower.startsWith("authorization:")) { r.auth = line.substring(14); r.auth.trim(); }
        else if (lower.startsWith("content-length:")) {
          r.contentLen = line.substring(15).toInt();
          if (r.contentLen > 3000) return false;
        }
      }
      line = "";
    } else {
      r.body += ch;
      if ((int)r.body.length() >= r.contentLen) return true;
    }
  }
  return headersDone && (int)r.body.length() >= r.contentLen;
}

String urlDecode(const String &s) {
  String o;
  o.reserve(s.length());
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '+') o += ' ';
    else if (c == '%' && i + 2 < s.length()) {
      char hx[3] = {s[i + 1], s[i + 2], 0};
      o += (char)strtol(hx, nullptr, 16);
      i += 2;
    } else o += c;
  }
  return o;
}

bool formFind(const String &b, const char *key, String *val = nullptr) {
  String k = String(key) + "=";
  unsigned pos = 0;
  while (pos < b.length()) {
    int amp = b.indexOf('&', pos);
    unsigned end = (amp < 0) ? b.length() : (unsigned)amp;
    if (b.startsWith(k, pos)) {
      if (val) *val = urlDecode(b.substring(pos + k.length(), end));
      return true;
    }
    pos = end + 1;
  }
  return false;
}
String formGet(const String &b, const char *key) {
  String v;
  formFind(b, key, &v);
  return v;
}

String htmlEsc(const char *s) {
  String o;
  for (; *s; s++) {
    switch (*s) {
      case '&':  o += F("&amp;"); break;
      case '<':  o += F("&lt;"); break;
      case '>':  o += F("&gt;"); break;
      case '"':  o += F("&quot;"); break;
      case '\'': o += F("&#39;"); break;
      default:   o += *s;
    }
  }
  return o;
}

String ipStr(uint32_t v) { return v ? IPAddress(v).toString() : String(""); }

// Reads an optional IPv4 field. Empty -> 0. Returns false (and sets err) if non-empty and invalid.
bool ipField(const String &b, const char *key, uint32_t &out, String &err, const char *label) {
  String v = formGet(b, key);
  v.trim();
  out = 0;
  if (v.length() == 0) return true;
  IPAddress ip;
  if (!ip.fromString(v.c_str())) { err = String(label) + " is not a valid IPv4 address."; return false; }
  out = (uint32_t)ip;
  return true;
}

// Validates the posted form into `draft`. Returns an error message, or "" after saving into cfg + flash.
String applySettingsForm(const String &b) {
  Settings &n = draft;
  n = cfg;
  String v, err;

  v = formGet(b, "host");
  v.trim();
  if (v.length() < 1 || v.length() > 32) return "Hostname must be 1-32 characters.";
  for (unsigned i = 0; i < v.length(); i++) {
    char c = v[i];
    if (!(isalnum((unsigned char)c) || c == '-')) return "Hostname may only contain letters, digits and '-'.";
  }
  strlcpy(n.hostname, v.c_str(), sizeof n.hostname);

  n.ethEnabled = formFind(b, "eEn");
  n.ethDhcp = (formGet(b, "eMode") != "static");
  if (!ipField(b, "eIp", n.ethIp, err, "Ethernet IP") || !ipField(b, "eMask", n.ethMask, err, "Ethernet mask") ||
      !ipField(b, "eGw", n.ethGw, err, "Ethernet gateway") || !ipField(b, "eDns", n.ethDns, err, "Ethernet DNS")) return err;
  if (n.ethEnabled && !n.ethDhcp && (!n.ethIp || !n.ethMask || !n.ethGw)) return "Static Ethernet needs IP, mask and gateway.";

  n.wifiEnabled = formFind(b, "wEn");
  n.wifiDhcp = (formGet(b, "wMode") != "static");
  v = formGet(b, "wSsid");
  v.trim();
  if (n.wifiEnabled && (v.length() < 1 || v.length() > 32)) return "WiFi SSID must be 1-32 characters.";
  if (v.length()) strlcpy(n.wifiSsid, v.c_str(), sizeof n.wifiSsid);
  v = formGet(b, "wPass");
  if (formFind(b, "wPassClr")) {
    n.wifiPass[0] = 0;
  } else if (v.length()) {
    if (v.length() < 8 || v.length() > 63) return "WiFi password must be 8-63 characters.";
    strlcpy(n.wifiPass, v.c_str(), sizeof n.wifiPass);
  }
  if (!ipField(b, "wIp", n.wifiIp, err, "WiFi IP") || !ipField(b, "wMask", n.wifiMask, err, "WiFi mask") ||
      !ipField(b, "wGw", n.wifiGw, err, "WiFi gateway") || !ipField(b, "wDns", n.wifiDns, err, "WiFi DNS")) return err;
  if (n.wifiEnabled && !n.wifiDhcp && (!n.wifiIp || !n.wifiMask || !n.wifiGw)) return "Static WiFi needs IP, mask and gateway.";

  if (!n.ethEnabled && !n.wifiEnabled) return "Enable at least one interface, or the device becomes unreachable.";

  for (int i = 0; i < 3; i++) {
    char k[8];
    snprintf(k, sizeof k, "sEn%d", i);
    n.srcEnabled[i] = formFind(b, k);
    snprintf(k, sizeof k, "sP%d", i);
    int p = formGet(b, k).toInt();
    n.srcPrio[i] = (p >= 1 && p <= 3) ? p : (i + 1);
  }
  if (!n.srcEnabled[0] && !n.srcEnabled[1] && !n.srcEnabled[2]) return "Enable at least one time source.";

  String p1 = formGet(b, "newPass"), p2 = formGet(b, "newPass2");
  if (formFind(b, "noPass")) {
    n.webPass[0] = 0;
  } else if (p1.length() || p2.length()) {
    if (p1 != p2) return "The two web passwords do not match.";
    if (p1.length() < 4 || p1.length() > 32) return "Web password must be 4-32 characters.";
    strlcpy(n.webPass, p1.c_str(), sizeof n.webPass);
  }

  v = formGet(b, "allow");
  v.trim();
  if (v.length() > 128) return "Allowed-clients list is too long (128 characters max).";
  uint32_t tn[8], tm[8];
  uint8_t tc;
  if (!parseAllowList(v.c_str(), tn, tm, tc)) return "Allowed clients: use IPs or CIDR like 192.168.1.0/24, comma separated, max 8 entries.";
  strlcpy(n.allow, v.c_str(), sizeof n.allow);

  cfg = n;
  settingsSave();
  return "";
}

static void optionTag(String &h, const char *value, const char *label, bool sel) {
  h += F("<option value='"); h += value; h += F("'");
  if (sel) h += F(" selected");
  h += F(">"); h += label; h += F("</option>");
}
static void textField(String &h, const char *label, const char *name, const String &value, const char *type = "text") {
  h += F("<label>"); h += label; h += F("<input type='"); h += type; h += F("' name='"); h += name;
  h += F("' value='"); h += htmlEsc(value.c_str()); h += F("'></label>");
}
static void checkField(String &h, const char *label, const char *name, bool on) {
  h += F("<label class='chk'><input type='checkbox' name='"); h += name; h += F("' value='1'");
  if (on) h += F(" checked");
  h += F("> "); h += label; h += F("</label>");
}
static void addressing(String &h, const char *selName, bool dhcp) {
  h += F("<label>Addressing<select name='"); h += selName; h += F("'>");
  optionTag(h, "dhcp", "DHCP (automatic)", dhcp);
  optionTag(h, "static", "Static IP", !dhcp);
  h += F("</select></label>");
}

String buildSettingsPage(const String &msg, bool isErr, const Settings &s) {
  String h;
  h.reserve(9000);
  h += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>Settings</title><style>"
         "body{font-family:sans-serif;max-width:720px;margin:1em auto;padding:0 1em}"
         "label{display:block;margin:.4em 0}"
         "label input[type=text],label input[type=password],label select{display:block;width:100%;box-sizing:border-box;padding:4px;margin-top:2px}"
         "label.chk{display:flex;gap:.5em;align-items:center}"
         "fieldset{margin:1em 0;border:1px solid #8886;border-radius:6px}legend{font-weight:600}"
         ".msg{padding:.6em;border-radius:6px;margin:1em 0}.err{background:#d6282822;color:#d62828}.okm{background:#1a9c3c22;color:#1a9c3c}"
         "button{padding:.5em 1.2em;margin-top:.5em}small{opacity:.7}</style></head><body>");
  h += F("<a href='/'>&larr; Status</a><h1>Settings</h1>");
  if (msg.length()) {
    h += isErr ? F("<div class='msg err'>") : F("<div class='msg okm'>");
    h += htmlEsc(msg.c_str());
    h += F("</div>");
  }
  if (!cfg.webPass[0]) h += F("<div class='msg err'>No web password is set: anyone on this network can open this page and change the settings.</div>");
  h += F("<form method='post' action='/settings'>");

  h += F("<fieldset><legend>Device</legend>");
  textField(h, "Hostname (letters, digits, '-')", "host", String(s.hostname));
  h += F("</fieldset>");

  h += F("<fieldset><legend>Ethernet (W5500)</legend>");
  checkField(h, "Enabled", "eEn", s.ethEnabled);
  addressing(h, "eMode", s.ethDhcp);
  textField(h, "IP address (static only)", "eIp", ipStr(s.ethIp));
  textField(h, "Subnet mask (static only)", "eMask", ipStr(s.ethMask));
  textField(h, "Gateway (static only)", "eGw", ipStr(s.ethGw));
  textField(h, "DNS (static only, blank = gateway)", "eDns", ipStr(s.ethDns));
  h += F("</fieldset>");

  h += F("<fieldset><legend>WiFi</legend>");
  checkField(h, "Enabled", "wEn", s.wifiEnabled);
  textField(h, "SSID", "wSsid", String(s.wifiSsid));
  textField(h, "Password (blank = keep current)", "wPass", String(""), "password");
  checkField(h, "Open network (clear the saved password)", "wPassClr", false);
  addressing(h, "wMode", s.wifiDhcp);
  textField(h, "IP address (static only)", "wIp", ipStr(s.wifiIp));
  textField(h, "Subnet mask (static only)", "wMask", ipStr(s.wifiMask));
  textField(h, "Gateway (static only)", "wGw", ipStr(s.wifiGw));
  textField(h, "DNS (static only, blank = gateway)", "wDns", ipStr(s.wifiDns));
  h += F("</fieldset>");

  static const char *names[3] = {"GNSS1 (ME31GR01)", "GNSS2 (NEO-M8N)", "RTC (DS3231)"};
  h += F("<fieldset><legend>Time sources (priority 1 = used first)</legend>");
  for (int i = 0; i < 3; i++) {
    char kEn[8], kP[8];
    snprintf(kEn, sizeof kEn, "sEn%d", i);
    snprintf(kP, sizeof kP, "sP%d", i);
    h += F("<div style='display:flex;gap:1em;align-items:center'>");
    h += F("<label class='chk' style='flex:1'><input type='checkbox' name='"); h += kEn; h += F("' value='1'");
    if (s.srcEnabled[i]) h += F(" checked");
    h += F("> "); h += names[i]; h += F("</label><select name='"); h += kP; h += F("' style='width:5em'>");
    for (int p = 1; p <= 3; p++) {
      char pv[4];
      snprintf(pv, sizeof pv, "%d", p);
      optionTag(h, pv, pv, s.srcPrio[i] == p);
    }
    h += F("</select></div>");
  }
  h += F("</fieldset>");

  h += F("<fieldset><legend>Web access</legend>"
         "<small>User name is <b>admin</b>. The password is sent unencrypted over HTTP, so it only keeps casual users out.</small>");
  textField(h, "New password (blank = keep current)", "newPass", String(""), "password");
  textField(h, "Repeat new password", "newPass2", String(""), "password");
  checkField(h, "Remove the password (open access)", "noPass", false);
  h += F("</fieldset>");

  h += F("<fieldset><legend>NTP access</legend>");
  textField(h, "Allowed clients (IPs or CIDR, comma separated; blank = everyone)", "allow", String(s.allow));
  h += F("</fieldset>");

  h += F("<button type='submit'>Save and restart</button></form>"
         "<form method='post' action='/reset' onsubmit=\"return confirm('Erase all settings and restart?')\">"
         "<button type='submit'>Factory reset</button></form>"
         "<p><small>If you get locked out, open the serial monitor and type <b>reset-settings</b> then Enter.</small></p>"
         "</body></html>");
  return h;
}

void sendResponse(NetworkClient &c, const char *status, const String &body, const char *extraHeaders = "") {
  c.print("HTTP/1.1 "); c.print(status);
  c.print("\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\nConnection: close\r\n");
  c.print(extraHeaders);
  c.print("Content-Length: "); c.print(body.length());
  c.print("\r\n\r\n");
  c.print(body);
}

static const char *RESTART_PAGE =
  "<!DOCTYPE html><html><head><meta charset='utf-8'><meta http-equiv='refresh' content='15;url=/'></head>"
  "<body><h3>Saved. Restarting...</h3><p>Reloading in 15 s. If the IP address changed, open the new one.</p></body></html>";

void serveWeb() {
  NetworkClient c = server.accept();
  if (!c) return;
  HttpReq r;
  if (!readHttpRequest(c, r)) { c.stop(); return; }
  int q = r.path.indexOf('?');
  if (q >= 0) r.path = r.path.substring(0, q);

  if (!authOk(r.auth)) {
    sendResponse(c, "401 Unauthorized", String("<h3>Password required</h3>"),
                 "WWW-Authenticate: Basic realm=\"Zaimu NTP server\"\r\n");
    c.stop();
    return;
  }
  bool post = (r.method == "POST");
  if (r.path == "/settings" && post) {
    String err = applySettingsForm(r.body);
    if (err.length()) {
      sendResponse(c, "200 OK", buildSettingsPage(err, true, draft));
    } else {
      sendResponse(c, "200 OK", String(RESTART_PAGE));
      c.stop();
      delay(500);
      ESP.restart();
    }
  } else if (r.path == "/settings") {
    sendResponse(c, "200 OK", buildSettingsPage("", false, cfg));
  } else if (r.path == "/reset" && post) {
    settingsErase();
    sendResponse(c, "200 OK", String(RESTART_PAGE));
    c.stop();
    delay(500);
    ESP.restart();
  } else if (r.path == "/" || r.path == "/index.html") {
    sendResponse(c, "200 OK", buildPage());
  } else {
    sendResponse(c, "404 Not Found", String("<h3>Not found</h3>"));
  }
  c.stop();
}

// ===================== SERIAL STATUS =====================
void printStatus() {
  Src s = selectSource();
  uint32_t ep, ms;
  char t[40] = "--";
  if (getTime(s, ep, ms)) fmtTime(ep, t, sizeof t);
  const char *c0, *c1;
  const char *s0 = gnssLevelText(0, &c0);
  const char *s1 = gnssLevelText(1, &c1);
  uint32_t p0, p1; int64_t u;
  readPps(0, p0, u);
  readPps(1, p1, u);
  String eip = ethGotIp ? ETH.localIP().toString() : String(cfg.ethEnabled ? "no IP" : "off");
  String wip = wifiGotIp ? WiFi.localIP().toString() : String(cfg.wifiEnabled ? "no IP" : "off");
  Serial.printf("%s | src=%s | G1: %s (PPS %lu) | G2: %s (PPS %lu) | RTC: %s | ETH: %s | WIFI: %s | NTP: %lu req\n",
                t, SRC_NAME[s], s0, (unsigned long)p0, s1, (unsigned long)p1,
                !rtcPresent ? "missing" : (rtcValid ? "ok" : "invalid"),
                eip.c_str(), wip.c_str(), (unsigned long)(ntpReqEth + ntpReqWifi));
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nBooting...");
  settingsLoad();
  settingsApplyDerived();
  Serial.println("Type reset-settings + Enter to erase saved settings.");
  Network.setHostname(cfg.hostname);

  // GNSS1 reset. ASSUMES ACTIVE-LOW: the pin is only ever driven low or released (high-Z), never driven high.
  if (GNSS1_RESET_ON_BOOT) {
    pinMode(GNSS1_RST_PIN, OUTPUT);
    digitalWrite(GNSS1_RST_PIN, LOW);
    delay(100);
  }
  pinMode(GNSS1_RST_PIN, INPUT);

  gnssBegin(0, "GNSS1 (ME31GR01)", &Serial1, GNSS1_RX_PIN, GNSS1_TX_PIN, GNSS1_PPS_PIN, GNSS1_BAUD, ppsIsr0);
  gnssBegin(1, "GNSS2 (NEO-M8N)",  &Serial2, GNSS2_RX_PIN, GNSS2_TX_PIN, GNSS2_PPS_PIN, GNSS2_BAUD, ppsIsr1);

  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  rtcInitOk = rtc.begin(&Wire);
  pollRtc();

  Network.onEvent(onEvent);
  if (cfg.ethEnabled) {
    pinMode(W5500_RST_PIN, OUTPUT);
    digitalWrite(W5500_RST_PIN, LOW);
    delay(100);
    digitalWrite(W5500_RST_PIN, HIGH);
    delay(200);
    SPI.begin(W5500_SCLK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN, W5500_CS_PIN);
    ETH.begin(ETH_PHY_W5500, 1, W5500_CS_PIN, W5500_INT_PIN, W5500_RST_PIN, SPI);
    if (!cfg.ethDhcp) {
      ETH.config(IPAddress(cfg.ethIp), IPAddress(cfg.ethGw), IPAddress(cfg.ethMask),
                 IPAddress(cfg.ethDns ? cfg.ethDns : cfg.ethGw));
    }
  }
  if (cfg.wifiEnabled) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    if (!cfg.wifiDhcp) {
      WiFi.config(IPAddress(cfg.wifiIp), IPAddress(cfg.wifiGw), IPAddress(cfg.wifiMask),
                  IPAddress(cfg.wifiDns ? cfg.wifiDns : cfg.wifiGw));
    }
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);               // modem sleep adds tens of ms of NTP jitter
    WiFi.begin(cfg.wifiSsid, cfg.wifiPass[0] ? cfg.wifiPass : nullptr);   // non-blocking
  }
  server.begin();
  xTaskCreatePinnedToCore(ntpTask, "ntp", 6144, nullptr, 10, nullptr, 1);
}

void loop() {
  gnssPoll(0);
  gnssPoll(1);

  static uint32_t lastRtcPoll = 0;
  if (millis() - lastRtcPoll >= 1000) { lastRtcPoll = millis(); pollRtc(); }

  maybeSyncRtc(selectSource());

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) { lastPrint = millis(); printStatus(); }

  static uint32_t lastNetPoll = 0;
  if (millis() - lastNetPoll >= 500) {
    lastNetPoll = millis();
    ethGotIp = cfg.ethEnabled && ETH.linkUp() && ((uint32_t)ETH.localIP() != 0);
    wifiGotIp = cfg.wifiEnabled && (WiFi.status() == WL_CONNECTED) && ((uint32_t)WiFi.localIP() != 0);
  }

  // Recovery: type "reset-settings" + Enter in the serial monitor to erase the saved settings.
  static String cmd;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      cmd.trim();
      if (cmd == "reset-settings") {
        settingsErase();
        Serial.println("Settings erased, restarting...");
        delay(300);
        ESP.restart();
      }
      cmd = "";
    } else if (cmd.length() < 40) {
      cmd += ch;
    }
  }

  if (ethGotIp || wifiGotIp) serveWeb();
  delay(1);
}