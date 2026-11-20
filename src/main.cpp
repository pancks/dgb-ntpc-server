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
// WiFi (station mode). Ethernet and WiFi run in parallel; the status page answers on either IP.
static const bool WIFI_ENABLED = true;   // set false to test GNSS reception with the WiFi radio off
#define WIFI_SSID "Digiboy"
#define WIFI_PASS "DGBsemangat"
static const uint32_t GNSS1_BAUD = 115200;       // fixed baud for GNSS1 (ME31GR01)
static const uint32_t GNSS2_BAUD = 9600;         // fixed baud for GNSS2 (NEO-M8N)
static const bool     GNSS_AUTOBAUD = false;     // true = scan BAUDS[] if no valid NMEA arrives
static const uint32_t GNSS_HOLD_MS = 3000;       // no fresh fix-time for this long -> source lost
static const bool     GNSS1_RESET_ON_BOOT = true; // assumes ACTIVE-LOW reset, see note below

// ===================== PPS (interrupts) =====================
portMUX_TYPE ppsMux = portMUX_INITIALIZER_UNLOCKED;
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
      g.anchorEpoch = dt.unixtime();
      g.anchorUs = aligned ? pus : now;
      g.ppsAligned = aligned;
      g.anchorValid = true;
      g.anchorWallMs = millis();
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
  rtcLost = rtc.lostPower();
  DateTime n = rtc.now();
  rtcEpoch = n.unixtime();
  rtcReadUs = esp_timer_get_time();
  rtcTemp = rtc.getTemperature();
  rtcValid = !rtcLost && n.year() >= 2024;
}
uint32_t rtcNowEpoch() {
  return rtcEpoch + (uint32_t)((esp_timer_get_time() - rtcReadUs) / 1000000);
}

// ===================== TIME SELECTION =====================
enum Src { SRC_NONE, SRC_GNSS1, SRC_GNSS2, SRC_RTC };
const char *SRC_NAME[] = {"NONE", "GNSS1", "GNSS2", "RTC"};

Src selectSource() {
  if (gnssUsable(0)) return SRC_GNSS1;
  if (gnssUsable(1)) return SRC_GNSS2;
  if (rtcPresent && rtcValid) return SRC_RTC;
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
      ethGotIp = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH link down");
      ethGotIp = false;
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("WiFi IP: "); Serial.println(WiFi.localIP());
      wifiGotIp = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi disconnected");
      wifiGotIp = false;
      break;
    default: break;
  }
}

void row(String &h, const char *k, const char *v, const char *cls = "") {
  h += F("<tr><td>"); h += k; h += F("</td><td class='"); h += cls; h += F("'>"); h += v; h += F("</td></tr>");
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
  row(h, "Link", ETH.linkUp() ? "UP" : "DOWN", ETH.linkUp() ? "ok" : "bad");
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
  row(h, "Link", up ? "CONNECTED" : (WIFI_ENABLED ? "DISCONNECTED" : "DISABLED"), up ? "ok" : (WIFI_ENABLED ? "bad" : "warn"));
  row(h, "SSID", WIFI_SSID);
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
         ".clock{font-size:2em;font-weight:600}h2{margin:1em 0 .3em}</style></head><body>");
  if (have) {
    char t[40];
    fmtTime(ep, t, sizeof t);
    h += F("<div class='clock'>"); h += t; h += F("</div><div>Source: <b>"); h += SRC_NAME[sel]; h += F("</b></div>");
  } else {
    h += F("<div class='clock bad'>NO VALID TIME SOURCE</div>");
  }
  addGnss(h, 0);
  addGnss(h, 1);
  addRtc(h, sel);
  addEth(h);
  addWifi(h);
  h += F("</body></html>");
  return h;
}

void serveWeb() {
  NetworkClient c = server.accept();
  if (!c) return;
  uint32_t t0 = millis();
  while (c.connected() && !c.available() && millis() - t0 < 300) {
    gnssPoll(0); gnssPoll(1);
  }
  while (c.available()) c.read();               // request content is ignored, every path returns the status page
  String body = buildPage();
  c.print("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\n"
          "Connection: close\r\nContent-Length: ");
  c.print(body.length());
  c.print("\r\n\r\n");
  c.print(body);
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
  String eip = ethGotIp ? ETH.localIP().toString() : String("no IP");
  String wip = wifiGotIp ? WiFi.localIP().toString() : String(WIFI_ENABLED ? "no IP" : "off");
  Serial.printf("%s | src=%s | G1: %s (PPS %lu) | G2: %s (PPS %lu) | RTC: %s | ETH: %s | WIFI: %s\n",
                t, SRC_NAME[s], s0, (unsigned long)p0, s1, (unsigned long)p1,
                !rtcPresent ? "missing" : (rtcValid ? "ok" : "invalid"),
                eip.c_str(), wip.c_str());
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nBooting...");

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

  pinMode(W5500_RST_PIN, OUTPUT);
  digitalWrite(W5500_RST_PIN, LOW);
  delay(100);
  digitalWrite(W5500_RST_PIN, HIGH);
  delay(200);

  Network.onEvent(onEvent);
  SPI.begin(W5500_SCLK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN, W5500_CS_PIN);
  ETH.begin(ETH_PHY_W5500, 1, W5500_CS_PIN, W5500_INT_PIN, W5500_RST_PIN, SPI);
  if (WIFI_ENABLED) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);   // non-blocking; IP arrives via the event handler
  }
  server.begin();
}

void loop() {
  gnssPoll(0);
  gnssPoll(1);

  static uint32_t lastRtcPoll = 0;
  if (millis() - lastRtcPoll >= 1000) { lastRtcPoll = millis(); pollRtc(); }

  maybeSyncRtc(selectSource());

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) { lastPrint = millis(); printStatus(); }

  if (ethGotIp || wifiGotIp) serveWeb();
  delay(1);
}