#ifndef GNSS_H_
#define GNSS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>
#include <driver/uart.h>
#include <esp_task_wdt.h>
#include "generic-io.h"

#define GNSS_UART UART_NUM_2
#define GNSS_BAUD 115200
#define GNSS_LINE_MAX 192
#define GNSS_FIELDS_MAX 32
#define GNSS_SEARCH_TIMEOUT_MS 300000UL
#define GNSS_SEARCH_TIMEOUT_MAX_MS 300000UL
#define GNSS_RESET_HOLD_MS 100
#define GNSS_NO_FIX_MS 0xFFFFFFFFUL

typedef struct {
  bool valid;
  bool fixActive;
  uint8_t fixQuality;
  uint8_t fixType;
  uint8_t satellitesUsed;
  uint16_t satsInView;
  uint16_t hdopC;
  uint16_t pdopC;
  uint16_t vdopC;
  int32_t latE7;
  int32_t lonE7;
  bool latSouth;
  bool lonWest;
  int32_t altCM;
  int32_t geoidSepCM;
  uint32_t timeHms;
  uint32_t dateDmy;
  uint16_t speedCKn;
  uint16_t courseCDeg;
  int16_t magVarCDeg;
  char magVarDir;
  char modeIndicator;
  char lastSentence[96];
  uint32_t lastFixMs;
  uint32_t fixStartMs;
  bool hasFix;
} GnssData;

static GnssData sGnss;
static char sGnssLine[GNSS_LINE_MAX];
static size_t sGnssLineLen = 0;
static char* sGnssFields[GNSS_FIELDS_MAX];
static uint8_t sGnssFieldCount = 0;
static bool sGnssPowered = false;

static void gnssClear(void)
{
  memset(&sGnss, 0, sizeof(sGnss));
  sGnss.hdopC = 10000;
  sGnss.pdopC = 10000;
  sGnss.vdopC = 10000;
  sGnss.modeIndicator = 'N';
  sGnss.magVarDir = ' ';
}

static void gnssMarkPowerOn(void)
{
  sGnss.fixStartMs = millis();
  sGnss.lastFixMs = 0;
  sGnss.hasFix = false;
}

static void gnssSplit(char* line)
{
  sGnssFieldCount = 0;
  sGnssFields[0] = line;
  sGnssFieldCount = 1;
  char* p = line;
  while (*p != 0 && sGnssFieldCount < GNSS_FIELDS_MAX) {
    if (*p == ',') {
      *p = 0;
      sGnssFields[sGnssFieldCount] = p + 1;
      sGnssFieldCount++;
    }
    p++;
  }
}

static int32_t gnssToE7(const char* v, bool neg)
{
  double d = strtod(v, 0);
  int32_t deg = (int32_t)(d / 100.0);
  double min = d - (double)deg * 100.0;
  int32_t e7 = (int32_t)((deg + min / 60.0) * 10000000.0);
  return neg ? -e7 : e7;
}

static uint16_t gnssToCenti(const char* v)
{
  if (v[0] == 0) {
    return 0;
  }
  double d = strtod(v, 0);
  if (d < 0) {
    d = -d;
  }
  if (d > 9999.0) {
    d = 9999.0;
  }
  return (uint16_t)(d * 100.0);
}

static void gnssApplyGga(char** f, uint8_t n)
{
  if (n < 15) {
    return;
  }
  if (f[1][0] != 0) {
    sGnss.timeHms = (uint32_t)atoi(f[1]);
  }
  if (f[2][0] != 0 && f[3][0] != 0) {
    sGnss.latE7 = gnssToE7(f[2], f[3][0] == 'S');
    sGnss.latSouth = f[3][0] == 'S';
  }
  if (f[4][0] != 0 && f[5][0] != 0) {
    sGnss.lonE7 = gnssToE7(f[4], f[5][0] == 'W');
    sGnss.lonWest = f[5][0] == 'W';
  }
  sGnss.fixQuality = (uint8_t)atoi(f[6]);
  sGnss.satellitesUsed = (uint8_t)atoi(f[7]);
  sGnss.hdopC = gnssToCenti(f[8]);
  sGnss.altCM = (int32_t)(strtod(f[9], 0) * 100.0);
  sGnss.geoidSepCM = (int32_t)(strtod(f[11], 0) * 100.0);
  sGnss.fixActive = sGnss.fixQuality > 0;
  sGnss.valid = sGnss.fixActive;
  sGnss.modeIndicator = sGnss.fixActive ? 'A' : 'N';
}

static void gnssApplyRmc(char** f, uint8_t n)
{
  if (n < 12) {
    return;
  }
  if (f[1][0] != 0) {
    sGnss.timeHms = (uint32_t)atoi(f[1]);
  }
  if (f[9][0] != 0) {
    sGnss.dateDmy = (uint32_t)atoi(f[9]);
  }
  bool active = (f[2][0] == 'A');
  if (active) {
    if (f[3][0] != 0 && f[4][0] != 0) {
      sGnss.latE7 = gnssToE7(f[3], f[4][0] == 'S');
      sGnss.latSouth = f[4][0] == 'S';
    }
    if (f[5][0] != 0 && f[6][0] != 0) {
      sGnss.lonE7 = gnssToE7(f[5], f[6][0] == 'W');
      sGnss.lonWest = f[6][0] == 'W';
    }
  }
  sGnss.speedCKn = (uint16_t)(strtod(f[7], 0) * 100.0);
  sGnss.courseCDeg = (uint16_t)(strtod(f[8], 0) * 100.0);
  sGnss.magVarCDeg = (int16_t)(strtod(f[10], 0) * 100.0);
  if (f[11][0] != 0) {
    sGnss.magVarDir = f[11][0];
  }
  if (n > 12 && f[12][0] != 0) {
    sGnss.modeIndicator = f[12][0];
  } else {
    sGnss.modeIndicator = active ? 'A' : 'N';
  }
  sGnss.fixActive = active;
  if (active && sGnss.fixQuality == 0) {
    sGnss.fixQuality = 1;
  }
  sGnss.valid = active;
}

static void gnssApplyGsa(char** f, uint8_t n)
{
  if (n < 18) {
    return;
  }
  uint8_t ft = (uint8_t)atoi(f[2]);
  sGnss.fixType = ft;
  uint16_t pd = gnssToCenti(f[15]);
  if (pd > 0) {
    sGnss.pdopC = pd;
  }
  uint16_t hd = gnssToCenti(f[16]);
  if (hd > 0) {
    sGnss.hdopC = hd;
  }
  uint16_t vd = gnssToCenti(f[17]);
  if (vd > 0) {
    sGnss.vdopC = vd;
  }
  if (ft >= 2) {
    sGnss.fixActive = true;
    if (sGnss.fixQuality == 0) {
      sGnss.fixQuality = 1;
    }
    sGnss.valid = true;
  }
}

static void gnssApplyGsv(char** f, uint8_t n)
{
  if (n < 4) {
    return;
  }
  uint16_t total = (uint16_t)atoi(f[3]);
  if (total > sGnss.satsInView) {
    sGnss.satsInView = total;
  }
}

static void gnssApplyGns(char** f, uint8_t n)
{
  if (n < 10) {
    return;
  }
  if (f[1][0] != 0) {
    sGnss.timeHms = (uint32_t)atoi(f[1]);
  }
  if (f[2][0] != 0 && f[3][0] != 0) {
    sGnss.latE7 = gnssToE7(f[2], f[3][0] == 'S');
    sGnss.latSouth = f[3][0] == 'S';
  }
  if (f[4][0] != 0 && f[5][0] != 0) {
    sGnss.lonE7 = gnssToE7(f[4], f[5][0] == 'W');
    sGnss.lonWest = f[5][0] == 'W';
  }
  if (f[6][0] != 0) {
    sGnss.modeIndicator = f[6][0];
  }
  sGnss.satellitesUsed = (uint8_t)atoi(f[7]);
  uint16_t hd = gnssToCenti(f[8]);
  if (hd > 0) {
    sGnss.hdopC = hd;
  }
  sGnss.altCM = (int32_t)(strtod(f[9], 0) * 100.0);
  if (n > 10) {
    sGnss.geoidSepCM = (int32_t)(strtod(f[10], 0) * 100.0);
  } else {
    sGnss.geoidSepCM = 0;
  }
  bool hasFix = f[6][0] != 0 && strchr(f[6], 'N') == 0;
  if (hasFix) {
    sGnss.fixActive = true;
    sGnss.valid = true;
    if (sGnss.fixQuality == 0) {
      sGnss.fixQuality = 1;
    }
  }
}

static bool gnssParseLine(char* line)
{
  char* star = strchr(line, '*');
  if (star != 0) {
    *star = 0;
  }
  if (strncmp(line + 3, "GGA", 3) == 0) {
    gnssSplit(line);
    gnssApplyGga(sGnssFields, sGnssFieldCount);
    return true;
  }
  if (strncmp(line + 3, "RMC", 3) == 0) {
    gnssSplit(line);
    gnssApplyRmc(sGnssFields, sGnssFieldCount);
    return true;
  }
  if (strncmp(line + 3, "GSA", 3) == 0) {
    gnssSplit(line);
    gnssApplyGsa(sGnssFields, sGnssFieldCount);
    return true;
  }
  if (strncmp(line + 3, "GSV", 3) == 0) {
    gnssSplit(line);
    gnssApplyGsv(sGnssFields, sGnssFieldCount);
    return true;
  }
  if (strncmp(line + 3, "GNS", 3) == 0) {
    gnssSplit(line);
    gnssApplyGns(sGnssFields, sGnssFieldCount);
    return true;
  }
  return false;
}

static bool gnssHasFix(void)
{
  return sGnss.fixActive && sGnss.valid && sGnss.fixQuality > 0;
}

static void gnssFeed(void)
{
  uint8_t buf[64];
  int n = uart_read_bytes(GNSS_UART, buf, sizeof(buf), 0);
  for (int i = 0; i < n; i++) {
    uint8_t c = buf[i];
    if (c == '\n') {
      sGnssLine[sGnssLineLen] = 0;
      if (sGnssLineLen > 6 && sGnssLine[0] == '$') {
        snprintf(sGnss.lastSentence, sizeof(sGnss.lastSentence), "%s", sGnssLine);
        if (gnssParseLine(sGnssLine) && gnssHasFix()) {
          uint32_t now = millis();
          if (!sGnss.hasFix) {
            sGnss.hasFix = true;
            if (sGnss.fixStartMs == 0) {
              sGnss.fixStartMs = now;
            }
            logMsg("[GNSS] fix detected: %s", sGnss.lastSentence);
          }
          sGnss.lastFixMs = now;
        }
      }
      sGnssLineLen = 0;
    } else if (c != '\r') {
      if (sGnssLineLen >= sizeof(sGnssLine) - 1) {
        sGnssLineLen = 0;
      } else {
        sGnssLine[sGnssLineLen++] = (char)c;
      }
    }
  }
}

static void gnssUartInit(void)
{
  uart_config_t uc;
  memset(&uc, 0, sizeof(uc));
  uc.baud_rate = GNSS_BAUD;
  uc.data_bits = UART_DATA_8_BITS;
  uc.parity = UART_PARITY_DISABLE;
  uc.stop_bits = UART_STOP_BITS_1;
  uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uc.source_clk = UART_SCLK_DEFAULT;
  uart_param_config(GNSS_UART, &uc);
  uart_set_pin(GNSS_UART, IO_GNSS_UART_TX, IO_GNSS_UART_RX,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(GNSS_UART, 1024, 0, 0, 0, 0);
}

static void gnssEnable(void)
{
  if (sGnssPowered) {
    return;
  }
  gpio_set_level((gpio_num_t)IO_GNSS_POWER, 1);
  gpio_set_level((gpio_num_t)IO_GNSS_RESET, 1);
  sGnssPowered = true;
  Serial.println("[GNSS] enabled at idle (power+reset high)");
}

static bool gnssPowered(void)
{
  return sGnssPowered;
}

static void gnssPowerOn(void)
{
  if (sGnssPowered) {
    sGnssLineLen = 0;
    gnssClear();
    gnssMarkPowerOn();
    Serial.println("[GNSS] already powered, wait fresh fix");
    return;
  }
  gpio_set_level((gpio_num_t)IO_GNSS_POWER, 1);
  gpio_set_level((gpio_num_t)IO_GNSS_RESET, 1);
  sGnssLineLen = 0;
  gnssClear();
  gnssMarkPowerOn();
  sGnssPowered = true;
  Serial.println("[GNSS] power on (reset kept high, warm start)");
}

static void gnssPowerOff(void)
{
  gpio_set_level((gpio_num_t)IO_GNSS_POWER, 0);
  gpio_set_level((gpio_num_t)IO_GNSS_RESET, 1);
  sGnssPowered = false;
  Serial.println("[GNSS] power off (reset high, backup retained)");
}

static void gnssResetPulse(void)
{
  gpio_set_level((gpio_num_t)IO_GNSS_RESET, 0);
  delay(GNSS_RESET_HOLD_MS);
  gpio_set_level((gpio_num_t)IO_GNSS_RESET, 1);
}

static bool gnssWaitFix(uint32_t timeoutMs)
{
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    gnssFeed();
    if (gnssHasFix()) {
      return true;
    }
    esp_task_wdt_reset();
    delay(20);
  }
  return false;
}

static const GnssData* gnssGet(void)
{
  return &sGnss;
}

static uint32_t gnssTimeSinceLastFixMs(void)
{
  if (sGnss.lastFixMs == 0) {
    return GNSS_NO_FIX_MS;
  }
  return millis() - sGnss.lastFixMs;
}

static uint32_t gnssTimeToFirstFixMs(void)
{
  if (!sGnss.hasFix || sGnss.fixStartMs == 0) {
    return GNSS_NO_FIX_MS;
  }
  return sGnss.lastFixMs >= sGnss.fixStartMs ? sGnss.lastFixMs - sGnss.fixStartMs : 0;
}

static void gnssPrint(const GnssData* g)
{
  int32_t la = g->latE7 < 0 ? -g->latE7 : g->latE7;
  int32_t lo = g->lonE7 < 0 ? -g->lonE7 : g->lonE7;
  logMsg("[GNSS] UTC %06lu date %06lu fix=%u mode=%c type=%u sat=%u/%u lat=%d.%05ld lon=%d.%05ld alt=%d.%02dm geo=%d.%02dm",
         (unsigned long)g->timeHms, (unsigned long)g->dateDmy,
         g->fixQuality, g->modeIndicator, g->fixType, g->satellitesUsed, g->satsInView,
         (int)(la / 10000000), (long)((la % 10000000) / 100),
         (int)(lo / 10000000), (long)((lo % 10000000) / 100),
         (int)(g->altCM / 100), (int)((g->altCM % 100) / 10),
         (int)(g->geoidSepCM / 100), (int)((g->geoidSepCM % 100) / 10));
  logMsg("[GNSS] hdop=%d.%02u pdop=%d.%02u vdop=%d.%02u spd=%d.%02ukn crs=%d.%02u ttff=%lums",
         (int)(g->hdopC / 100), g->hdopC % 100,
         (int)(g->pdopC / 100), g->pdopC % 100,
         (int)(g->vdopC / 100), g->vdopC % 100,
         (int)(g->speedCKn / 100), g->speedCKn % 100,
         (int)(g->courseCDeg / 100), g->courseCDeg % 100,
         (unsigned long)gnssTimeToFirstFixMs());
}

#endif
