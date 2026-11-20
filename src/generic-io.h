#ifndef GENERIC_IO_H_
#define GENERIC_IO_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_attr.h>

#define IO_LTE_UART_RX 4
#define IO_LTE_UART_TX 5
#define IO_LTE_POWER 48
#define IO_LTE_DP 16
#define IO_GNSS_RESET 8
#define IO_GNSS_UART_RX 18
#define IO_GNSS_UART_TX 17
#define IO_GNSS_POWER 45
#define IO_SOS 47
#define IO_BATT_ADC 3

#define SOS_ACTIVE_LEVEL 0
#define SOS_DEBOUNCE_MS 30
#define SOS_LONG_MS 6000
#define SOS_CLASSIFY_MS 6000
#define SOS_MAX_PRESS_MS 30000UL

#define LOG_BUF_MAX 2048
#define LOG_MAGIC 0x31474F4CUL

RTC_NOINIT_ATTR static uint32_t sLogMagic;
RTC_NOINIT_ATTR static uint32_t sLogLen;
RTC_NOINIT_ATTR static char sLogBuf[LOG_BUF_MAX];

static void logInit(void)
{
  if (sLogMagic != LOG_MAGIC || sLogLen >= LOG_BUF_MAX) {
    sLogLen = 0;
    sLogBuf[0] = 0;
  } else {
    sLogBuf[sLogLen] = 0;
  }
  sLogMagic = LOG_MAGIC;
}

static void logPersist(void)
{
  if (sLogLen >= LOG_BUF_MAX) {
    sLogLen = LOG_BUF_MAX - 1;
  }
  sLogBuf[sLogLen] = 0;
  sLogMagic = LOG_MAGIC;
}

static void logAppend(const char* msg)
{
  size_t mlen = strlen(msg);
  if (mlen == 0) {
    return;
  }
  if (mlen >= LOG_BUF_MAX - 1) {
    memcpy(sLogBuf, msg + mlen - (LOG_BUF_MAX - 2), LOG_BUF_MAX - 2);
    sLogBuf[LOG_BUF_MAX - 2] = '\n';
    sLogBuf[LOG_BUF_MAX - 1] = 0;
    sLogLen = LOG_BUF_MAX - 1;
    return;
  }
  if (sLogLen + mlen >= LOG_BUF_MAX) {
    size_t drop = sLogLen + mlen - (LOG_BUF_MAX - 1);
    if (drop > sLogLen) {
      drop = sLogLen;
    }
    memmove(sLogBuf, sLogBuf + drop, sLogLen - drop);
    sLogLen -= drop;
    sLogBuf[sLogLen] = 0;
  }
  memcpy(sLogBuf + sLogLen, msg, mlen);
  sLogLen += mlen;
  sLogBuf[sLogLen] = 0;
}

static void logMsg(const char* fmt, ...)
{
  char tmp[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  Serial.println(tmp);
  logAppend(tmp);
  logAppend("\n");
}

static const char* logBuf(void)
{
  return sLogBuf;
}

static size_t logLen(void)
{
  return (size_t)sLogLen;
}

static void ioInit(void)
{
  gpio_config_t cfg;
  cfg.intr_type = GPIO_INTR_DISABLE;
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pin_bit_mask = (1ULL << IO_LTE_POWER) | (1ULL << IO_LTE_DP) |
                     (1ULL << IO_GNSS_POWER) | (1ULL << IO_GNSS_RESET);
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&cfg);
  gpio_set_level((gpio_num_t)IO_LTE_POWER, 0);
  gpio_set_level((gpio_num_t)IO_LTE_DP, 0);
  gpio_set_level((gpio_num_t)IO_GNSS_POWER, 0);
  gpio_set_level((gpio_num_t)IO_GNSS_RESET, 0);
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pin_bit_mask = (1ULL << IO_SOS);
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&cfg);
}

static bool ioSosRaw(void)
{
  return gpio_get_level((gpio_num_t)IO_SOS) == SOS_ACTIVE_LEVEL;
}

static bool ioSosPressed(void)
{
  if (!ioSosRaw()) {
    return false;
  }
  delay(SOS_DEBOUNCE_MS);
  return ioSosRaw();
}

static uint32_t sSosPressStart = 0;
static bool sSosConfirmed = false;
static bool sSosLongFired = false;

static int ioSosUpdate(void)
{
  bool pressed = ioSosRaw();
  uint32_t now = millis();
  if (pressed) {
    if (sSosPressStart == 0) {
      sSosPressStart = now;
      sSosConfirmed = false;
      sSosLongFired = false;
    } else if (now - sSosPressStart > SOS_MAX_PRESS_MS) {
      sSosPressStart = now;
      sSosConfirmed = false;
      sSosLongFired = false;
      return 0;
    }
    if (!sSosConfirmed && now - sSosPressStart >= SOS_DEBOUNCE_MS) {
      sSosConfirmed = true;
    }
    if (sSosConfirmed && !sSosLongFired &&
        now - sSosPressStart >= SOS_LONG_MS) {
      sSosLongFired = true;
      logMsg("[SOS] long press dur=%lums masih ditahan, klasifikasi long",
             (unsigned long)(now - sSosPressStart));
      return 2;
    }
  } else {
    if (sSosPressStart > 0) {
      uint32_t dur = now - sSosPressStart;
      bool wasLong = sSosLongFired;
      bool wasConfirmed = sSosConfirmed;
      sSosPressStart = 0;
      sSosConfirmed = false;
      sSosLongFired = false;
      if (!wasConfirmed) {
        return 0;
      }
      if (wasLong) {
        return 0;
      }
      if (dur >= SOS_LONG_MS) {
        return 2;
      }
      if (dur > 0) {
        return 1;
      }
    }
  }
  return 0;
}

static int ioSosClassifyWake(void)
{
  uint32_t start = millis();
  int cls = 0;
  while (millis() - start < SOS_CLASSIFY_MS) {
    cls = ioSosUpdate();
    if (cls == 1 || cls == 2) {
      break;
    }
    delay(20);
  }
  if (cls == 0) {
    if (sSosPressStart == 0 && !ioSosRaw()) {
      return 0;
    }
  }
  return cls;
}

#endif