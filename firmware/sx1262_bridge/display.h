// Status display of the bridge on the OLED of the Heltec V4 (SSD1306, 128x64, I2C on GPIO 17/18,
// reset on GPIO 21, powered by Vext). It shows the WLAN, the host link, the devices that talk to
// this coordinator and the outcome of the transmit jobs. A task of its own on core 0 draws it,
// away from the radio task. An OLED burns in, so it lights for DISPLAY_ON_MS at boot and after
// each press of the PRG button.

#pragma once

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <atomic>
#include <string.h>

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const int PIN_OLED_SDA = 17;
static const int PIN_OLED_SCL = 18;
static const int PIN_OLED_RESET = 21;
static const int PIN_PRG_BUTTON = 0;
static const uint32_t DISPLAY_ON_MS = 60000;
static const uint32_t DISPLAY_REFRESH_MS = 1000;
static const uint32_t DISPLAY_POLL_MS = 50;
// A valve reports at least every hour, so two hours tell a device that left from one that is quiet.
static const uint32_t DISPLAY_DEVICE_WINDOW_MS = 2UL * 3600UL * 1000UL;
static const uint8_t DISPLAY_DEVICES = 16;
static const uint8_t DISPLAY_ID_LEN = 5;  // the device part of an EUI, as the Enhanced ACK carries it
static const uint32_t DISPLAY_HOST_MS = 5000;
static const uint32_t DISPLAY_STACK = 4096;
static const UBaseType_t DISPLAY_PRIO = 1;
static const int DISPLAY_CORE = 0;

struct DisplayDevice {
  bool used;
  uint8_t id[DISPLAY_ID_LEN];
  uint32_t lastMs;
};

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C dispOled(U8G2_R0, PIN_OLED_RESET, PIN_OLED_SCL, PIN_OLED_SDA);
static DisplayDevice dispDevices[DISPLAY_DEVICES];
static portMUX_TYPE dispMux = portMUX_INITIALIZER_UNLOCKED;
static std::atomic<uint32_t> dispJobsOk(0);
static std::atomic<uint32_t> dispJobsFailed(0);
static std::atomic<uint32_t> *dispHostMs = nullptr;
static const char *dispBuild = "";

// A device of this coordinator was heard: an uplink addressed to it, or the Enhanced ACK to one of
// its Data frames. The radio task calls this.
static void displayNoteDevice(const uint8_t *id) {
  uint32_t now = millis();
  portENTER_CRITICAL(&dispMux);
  int idx = -1, freeIdx = -1, oldest = -1;
  for (int i = 0; i < DISPLAY_DEVICES; i++) {
    DisplayDevice &d = dispDevices[i];
    if (d.used && memcmp(d.id, id, DISPLAY_ID_LEN) == 0) {
      idx = i;
      break;
    }
    if (!d.used) {
      if (freeIdx < 0) freeIdx = i;
    } else if (oldest < 0 || now - d.lastMs > now - dispDevices[oldest].lastMs) {
      oldest = i;
    }
  }
  if (idx < 0) idx = (freeIdx >= 0) ? freeIdx : oldest;
  dispDevices[idx].used = true;
  memcpy(dispDevices[idx].id, id, DISPLAY_ID_LEN);
  dispDevices[idx].lastMs = now;
  portEXIT_CRITICAL(&dispMux);
}

static void displayNoteJob(bool ok) {
  (ok ? dispJobsOk : dispJobsFailed).fetch_add(1, std::memory_order_relaxed);
}

static uint8_t displayDeviceCount() {
  uint32_t now = millis();
  uint8_t n = 0;
  portENTER_CRITICAL(&dispMux);
  for (int i = 0; i < DISPLAY_DEVICES; i++) {
    if (dispDevices[i].used && now - dispDevices[i].lastMs < DISPLAY_DEVICE_WINDOW_MS) n++;
  }
  portEXIT_CRITICAL(&dispMux);
  return n;
}

// Why the chip started last, at most 7 characters: it tells a power supply that sags from a
// firmware that crashes.
static const char *dispResetName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "Strom";
    case ESP_RST_EXT:      return "Taste";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_USB:      return "USB";
    case ESP_RST_PANIC:    return "Absturz";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_BROWNOUT: return "U tief";
    default:               return "?";
  }
}

static void displayDraw() {
  static const int LINE_Y[] = {10, 21, 32, 43, 54, 64};
  char line[40];
  uint32_t now = millis();
  dispOled.clearBuffer();
  dispOled.setFont(u8g2_font_6x10_tf);

  snprintf(line, sizeof(line), "freedo %s", dispBuild);
  dispOled.drawUTF8(0, LINE_Y[0], line);

#if LINK_WIFI
  wl_status_t wl = WiFi.status();
  if (wl == WL_CONNECTED) {
    snprintf(line, sizeof(line), "WLAN %d dBm  .%u", (int)WiFi.RSSI(), (unsigned)WiFi.localIP()[3]);
  } else if (wl == WL_NO_SSID_AVAIL) {
    snprintf(line, sizeof(line), "WLAN weg: kein Netz");
  } else if (wl == WL_CONNECT_FAILED) {
    snprintf(line, sizeof(line), "WLAN weg: Anmeldung");
  } else {
    snprintf(line, sizeof(line), "WLAN getrennt");
  }
#else
  snprintf(line, sizeof(line), "WLAN aus (nur USB)");
#endif
  dispOled.drawUTF8(0, LINE_Y[1], line);

  bool hostActive = dispHostMs != nullptr &&
                    now - dispHostMs->load(std::memory_order_relaxed) < DISPLAY_HOST_MS;
  const char *host = "keiner";
#if LINK_WIFI
  if (linkFd >= 0) host = hostActive ? "WLAN" : "WLAN, still";
  else if (hostActive) host = "USB";
#else
  if (hostActive) host = "USB";
#endif
  snprintf(line, sizeof(line), "Daemon: %s", host);
  dispOled.drawUTF8(0, LINE_Y[2], line);

  snprintf(line, sizeof(line), "Ventile: %u (2 h)", (unsigned)displayDeviceCount());
  dispOled.drawUTF8(0, LINE_Y[3], line);

  snprintf(line, sizeof(line), "Aufträge ok %lu F %lu",
           (unsigned long)dispJobsOk.load(std::memory_order_relaxed),
           (unsigned long)dispJobsFailed.load(std::memory_order_relaxed));
  dispOled.drawUTF8(0, LINE_Y[4], line);

  uint32_t mins = now / 60000UL;
  snprintf(line, sizeof(line), "%s %luh%02lum %s", SX_TX_ENABLED ? "TX an" : "nur RX",
           (unsigned long)(mins / 60), (unsigned long)(mins % 60), dispResetName());
  dispOled.drawUTF8(0, LINE_Y[5], line);

  dispOled.sendBuffer();
}

static void displayTask(void *) {
  pinMode(PIN_PRG_BUTTON, INPUT_PULLUP);
  dispOled.begin();
  uint32_t onUntil = millis() + DISPLAY_ON_MS;
  bool lit = true;
  uint32_t drawnAt = 0;
  bool drawn = false;
  for (;;) {
    uint32_t now = millis();
    if (digitalRead(PIN_PRG_BUTTON) == LOW) onUntil = now + DISPLAY_ON_MS;
    bool want = (int32_t)(onUntil - now) > 0;
    if (want != lit) {
      dispOled.setPowerSave(want ? 0 : 1);
      lit = want;
      drawn = false;
    }
    if (lit && (!drawn || now - drawnAt >= DISPLAY_REFRESH_MS)) {
      displayDraw();
      drawnAt = now;
      drawn = true;
    }
    vTaskDelay(pdMS_TO_TICKS(DISPLAY_POLL_MS));
  }
}

// After sxBegin, which switches Vext on. hostMs is the last request of the host, in ms.
static void displayBegin(const char *build, std::atomic<uint32_t> *hostMs) {
  dispBuild = build;
  dispHostMs = hostMs;
  xTaskCreatePinnedToCore(displayTask, "display", DISPLAY_STACK, nullptr, DISPLAY_PRIO, nullptr, DISPLAY_CORE);
}
