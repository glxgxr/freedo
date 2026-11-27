// The host link of the bridge: the native USB serial, or one TCP client over the WiFi. The bytes
// are the serial protocol of the CC1101 bridge either way, so the host reads a TCP link as a
// serial port (for example a pty from socat). Only the serial task calls this.
//
// The WiFi starts only when wifi_secrets.h exists next to the sketch (see
// wifi_secrets.h.example; git ignores the file). Without it the bridge speaks USB alone.
// While a TCP client is connected the bridge ignores USB input; a new connection replaces the
// old one, so a daemon that restarts gets the bridge back at once.

#pragma once

#include <Arduino.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#define LINK_WIFI 1
#else
#define LINK_WIFI 0
#endif

#if LINK_WIFI
#include <WiFi.h>
#include <atomic>
#include <errno.h>
#include "esp_timer.h"
#include "lwip/sockets.h"

static const uint16_t LINK_TCP_PORT = 7820;
static const char LINK_HOSTNAME[] = "freedo-bridge";
static const uint32_t LINK_POLL_US = 50000;
// The auto reconnect of the core gives up after some reasons (an authentication failure after
// the first connect, a leave of the access point), so a bridge without the WiFi asks again once
// the station has been quiet this long. An attempt of the core ends in an event well within it.
static const uint32_t LINK_WIFI_RETRY_MS = 5000;
// A peer that vanished (power cut, cable) is dropped after about 5 + 3 * 2 s of silence.
static const int LINK_KEEPIDLE_S = 5;
static const int LINK_KEEPINTVL_S = 2;
static const int LINK_KEEPCNT = 3;

static NetworkServer linkServer(LINK_TCP_PORT, 1);
static NetworkClient linkClient;
static int linkFd = -1;
static bool linkListening = false;
static uint64_t linkPolledUs = 0;
static uint32_t linkWifiAskedMs = 0;  // last time the bridge asked for the WiFi again
// Written by the event task of the core, read by the serial and the display task.
static std::atomic<uint32_t> linkWifiEventMs(0);  // last event of the station
static std::atomic<uint32_t> linkWifiDrops(0);    // times the WiFi went away after an address
static std::atomic<uint8_t> linkWifiReason(0);    // reason code of the last of them
static bool linkWifiHadIp = false;                // event task only

static void linkWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  if (event < ARDUINO_EVENT_WIFI_STA_START || event > ARDUINO_EVENT_WIFI_STA_LOST_IP) return;
  linkWifiEventMs.store(millis(), std::memory_order_relaxed);
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    linkWifiHadIp = true;
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED && linkWifiHadIp) {
    linkWifiHadIp = false;
    linkWifiReason.store(info.wifi_sta_disconnected.reason, std::memory_order_relaxed);
    linkWifiDrops.fetch_add(1, std::memory_order_relaxed);
  }
}

static void linkDrop() {
  linkClient.stop();
  linkFd = -1;
}

// Take a new TCP client, at most every LINK_POLL_US. The server starts once the WiFi has an
// address.
static void linkPoll() {
  uint64_t now = (uint64_t)esp_timer_get_time();
  if (now - linkPolledUs < LINK_POLL_US) return;
  linkPolledUs = now;
  bool up = WiFi.status() == WL_CONNECTED;
  uint32_t nowMs = millis();
  if (!up && nowMs - linkWifiEventMs.load(std::memory_order_relaxed) >= LINK_WIFI_RETRY_MS &&
      nowMs - linkWifiAskedMs >= LINK_WIFI_RETRY_MS) {
    WiFi.reconnect();
    linkWifiAskedMs = nowMs;
  }
  if (!linkListening) {
    if (!up) return;
    linkServer.begin();
    linkServer.setNoDelay(true);
    linkListening = true;
  }
  NetworkClient c = linkServer.accept();
  if (!c) return;
#ifdef LINK_ALLOWED_IP
  if (c.remoteIP() != IPAddress(LINK_ALLOWED_IP)) {
    c.stop();
    return;
  }
#endif
  linkDrop();
  linkClient = c;
  linkClient.setNoDelay(true);
  linkFd = linkClient.fd();
  int on = 1, idle = LINK_KEEPIDLE_S, intvl = LINK_KEEPINTVL_S, cnt = LINK_KEEPCNT;
  setsockopt(linkFd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
  setsockopt(linkFd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  setsockopt(linkFd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
  setsockopt(linkFd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
}
#endif

static void linkBegin() {
#if LINK_WIFI
  WiFi.onEvent(linkWifiEvent);
  linkWifiAskedMs = millis();
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(LINK_HOSTNAME);
  WiFi.setSleep(false);  // power save delays every reply by up to a beacon interval
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif
}

// One byte from the host, or -1.
static int linkRead() {
#if LINK_WIFI
  linkPoll();
  if (linkFd >= 0) {
    uint8_t b;
    int r = recv(linkFd, &b, 1, MSG_DONTWAIT);
    if (r == 1) return b;
    if (r == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) linkDrop();
    return -1;
  }
#endif
  return Serial.read();
}

// Write what fits now and return its length; 0 means try again later. The rest of a reply to
// a link that broke counts as written: nobody reads it.
static size_t linkWriteSome(const uint8_t *p, size_t n) {
#if LINK_WIFI
  linkPoll();
  if (linkFd >= 0) {
    int r = send(linkFd, p, n, MSG_DONTWAIT);
    if (r > 0) return (size_t)r;
    if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      linkDrop();
      return n;
    }
    return 0;
  }
#endif
  int room = Serial.availableForWrite();
  if (room <= 0) return 0;
  if ((size_t)room < n) n = (size_t)room;
  return Serial.write(p, n);
}
