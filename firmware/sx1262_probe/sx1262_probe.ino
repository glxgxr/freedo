// Receive-only probe: can an SX1262 hear the tado V3+ 868 MHz network?
//
// Board: Heltec WiFi LoRa 32 V4 and its clones (ESP32-S3 + SX1262 + RF front end).
// This sketch never transmits: it holds no transmit call, and the PA of the front end
// stays in bypass.
//
// Output, on the native USB serial and on UART0 at 115200:
//   "# ..." lines are status. A frame line has the layout of `freedo-dev rx`, so the
//   freedo replay and dissect tools read it:
//   <seconds> [CRC-OK rssi=-71 mode=HW] <hex of the MAC frame>
//
// Two receive modes, switched with the PRG button (GPIO0):
//   HW  variable length, the SX1262 checks the CC1101 CRC itself.
//   RAW fixed length, the sketch checks the CC1101 CRC in software. Use it when HW
//       shows only CRC-BAD frames: it tells whether the SX1262 CRC setup or the
//       reception is wrong.

#include <RadioLib.h>
#include <SPI.h>
#include <stdarg.h>
#include "driver/gpio.h"

// Heltec V4 pin map, from Meshtastic variants/esp32s3/heltec_v4/variant.h.
static const int PIN_LORA_CS = 8;
static const int PIN_LORA_SCK = 9;
static const int PIN_LORA_MOSI = 10;
static const int PIN_LORA_MISO = 11;
static const int PIN_LORA_RST = 12;
static const int PIN_LORA_BUSY = 13;
static const int PIN_LORA_DIO1 = 14;
static const int PIN_VEXT = 36;         // active low
static const int PIN_FEM_POWER = 7;     // LDO of the RF front end
static const int PIN_FEM_CSD = 2;       // front end enable, read first to tell the two front ends apart
static const int PIN_FEM_KCT_CTX = 5;   // KCT8103L (V4.3): low selects the receive LNA
static const int PIN_FEM_GC_CPS = 46;   // GC1109 (V4.2): low keeps the PA in bypass
static const int PIN_BUTTON = 0;

// tado PHY, from tadoRegs in freedo cc1101/cc1101.go (CC1101, 26 MHz crystal).
static const float FREQ_MHZ = 868.3237;    // FREQ 0x213276 = 863.1249 MHz + CHANNR 26 x 199.951 kHz
static const float BITRATE_KBPS = 49.988;  // MDMCFG4/3: DRATE_E 10, DRATE_M 248
static const float DEVIATION_KHZ = 25.391; // DEVIATN 0x40
static const float RX_BW_KHZ = 156.2;      // the CC1101 uses 135 kHz; the SX1262 steps are 117.3 and 156.2
static const uint16_t PREAMBLE_BITS = 32;  // MDMCFG1: 4 bytes
static uint8_t SYNC_WORD[] = {0xD3, 0x91, 0xD3, 0x91}; // SYNC_MODE 3 sends the 16-bit word twice
static const uint8_t MAX_FRAME = 140;      // PKTLEN 0x8C
static const uint8_t RAW_LEN = 1 + MAX_FRAME + 2;

SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY, SPI);

static volatile bool rxFlag = false;
static bool radioOk = false;
static bool hwMode = true;
static float tcxoVolts = 1.8;
static const char *femName = "?";
static uint32_t framesOk = 0, framesBad = 0, framesLen = 0;
static uint32_t lastStats = 0;

static void IRAM_ATTR onPacket() { rxFlag = true; }

static void out(const char *fmt, ...) {
  char buf[640];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  Serial.print(buf);
  Serial0.print(buf);
}

// CC1101 CRC-16: polynomial 0x8005, seed 0xFFFF, over the length byte and the payload,
// sent most significant byte first.
static uint16_t cc1101Crc(const uint8_t *p, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)p[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
  }
  return crc;
}

static void printFrame(const char *flag, float rssi, const uint8_t *p, size_t n) {
  static char hex[2 * 256 + 1];
  for (size_t i = 0; i < n; i++) sprintf(hex + 2 * i, "%02x", p[i]);
  hex[2 * n] = 0;
  out("%.3f [%s rssi=%.0f mode=%s] %s\n", millis() / 1000.0, flag, rssi, hwMode ? "HW" : "RAW", hex);
}

static void check(const char *what, int16_t st) {
  if (st != RADIOLIB_ERR_NONE) out("# %s failed: %d\n", what, st);
}

// The V4.2 and V4.3 carry different front ends. Meshtastic tells them apart by the level
// of the enable pin right after the front end is powered.
static void frontEndReceive() {
  const int pins[] = {PIN_FEM_POWER, PIN_FEM_CSD, PIN_FEM_KCT_CTX, PIN_FEM_GC_CPS};
  for (int p : pins) gpio_hold_dis((gpio_num_t)p);
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  pinMode(PIN_FEM_POWER, OUTPUT);
  digitalWrite(PIN_FEM_POWER, HIGH);
  delay(1);
  pinMode(PIN_FEM_CSD, INPUT);
  delay(1);
  bool kct = digitalRead(PIN_FEM_CSD) == HIGH;
  pinMode(PIN_FEM_CSD, OUTPUT);
  digitalWrite(PIN_FEM_CSD, HIGH);
  if (kct) {
    femName = "KCT8103L";
    pinMode(PIN_FEM_KCT_CTX, OUTPUT);
    digitalWrite(PIN_FEM_KCT_CTX, LOW);
  } else {
    femName = "GC1109";
    pinMode(PIN_FEM_GC_CPS, OUTPUT);
    digitalWrite(PIN_FEM_GC_CPS, LOW);
  }
}

static void applyMode() {
  if (hwMode) {
    check("variablePacketLengthMode", radio.variablePacketLengthMode(MAX_FRAME));
    check("setCRC", radio.setCRC(2, 0xFFFF, 0x8005, false));
  } else {
    check("fixedPacketLengthMode", radio.fixedPacketLengthMode(RAW_LEN));
    check("setCRC", radio.setCRC(0));
  }
}

static bool startRadio() {
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
  // The lowest power: the value is required by the call, and nothing is ever sent.
  int16_t st = radio.beginFSK(FREQ_MHZ, BITRATE_KBPS, DEVIATION_KHZ, RX_BW_KHZ, -9, PREAMBLE_BITS, tcxoVolts, false);
  if (st != RADIOLIB_ERR_NONE) {
    out("# beginFSK with TCXO 1.8 V failed: %d, trying a crystal\n", st);
    tcxoVolts = 0;
    st = radio.beginFSK(FREQ_MHZ, BITRATE_KBPS, DEVIATION_KHZ, RX_BW_KHZ, -9, PREAMBLE_BITS, tcxoVolts, false);
  }
  if (st != RADIOLIB_ERR_NONE) {
    out("# beginFSK failed: %d (-2 means no SX126x answered on SPI)\n", st);
    return false;
  }
  check("setDio2AsRfSwitch", radio.setDio2AsRfSwitch(true));
  check("setDataShaping", radio.setDataShaping(RADIOLIB_SHAPING_0_5));
  check("setEncoding", radio.setEncoding(RADIOLIB_ENCODING_NRZ));
  check("setWhitening", radio.setWhitening(false));
  check("setSyncWord", radio.setSyncWord(SYNC_WORD, sizeof SYNC_WORD));
  check("setRxBoostedGainMode", radio.setRxBoostedGainMode(true));
  applyMode();
  radio.setPacketReceivedAction(onPacket);
  check("startReceive", radio.startReceive());
  return true;
}

static void printConfig() {
  out("# freedo sx1262 probe v2, receive only. front end %s, TCXO %.1f V, %.4f MHz, %.3f kbps, dev %.3f kHz, bw %.1f kHz\n",
      femName, tcxoVolts, FREQ_MHZ, BITRATE_KBPS, DEVIATION_KHZ, RX_BW_KHZ);
}

static void handlePacket() {
  uint8_t buf[256];
  size_t n = hwMode ? radio.getPacketLength() : RAW_LEN;
  if (n > sizeof buf) n = sizeof buf;
  int16_t st = radio.readData(buf, n);
  float rssi = radio.getRSSI();
  // Listen again before printing. A long line blocks for about 20 ms, longer than the gap to
  // an Enhanced ACK that follows a data frame. A restart after the print cleared the flags of
  // a frame that arrived meanwhile and read a stale buffer.
  check("startReceive", radio.startReceive());
  if (hwMode) {
    if (st == RADIOLIB_ERR_NONE) {
      framesOk++;
      printFrame("CRC-OK", rssi, buf, n);
    } else if (st == RADIOLIB_ERR_CRC_MISMATCH) {
      framesBad++;
      printFrame("CRC-BAD", rssi, buf, n);
    } else {
      out("# readData failed: %d\n", st);
    }
  } else if (st != RADIOLIB_ERR_NONE) {
    out("# readData failed: %d\n", st);
  } else {
    uint8_t len = buf[0];
    if (len == 0 || len > MAX_FRAME) {
      framesLen++;
      out("%.3f [LEN-BAD rssi=%.0f mode=RAW len=%u]\n", millis() / 1000.0, rssi, len);
    } else {
      uint16_t sent = (uint16_t)(buf[1 + len] << 8) | buf[2 + len];
      bool ok = cc1101Crc(buf, 1 + len) == sent;
      ok ? framesOk++ : framesBad++;
      printFrame(ok ? "CRC-OK" : "CRC-BAD", rssi, buf + 1, len);
    }
  }
}

void setup() {
  Serial.setTxTimeoutMs(0); // do not stall when no USB host reads
  Serial.setTxBufferSize(4096);
  Serial0.setTxBufferSize(4096);
  Serial.begin(115200);
  Serial0.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  delay(1500);
  frontEndReceive();
  radioOk = startRadio();
  printConfig();
  out("# mode=%s, press PRG to switch between HW and RAW\n", hwMode ? "HW" : "RAW");
}

void loop() {
  if (!radioOk) {
    static uint32_t last = 0;
    if (millis() - last > 5000) {
      last = millis();
      out("# radio not started, see the error above; check the pin map of the board\n");
    }
    delay(10);
    return;
  }
  if (rxFlag) {
    rxFlag = false;
    handlePacket();
  }
  static bool lastButton = true;
  static uint32_t lastEdge = 0;
  bool button = digitalRead(PIN_BUTTON) == HIGH;
  if (button != lastButton && millis() - lastEdge > 50) {
    lastEdge = millis();
    lastButton = button;
    if (!button) {
      hwMode = !hwMode;
      check("standby", radio.standby());
      applyMode();
      check("startReceive", radio.startReceive());
      out("# mode=%s\n", hwMode ? "HW" : "RAW");
    }
  }
  if (millis() - lastStats > 60000) {
    lastStats = millis();
    printConfig();
    out("# stats t=%lus mode=%s crc_ok=%lu crc_bad=%lu len_bad=%lu\n", millis() / 1000, hwMode ? "HW" : "RAW",
        (unsigned long)framesOk, (unsigned long)framesBad, (unsigned long)framesLen);
  }
}
