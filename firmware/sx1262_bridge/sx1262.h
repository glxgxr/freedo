// SX1262 driver of the bridge: the SX126x commands the bridge uses, for the Heltec WiFi LoRa 32 V4
// and its clones (ESP32-S3, SX1262 with a 1.8 V TCXO, RF front end). Only the radio task calls it.
//
// The chip receives continuously and falls back to FS after a frame, so the TCXO and the
// synthesizer keep running: a turnaround to TX or back to RX costs no oscillator start-up.

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"

// Heltec V4 pin map, from Meshtastic variants/esp32s3/heltec_v4/variant.h.
static const int PIN_SX_NSS = 8;
static const int PIN_SX_SCK = 9;
static const int PIN_SX_MOSI = 10;
static const int PIN_SX_MISO = 11;
static const int PIN_SX_RESET = 12;
static const int PIN_SX_BUSY = 13;
static const int PIN_SX_DIO1 = 14;
static const int PIN_VEXT = 36;         // active low
static const int PIN_FEM_POWER = 7;     // LDO of the RF front end
static const int PIN_FEM_CSD = 2;       // front end enable, read first to tell the two front ends apart
static const int PIN_FEM_KCT_CTX = 5;   // KCT8103L (V4.3): low selects the receive LNA
static const int PIN_FEM_GC_CPS = 46;   // GC1109 (V4.2): low keeps the PA in bypass

// A build without -DSX1262_TX=1 never sends SetTx: sxTransmit returns before it touches the chip.
#ifndef SX1262_TX
#define SX1262_TX 0
#endif
static const bool SX_TX_ENABLED = SX1262_TX != 0;

// tado PHY of tadoRegs in cc1101/cc1101.go (a CC1101 with a 26 MHz crystal), in SX1262 units with
// its 32 MHz reference: frequency and deviation f * 2^25 / 32 MHz, bit rate 32 * 32 MHz / rate.
static const uint32_t SX_RF_FREQ = 0x36452DC0;  // 868.3237 MHz: FREQ 0x213276 + CHANNR 26 * 199.951 kHz
static const uint32_t SX_BIT_RATE = 0x005005;   // 49.988 kbps: DRATE_E 10, DRATE_M 248
static const uint32_t SX_FDEV = 0x006800;       // 25.391 kHz: DEVIATN 0x40
static const uint8_t SX_SHAPING = 0x09;         // Gaussian, BT 0.5
static const uint8_t SX_RX_BW = 0x1A;           // 156.2 kHz; the CC1101 uses 135, the SX1262 steps are 117.3 and 156.2
static const uint16_t SX_PREAMBLE_BITS = 32;    // MDMCFG1: 4 bytes
// 16 bits. With 32, the whole preamble, the receiver caught the Enhanced ACK that follows a Data
// frame within 0.4 ms for 64 % of the Data frames, with 16 for 86 % (live network, 10 minutes each).
static const uint8_t SX_PREAMBLE_DETECT = 0x05;
static const uint8_t SX_SYNC_BITS = 32;
static const uint8_t SX_SYNC_WORD[4] = {0xD3, 0x91, 0xD3, 0x91};  // SYNC_MODE 3 sends the 16-bit word twice
static const uint8_t SX_CRC_2_BYTE = 0x02;      // with the seed and polynomial below: CRC-16 0x8005, seed 0xFFFF
static const uint8_t SX_CRC_SEED[2] = {0xFF, 0xFF};
static const uint8_t SX_CRC_POLY[2] = {0x80, 0x05};
static const uint8_t SX_PACKET_VARIABLE = 0x01;  // length byte first, as PKTCTRL0 0x05

// The lowest output power of the SX1262 PA, -9 dBm (paDutyCycle 2, hpMax 2, power -5). The
// KCT8103L adds about 13 dB (the gain table of the Meshtastic firmware), so about +4 dBm leave the
// antenna connector; the band allows +14 dBm e.r.p.
static const uint8_t SX_PA_CONFIG[4] = {0x02, 0x02, 0x00, 0x01};
static const int8_t SX_TX_POWER = -5;
static const uint8_t SX_PA_RAMP_40US = 0x02;
static const uint8_t SX_OCP_60MA = 0x18;

static const uint8_t SX_TCXO_1V8 = 0x02;
static const uint32_t SX_TCXO_STARTUP = 320;  // 5 ms in 15.625 us steps
static const uint8_t SX_CAL_ALL = 0x7F;
static const uint8_t SX_CAL_IMAGE_863_870[2] = {0xD7, 0xDB};
static const uint8_t SX_RX_GAIN_BOOSTED = 0x96;
static const uint8_t SX_TX_CLAMP_FIX = 0x1E;  // datasheet 15.2, better resistance to antenna mismatch

static const uint32_t SX_SPI_HZ = 8000000;
// The longest BUSY phase is a calibration, which starts the TCXO first.
static const uint32_t SX_BUSY_TMO_US = 30000;
static const uint8_t SX_RESET_TRIES = 10;

enum : uint8_t {
  SX_CLEAR_IRQ = 0x02,
  SX_CLEAR_DEVICE_ERRORS = 0x07,
  SX_SET_DIO_IRQ = 0x08,
  SX_WRITE_REGISTER = 0x0D,
  SX_WRITE_BUFFER = 0x0E,
  SX_GET_IRQ = 0x12,
  SX_GET_RX_BUFFER_STATUS = 0x13,
  SX_GET_PACKET_STATUS = 0x14,
  SX_GET_RSSI_INST = 0x15,
  SX_GET_DEVICE_ERRORS = 0x17,
  SX_READ_REGISTER = 0x1D,
  SX_READ_BUFFER = 0x1E,
  SX_SET_STANDBY = 0x80,
  SX_SET_RX = 0x82,
  SX_SET_TX = 0x83,
  SX_SET_RF_FREQ = 0x86,
  SX_CALIBRATE = 0x89,
  SX_SET_PACKET_TYPE = 0x8A,
  SX_SET_MOD_PARAMS = 0x8B,
  SX_SET_PACKET_PARAMS = 0x8C,
  SX_SET_TX_PARAMS = 0x8E,
  SX_SET_BUFFER_BASE = 0x8F,
  SX_SET_FALLBACK = 0x93,
  SX_SET_PA_CONFIG = 0x95,
  SX_SET_REGULATOR = 0x96,
  SX_SET_DIO3_TCXO = 0x97,
  SX_CALIBRATE_IMAGE = 0x98,
  SX_SET_DIO2_RF_SWITCH = 0x9D,
  SX_GET_STATUS = 0xC0,
  SX_SET_FS = 0xC1
};

static const uint16_t SX_REG_CRC_SEED = 0x06BC;
static const uint16_t SX_REG_CRC_POLY = 0x06BE;
static const uint16_t SX_REG_SYNC_WORD = 0x06C0;
static const uint16_t SX_REG_RX_GAIN = 0x08AC;
static const uint16_t SX_REG_TX_CLAMP = 0x08D8;
static const uint16_t SX_REG_OCP = 0x08E7;

static const uint16_t SX_IRQ_TX_DONE = 1U << 0;
static const uint16_t SX_IRQ_RX_DONE = 1U << 1;
static const uint16_t SX_IRQ_PREAMBLE = 1U << 2;
static const uint16_t SX_IRQ_SYNC_WORD = 1U << 3;
static const uint16_t SX_IRQ_CRC_ERR = 1U << 6;
static const uint16_t SX_IRQ_TIMEOUT = 1U << 9;
static const uint16_t SX_IRQ_ALL = 0x43FF;
static const uint16_t SX_IRQ_RX_ALL = SX_IRQ_RX_DONE | SX_IRQ_PREAMBLE | SX_IRQ_SYNC_WORD | SX_IRQ_CRC_ERR | SX_IRQ_TIMEOUT;

// GetStatus chip modes.
static const uint8_t SX_MODE_STBY_RC = 2;
static const uint8_t SX_MODE_STBY_XOSC = 3;
static const uint8_t SX_MODE_FS = 4;
static const uint8_t SX_MODE_RX = 5;
static const uint8_t SX_MODE_TX = 6;

static const uint8_t SX_FALLBACK_FS = 0x40;
static const uint8_t SX_STANDBY_RC = 0x00;
static const uint8_t SX_STANDBY_XOSC = 0x01;

// Device errors that leave the chip unusable. RC64K_CALIB_ERR is left out: the 64 kHz RC clocks
// only the sleep and duty-cycle timers, which the bridge does not use.
static const uint16_t SX_ERR_FATAL = 0x007E;

static SPISettings sxSpi(SX_SPI_HZ, MSBFIRST, SPI_MODE0);
static bool sxFault;  // BUSY stayed high past SX_BUSY_TMO_US since the last sxInit
static bool sxKct;    // the front end is a KCT8103L

static inline uint64_t sxNowUs() {
  return (uint64_t)esp_timer_get_time();
}

static bool sxWaitBusy() {
  if (digitalRead(PIN_SX_BUSY) == LOW) return true;
  uint64_t end = sxNowUs() + SX_BUSY_TMO_US;
  while (digitalRead(PIN_SX_BUSY) == HIGH) {
    if (sxNowUs() > end) {
      sxFault = true;
      return false;
    }
  }
  return true;
}

// One SPI transaction of n bytes in place: the request goes out, the reply comes back in p.
static bool sxXfer(uint8_t *p, size_t n) {
  if (!sxWaitBusy()) return false;
  SPI.beginTransaction(sxSpi);
  digitalWrite(PIN_SX_NSS, LOW);
  SPI.transfer(p, n);
  digitalWrite(PIN_SX_NSS, HIGH);
  SPI.endTransaction();
  return true;
}

static bool sxCmd(uint8_t op, const uint8_t *args, uint8_t n) {
  uint8_t b[1 + 9];
  if (n > sizeof(b) - 1) return false;
  b[0] = op;
  if (n) memcpy(b + 1, args, n);
  return sxXfer(b, 1 + (size_t)n);
}

static bool sxCmd1(uint8_t op, uint8_t arg) {
  return sxCmd(op, &arg, 1);
}

// A Get command: opcode, one status byte, then n reply bytes.
static bool sxGet(uint8_t op, uint8_t *out, uint8_t n) {
  uint8_t b[2 + 4] = {op};
  if (n > sizeof(b) - 2) return false;
  if (!sxXfer(b, 2 + (size_t)n)) return false;
  memcpy(out, b + 2, n);
  return true;
}

static bool sxWriteReg(uint16_t addr, const uint8_t *v, uint8_t n) {
  uint8_t b[3 + 8] = {SX_WRITE_REGISTER, (uint8_t)(addr >> 8), (uint8_t)addr};
  if (n > sizeof(b) - 3) return false;
  memcpy(b + 3, v, n);
  return sxXfer(b, 3 + (size_t)n);
}

static bool sxReadReg(uint16_t addr, uint8_t *v, uint8_t n) {
  uint8_t b[4 + 8] = {SX_READ_REGISTER, (uint8_t)(addr >> 8), (uint8_t)addr};
  if (n > sizeof(b) - 4) return false;
  if (!sxXfer(b, 4 + (size_t)n)) return false;
  memcpy(v, b + 4, n);
  return true;
}

static bool sxWriteBuffer(const uint8_t *p, uint8_t n) {
  uint8_t b[2 + 255] = {SX_WRITE_BUFFER, 0x00};
  memcpy(b + 2, p, n);
  return sxXfer(b, 2 + (size_t)n);
}

static bool sxReadBuffer(uint8_t offset, uint8_t *p, uint8_t n) {
  uint8_t b[3 + 255] = {SX_READ_BUFFER, offset};
  if (!sxXfer(b, 3 + (size_t)n)) return false;
  memcpy(p, b + 3, n);
  return true;
}

// The chip mode from GetStatus, 0 when the chip does not answer.
static uint8_t sxMode() {
  uint8_t b[2] = {SX_GET_STATUS, 0x00};
  if (!sxXfer(b, 2)) return 0;
  return (b[1] >> 4) & 0x07;
}

static uint16_t sxIrq() {
  uint8_t r[2] = {0, 0};
  if (!sxGet(SX_GET_IRQ, r, 2)) return 0;
  return ((uint16_t)r[0] << 8) | r[1];
}

static void sxClearIrq(uint16_t mask) {
  const uint8_t a[2] = {(uint8_t)(mask >> 8), (uint8_t)mask};
  sxCmd(SX_CLEAR_IRQ, a, 2);
}

// Instantaneous RSSI in -0.5 dBm steps: dBm = -raw / 2.
static uint8_t sxRssiInst() {
  uint8_t r = 0xFF;
  sxGet(SX_GET_RSSI_INST, &r, 1);
  return r;
}

static void sxStandby(uint8_t mode) {
  sxCmd1(SX_SET_STANDBY, mode);
}

// FS stops a receiver and keeps the synthesizer locked.
static void sxSetFs() {
  sxCmd(SX_SET_FS, nullptr, 0);
}

// In receive the payload length is the longest frame the chip accepts, in transmit the length
// of the frame. The chip sends the length byte first and appends the CRC.
static void sxPacketParams(uint8_t payloadLen) {
  const uint8_t a[9] = {(uint8_t)(SX_PREAMBLE_BITS >> 8), (uint8_t)SX_PREAMBLE_BITS, SX_PREAMBLE_DETECT,
                        SX_SYNC_BITS, 0x00, SX_PACKET_VARIABLE, payloadLen, SX_CRC_2_BYTE, 0x00};
  sxCmd(SX_SET_PACKET_PARAMS, a, sizeof(a));
}

// Continuous receive: the chip stays in RX after every frame.
static void sxRxStart(uint8_t maxLen) {
  sxPacketParams(maxLen);
  sxClearIrq(SX_IRQ_ALL);
  const uint8_t t[3] = {0xFF, 0xFF, 0xFF};
  sxCmd(SX_SET_RX, t, 3);
}

// Take the frame the receiver finished. False when no RxDone is pending. A frame with a
// length outside 1..maxLen is dropped and reads as len 0. rssiRaw is RssiSync, dBm = -raw / 2.
static bool sxTakeFrame(uint8_t *dst, uint8_t maxLen, uint8_t *len, bool *crcOk, uint8_t *rssiRaw) {
  uint16_t irq = sxIrq();
  if ((irq & SX_IRQ_RX_DONE) == 0) return false;
  uint8_t bs[2] = {0, 0};  // payload length, start pointer
  uint8_t ps[3] = {0, 0xFF, 0xFF};  // rx status, rssi sync, rssi avg
  bool ok = sxGet(SX_GET_RX_BUFFER_STATUS, bs, 2) && sxGet(SX_GET_PACKET_STATUS, ps, 3);
  uint8_t n = bs[0];
  if (!ok || n == 0 || n > maxLen || !sxReadBuffer(bs[1], dst, n)) n = 0;
  sxClearIrq(SX_IRQ_RX_ALL);
  *len = n;
  *crcOk = (irq & SX_IRQ_CRC_ERR) == 0;
  *rssiRaw = ps[1];
  return true;
}

// The front end. The V4.2 and V4.3 carry different ones; Meshtastic tells them apart by the
// level of the enable pin right after the front end is powered. Ends with the receive path.
static void sxFrontEndInit() {
  const int pins[] = {PIN_FEM_POWER, PIN_FEM_CSD, PIN_FEM_KCT_CTX, PIN_FEM_GC_CPS};
  for (int p : pins) gpio_hold_dis((gpio_num_t)p);
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  pinMode(PIN_FEM_POWER, OUTPUT);
  digitalWrite(PIN_FEM_POWER, HIGH);
  delay(1);
  pinMode(PIN_FEM_CSD, INPUT);
  delay(1);
  sxKct = digitalRead(PIN_FEM_CSD) == HIGH;
  pinMode(PIN_FEM_CSD, OUTPUT);
  digitalWrite(PIN_FEM_CSD, HIGH);
  if (sxKct) {
    pinMode(PIN_FEM_KCT_CTX, OUTPUT);
    digitalWrite(PIN_FEM_KCT_CTX, LOW);
  } else {
    pinMode(PIN_FEM_GC_CPS, OUTPUT);
    digitalWrite(PIN_FEM_GC_CPS, LOW);
  }
}

// The KCT8103L transmits with CTX high (its CPS follows DIO2) and receives through its LNA with
// CTX low; it has no transmit bypass. The GC1109 transmits through its bypass with CPS low.
static void sxFrontEndTx(bool on) {
  if (sxKct) digitalWrite(PIN_FEM_KCT_CTX, on ? HIGH : LOW);
}

// Send one frame and wait for TxDone. The chip falls back to FS. It never leaves RX or FS in a
// receive-only build.
static bool sxTransmit(const uint8_t *p, uint8_t n, uint32_t tmoUs) {
  if (!SX_TX_ENABLED) return false;
  sxSetFs();
  sxWriteBuffer(p, n);
  sxPacketParams(n);
  sxClearIrq(SX_IRQ_ALL);
  sxFrontEndTx(true);
  const uint8_t t[3] = {0x00, 0x00, 0x00};  // single frame, no timeout
  sxCmd(SX_SET_TX, t, 3);
  uint64_t end = sxNowUs() + tmoUs;
  bool done = false;
  while (!done && sxNowUs() < end && !sxFault) done = (sxIrq() & SX_IRQ_TX_DONE) != 0;
  if (!done) sxSetFs();  // aborts a transmission that is still on air
  sxFrontEndTx(false);
  return done;
}

// Calibrate every block and the image of the 863-870 MHz band. True when the chip reports no
// error that matters. Ends in STDBY_XOSC.
static bool sxCalibrate() {
  sxStandby(SX_STANDBY_RC);  // the calibrations run in STDBY_RC
  const uint8_t zero[2] = {0, 0};
  sxCmd(SX_CLEAR_DEVICE_ERRORS, zero, 2);
  sxCmd1(SX_CALIBRATE, SX_CAL_ALL);
  sxCmd(SX_CALIBRATE_IMAGE, SX_CAL_IMAGE_863_870, 2);
  uint8_t e[2] = {0xFF, 0xFF};
  bool ok = sxGet(SX_GET_DEVICE_ERRORS, e, 2);
  sxStandby(SX_STANDBY_XOSC);
  uint16_t err = ((uint16_t)e[0] << 8) | e[1];
  return ok && !sxFault && (err & SX_ERR_FATAL) == 0;
}

// True when the chip lost the sync word this driver writes: it reset by itself. Two reads agree,
// so one corrupt read does not count.
static bool sxLostConfig() {
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t v = 0;
    if (sxReadReg(SX_REG_SYNC_WORD, &v, 1) && v == SX_SYNC_WORD[0]) return false;
  }
  return true;
}

// Reset the chip and load the tado PHY, in STDBY_XOSC. False when no SX1262 answers, a
// calibration fails or the register read back differs.
static bool sxInit(uint8_t rxMaxLen) {
  sxFault = false;
  digitalWrite(PIN_SX_RESET, LOW);
  delay(1);
  digitalWrite(PIN_SX_RESET, HIGH);
  // BUSY rises a moment after the reset and the chip ignores commands until its start-up
  // calibration ends, so ask until it reports STDBY_RC.
  bool up = false;
  for (uint8_t i = 0; i < SX_RESET_TRIES && !up; i++) {
    delay(1);
    sxStandby(SX_STANDBY_RC);
    up = sxMode() == SX_MODE_STBY_RC;
  }
  if (!up) return false;
  sxFault = false;
  const uint8_t tcxo[4] = {SX_TCXO_1V8, (uint8_t)(SX_TCXO_STARTUP >> 16), (uint8_t)(SX_TCXO_STARTUP >> 8),
                           (uint8_t)SX_TCXO_STARTUP};
  sxCmd(SX_SET_DIO3_TCXO, tcxo, 4);
  sxCmd1(SX_SET_REGULATOR, 0x01);  // DC-DC
  sxCmd1(SX_SET_PACKET_TYPE, 0x00);  // GFSK
  const uint8_t base[2] = {0x00, 0x00};
  sxCmd(SX_SET_BUFFER_BASE, base, 2);
  sxCmd1(SX_SET_FALLBACK, SX_FALLBACK_FS);
  // Every flag the bridge polls is latched; only RxDone drives DIO1, as GDO0 did on the CC1101.
  const uint16_t irqMask = SX_IRQ_TX_DONE | SX_IRQ_RX_ALL;
  const uint8_t dio[8] = {(uint8_t)(irqMask >> 8), (uint8_t)irqMask, 0x00, (uint8_t)SX_IRQ_RX_DONE, 0, 0, 0, 0};
  sxCmd(SX_SET_DIO_IRQ, dio, sizeof(dio));
  sxClearIrq(SX_IRQ_ALL);
  sxCmd1(SX_SET_DIO2_RF_SWITCH, 0x01);
  const uint8_t freq[4] = {(uint8_t)(SX_RF_FREQ >> 24), (uint8_t)(SX_RF_FREQ >> 16), (uint8_t)(SX_RF_FREQ >> 8),
                           (uint8_t)SX_RF_FREQ};
  sxCmd(SX_SET_RF_FREQ, freq, 4);
  const uint8_t mod[8] = {(uint8_t)(SX_BIT_RATE >> 16), (uint8_t)(SX_BIT_RATE >> 8), (uint8_t)SX_BIT_RATE,
                          SX_SHAPING, SX_RX_BW,
                          (uint8_t)(SX_FDEV >> 16), (uint8_t)(SX_FDEV >> 8), (uint8_t)SX_FDEV};
  sxCmd(SX_SET_MOD_PARAMS, mod, sizeof(mod));
  sxPacketParams(rxMaxLen);
  sxWriteReg(SX_REG_SYNC_WORD, SX_SYNC_WORD, sizeof(SX_SYNC_WORD));
  sxWriteReg(SX_REG_CRC_SEED, SX_CRC_SEED, 2);
  sxWriteReg(SX_REG_CRC_POLY, SX_CRC_POLY, 2);
  uint8_t clamp = 0;
  sxReadReg(SX_REG_TX_CLAMP, &clamp, 1);
  clamp |= SX_TX_CLAMP_FIX;
  sxWriteReg(SX_REG_TX_CLAMP, &clamp, 1);
  sxCmd(SX_SET_PA_CONFIG, SX_PA_CONFIG, sizeof(SX_PA_CONFIG));
  const uint8_t txp[2] = {(uint8_t)SX_TX_POWER, SX_PA_RAMP_40US};
  sxCmd(SX_SET_TX_PARAMS, txp, 2);
  sxWriteReg(SX_REG_OCP, &SX_OCP_60MA, 1);  // after SetPaConfig, which resets it
  sxWriteReg(SX_REG_RX_GAIN, &SX_RX_GAIN_BOOSTED, 1);
  bool ok = sxCalibrate();
  uint8_t sync[sizeof(SX_SYNC_WORD)] = {0};
  ok = ok && sxReadReg(SX_REG_SYNC_WORD, sync, sizeof(sync)) && memcmp(sync, SX_SYNC_WORD, sizeof(sync)) == 0;
  return ok && !sxFault;
}

// Pins and SPI, once at boot. The front end comes up on its receive path.
static void sxBegin() {
  pinMode(PIN_SX_NSS, OUTPUT);
  digitalWrite(PIN_SX_NSS, HIGH);
  pinMode(PIN_SX_RESET, OUTPUT);
  digitalWrite(PIN_SX_RESET, HIGH);
  pinMode(PIN_SX_BUSY, INPUT);
  pinMode(PIN_SX_DIO1, INPUT);
  sxFrontEndInit();
  SPI.begin(PIN_SX_SCK, PIN_SX_MISO, PIN_SX_MOSI, PIN_SX_NSS);
}
