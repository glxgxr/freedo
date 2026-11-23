# Porting the bridge to an ESP32-S3 with an SX1262

Notes for a bridge firmware that runs the unmodified freedo daemon on an ESP32-S3 board with a
Semtech SX1262 in place of the ESP32 + CC1101 of `firmware/cc1101_bridge`. The target board is a
Heltec WiFi LoRa 32 V4 (and clones): ESP32-S3R2, SX1262 with a 1.8 V TCXO, an RF front end with PA
and LNA, native USB. Line references are to the upstream tree at commit 6dba46e.

## State

- Receive works. `firmware/sx1262_probe` receives the tado V3+ network with the PHY below: wake-up
  frames, encrypted Data frames and 29-byte Enhanced ACKs, CRC-OK in hardware, and every Enhanced
  ACK carries a valid software MAC FCS. Tested on one board, front end KCT8103L, next to a running
  commercial bridge.
- Transmit, the Enhanced ACK turnaround and the CSL timing are not tested.

## PHY (probe values)

| Parameter | Value | CC1101 source (`cc1101/cc1101.go` tadoRegs) |
|---|---|---|
| Carrier | 868.3237 MHz | FREQ 0x213276 + CHANNR 26 × 199.951 kHz |
| Modulation | 2-GFSK, BT 0.5 | MDMCFG2 0x13 |
| Bit rate | 49.988 kbps | MDMCFG4/3 |
| Deviation | 25.391 kHz | DEVIATN 0x40 |
| RX bandwidth | 156.2 kHz (SX1262 step; the CC1101 uses 135 kHz) | MDMCFG4 |
| Preamble | 32 bits | MDMCFG1 |
| Sync word | D3 91 D3 91 (SYNC_MODE 3 sends the 16-bit word twice) | SYNC1/0 |
| Packet | variable length, length byte first, at most 140 | PKTCTRL0, PKTLEN 0x8C |
| PHY CRC | CRC-16 poly 0x8005, seed 0xFFFF, not inverted, over length byte and payload | PKTCTRL0 |
| Whitening | off | PKTCTRL0 |

RadioLib 7.8.1: `beginFSK(868.3237, 49.988, 25.391, 156.2, power, 32, 1.8)`, `setDataShaping(RADIOLIB_SHAPING_0_5)`,
`setSyncWord({D3,91,D3,91}, 4)`, `setCRC(2, 0xFFFF, 0x8005, false)`, `setWhitening(false)`,
`variablePacketLengthMode(140)`, `setDio2AsRfSwitch(true)`, `setRxBoostedGainMode(true)`.

Board pins (Meshtastic `variants/esp32s3/heltec_v4/variant.h`): NSS 8, SCK 9, MOSI 10, MISO 11,
NRESET 12, BUSY 13, DIO1 14 (IRQ), DIO2 drives the RF switch, DIO3 the TCXO (1.8 V). Front end:
GPIO7 high powers its LDO; read GPIO2 as input right after: high means KCT8103L (V4.3), low means
GC1109 (V4.2). Then GPIO2 high (enable). KCT8103L: GPIO5 low selects the receive LNA. GC1109: GPIO46
selects the PA (low = bypass). The PA adds up to about 11 dB: keep the conducted power low enough
for the +14 dBm e.r.p. limit of the 868 MHz band (`hardware/radio/README.md` section 7, item 13).

Lesson from the probe: after a received frame, re-enter receive before anything slow (serial
output). A restart after a 20 ms print cleared the IRQ flags of a frame that arrived meanwhile, and
the read returned a stale buffer flagged as CRC-OK.

## What the daemon needs from the bridge

A firmware that answers the serial protocol exactly as `cc1101_bridge.ino` v2.11 does, with a small
fake register file, runs the unmodified daemon.

Start sequence (`cc1101/bridge.go` Open, `cc1101/cc1101.go:177-235` Configure,
`internal/daemon/daemon.go:1877` openConfiguredBridge, `:1950` seedFSCTRL0):

1. Opens 230400 baud (fallback 921600), waits 2 s, PINGs. The banner must read
   `CC1101BRIDGE v2.11 (<build>)`: first word exactly `CC1101BRIDGE`, then `vMAJ.MIN` ≥ v2.11
   (`bridge.go:78-102`). Otherwise the daemon runs on a mock radio and does not retry.
2. CSLQUEUE flush, RESET, 37 × XFER `[addr, val]` (status 0, reply as long as the request, content
   ignored), XFER PATABLE `[0x7E, 0xC0]`.
3. CalibrateSynth: STROBE SIDLE, STROBE SCAL, 8 ms, XFER `[0xA5, 0x00]`: FSCAL1 must not be 0x3F
   (0x3F three times gives ErrNoSynthLock). A usable way to report an SX1262 calibration error.
4. AIRTIME (reply status 0 + 130 bytes), LBT (echo both bytes), CONFIGURED.
5. XFER `[0x0C, seed]` (FSCTRL0, default 0x34), CalibrateSynth again, CSLQUEUE flush (a 0x1C here
   is fatal), then the receive loop.

The daemon does not read VERSION, PARTNUM, MARCSTATE or the FIFOs. `freedo-dev config/regs/tx/emulate`
do; they are bench tools.

Runtime raw access:

- `Configured()` (`cc1101.go:242-258`): XFER `[0x82, 0x00]` must read IOCFG0 = 0x07 after the host
  wrote it, and any other value after a reset or a reboot of the bridge (upstream: 0x2F after its own
  reset, 0x3F chip reset). Called after an expired CSL job and after ~10 s without a CRC-OK frame.
- Hourly synth check and the live receiver calibration / FSCTRL0 scan
  (`internal/radio/health.go:209-245`, `calibration.go:436-597`): FSCTRL0 writes plus CalibrateSynth.
  None can be switched off. **FSCTRL0 trap:** 0x34 is the crystal correction of the CC1101 module
  (+82.5 kHz). Do not apply it to the SX1262 literally: store it (reads echo it) and ignore it, or
  apply only `(int8(v) - 0x34) × 1586.9 Hz`.

Received frame blob (`parseRxBlob` `cc1101.go:682-707`): `[len][payload][rssi][lqi | crc<<7][freqest]`.

- RSSI raw = `int8(2 × (dBm + 74))`; the host only logs it.
- Bit 7 of the LQI byte is CRC-OK and mandatory; the loop drops frames without it.
- **Always append FREQEST.** 0 is fine: the calibration settles on "no correction". Without the
  byte no device frame counts as heard, and after 10 minutes the FSCTRL0 scan starts and defers
  Router Advertisement bursts and bootstrap trains for about 24 minutes, every hour.
- The CSL ACK blob is exactly 32 bytes: `[29][29 bytes][rssi][lqi]`, no FREQEST.

High-level commands the daemon uses: AUTOACK (with `--tx`), RECV (dry-run), TXTRAIN, RABURST,
CSLQUEUE (enqueue/poll/flush), CSLTX (fallback), PAIRWIN, AIRTIME, LBT. GDO and RESPOND are never
sent. The bridge forgets coordinator, queue and pair window after 5 s with no request.

The test double `firmwarePort` in `cc1101/bridge_test.go:35-69` shows how little the register side
needs.

Optional Go changes, only if wanted later: skip `seedFSCTRL0` and start the live radio without a
tuner for a non-CC1101 banner (`daemon.go:1877`, `:1931`); guard the register load and the synth
checks in `Configure`; accept another banner word (`bridge.go:80, 166`).

## The firmware

About 80 % of `cc1101_bridge.ino` does not depend on the chip. CC1101 code: constants `ino:134-190`
and `264-275`; ISRs, SPI, reset and frame send `ino:654-979`; FIFO receive assembler `ino:1061-1155`;
`ccWatchReset` `1976-1986`; XFER/STROBE/GDO/RESET `2359-2396`; `setup()` wiring `2779-2799`; the
`CRC_OK_BIT` checks `1016, 1144, 1546, 1550` and the FREQEST read `1128`.

Radio API that the rest of the firmware calls, to reimplement for the SX1262:

| Upstream | Purpose |
|---|---|
| `manualReset()`, `ccQuietIfReset()` | chip reset; detect a chip that reset by itself, clear `configured` |
| `ccArmRx()`, `ccEnsureRx()`, `rxFlush()` | (re)enter receive; keep a frame in flight; set `rxValidUs` |
| `ccSendFrameEx(p, n, fromRx)`, `ccSendFrame` | blocking send, back to receive afterwards, `airCount`; `fromRx` = carrier-sense gate |
| `rxDuePump`, `rxPump`, `rxTake(dst, &len, &stampUs, &freqest)`, `rxAssembling`, `rxReset`, `rxCrcOk` | frame ready?, take the blob with the end-of-frame timestamp |
| `lbtListenOnce`, `lbtClear` | RSSI plus sync/preamble detect for listen before talk |
| `ccStrobe(CC_SIDLE)` at stream end (`2077`) | idle |

Differences to handle:

- The CC1101 returns to receive by itself after TX and after RX (MCSM1 0x3F). The SX1262 does not:
  re-enter receive after every TX.
- The carrier-sense gate of the CC1101 send path is hardware; on the SX1262 it is a software RSSI /
  preamble check before TX.
- The 64-byte FIFO streaming, the errata handling and the GDO2 threshold go away (256-byte buffer).
- GDO0 end-of-frame timestamp → DIO1 RxDone IRQ timestamp. CSL anchors depend on it.
- The SX1262 has a BUSY line to wait on before every SPI command.
- No GFSK frequency-error readout: FREQEST is 0.
- ESP32-S3: GPIO 22-25 do not exist; the serial port is native USB CDC (`Serial.setClockSource`
  and the 230400/921600 baud rates do not apply the same way; check). Core pinning, the task
  watchdog and `esp_timer_get_time()` work as on the ESP32.

Frames built in the firmware (keep byte-exact):

- Enhanced ACK, 29 bytes (`ccAutoAck` `ino:1015-1041`):
  `42 EE seq | dst EUI | coordinator EUI | 04 0D phaseLo phaseHi 35 0C | 80 3F | FCS`.
  FCS = CRC-16/CCITT reflected (0x8408), init 0, little-endian (`macFCS` `ino:562-571`).
- Wake-up frame, 12 bytes (`cslSendWake` `ino:1680-1681`):
  `25 seq CD AB dst0 dst1 82 0E rzLo rzHi 80 3F`, no software FCS.
- Enhanced Beacon (pairing) with FCS and AES-CCM* (`ino:1270-1282`).

## Timing (`docs/PROTOCOL.md`)

| Item | Requirement |
|---|---|
| Enhanced ACK | 0 to 1 ms after the end of the uplink; the device waits only in that window (P:2070-2073). Upstream computes ~0.4 ms. Must run on the MCU. |
| Wake-up train | one frame every 4.0 ms (12.5 units of 320 µs); short train 248→0 in 21 frames, bootstrap 3125→0 in ~254 frames over ~1 s; broadcast trains to `ffff` |
| Data frame | 12 ms after the last wake-up frame (commercial 9-24 ms) |
| Device ACK to our Data frame | ~192 µs after our TX ends; we must already listen |
| Beacon answer (pairing) | 3968 µs after the Beacon Request (window 2.4-7.4 ms) |
| CSL | period 1.000 s, device clocks -6 to +18 ppm |

## Operation

- PAN 0xABCD and channel 26 are fixed: a freedo network shares them with the commercial bridge and
  differs only by network key and coordinator EUI. The daemon stops transmitting for the rest of the
  run when it hears a frame that only the commercial bridge sends (`bridgePresent`). Mixed operation
  with the commercial bridge is not supported. One coordinator per plant.
- Dry-run (no `--tx`) receives only and sends nothing, not even Enhanced ACKs. Frames of a network
  with another key are logged as `mic-fail`; stats and the unhandled log still show traffic.
- Every device pairs from scratch: hold its button, pair window, declare it, `POST /api/pair`
  (`docs/OPERATION.md` 777-834). Going back means pairing it to the commercial bridge again.
- A plant of valves only needs 0.12.0-beta.11 or later with `no_boiler: true`.
- Without the Home Assistant Supervisor: container `ghcr.io/cappyt/freedo` (see
  `docker-compose.example.yml`), API token, port 8080, data in `/data`, set `TZ`; the integration
  comes from HACS and connects with host, port and token.
