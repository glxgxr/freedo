# freedo SX1262 bridge

Bridge firmware for the Heltec WiFi LoRa 32 V4 and its clones (ESP32-S3, SX1262 with a 1.8 V
TCXO, KCT8103L or GC1109 front end, native USB). It speaks serial protocol v2.11 of
`firmware/cc1101_bridge` byte for byte, so the unmodified freedo daemon runs on it. See
[PORTING.md](PORTING.md) for the analysis behind it.

## Layout

- `sx1262_bridge.ino`: `cc1101_bridge.ino` with the radio layer replaced. The radio functions keep
  their names (`ccArmRx`, `ccSendFrameEx`, `rxPump`, ...), so `diff` against the CC1101 sketch shows
  only the radio layer.
- `sx1262.h`: the SX1262 driver, with the tado PHY as constants.

## Differences to the CC1101 bridge

- The host still loads its CC1101 register set. The bridge keeps it in a register file and answers
  reads from it: IOCFG0 reads 0x2F after a reset and 0x07 once the host loaded it, as `Configured()`
  expects. PARTNUM and VERSION read 0x00 and 0x14.
- SCAL runs the SX1262 calibration. A failed one reads FSCAL1 = 0x3F, which the host reports as
  `ErrNoSynthLock`. SRX, SIDLE, SFRX and SRES act on the SX1262; STX and the FIFO do nothing.
- FSCTRL0 is stored and not applied: the TCXO needs no crystal correction. FREQEST is always 0.
- The receiver runs continuously and falls back to FS after a frame, so a turnaround to TX or back
  to RX needs no oscillator start-up. A wake train leaves from FS frame after frame; the receiver
  comes back after the Data frame.
- Serial is the native USB CDC. It ignores the line rate.

## Build (in a container)

    ./build.sh          # receive only: no path reaches SetTx
    TX=1 ./build.sh     # with transmit

The result is `build/sx1262_bridge.ino.merged.bin` (bootloader, partitions and app, offset 0x0).
The banner names the build: `CC1101BRIDGE v2.11 (sx1262-rx-<commit>)` without transmit,
`(sx1262-<commit>)` with it, `-dirty` for uncommitted changes. Tools: arduino-cli 1.6.0, ESP32 core
3.3.11 (the core of the CC1101 bridge).

## Flash

    esptool --chip esp32s3 --port <PORT> write-flash 0x0 sx1262_bridge.ino.merged.bin

Or in the browser with https://espressif.github.io/esptool-js/ (Chrome or Edge, address 0x0). If the
board does not answer: hold PRG, press RST, release PRG.

## WiFi link

Without `wifi_secrets.h` the host link is the USB serial. Copy `wifi_secrets.h.example` to
`wifi_secrets.h` (git ignores it) and build again: the bridge joins the WiFi and serves the serial
protocol on TCP port 7820 to one client. A new connection replaces the old one, so only one daemon
may use the bridge. The daemon opens a serial port, so the host turns the TCP link into a pty:

    socat tcp:<bridge>:7820,nodelay,keepalive pty,link=/tmp/bridge,raw,echo=0 &
    freedo --port=/tmp/bridge

When the WiFi stays away for 5 s, the bridge asks for it again. `LINK_ALLOWED_IP` takes the link
only from the host of the daemon. The image holds the WiFi password.

## Display

The OLED lights for a minute at boot and at each press of PRG. Its lines: the build; the WiFi signal
and the last byte of the address, every few seconds in turn with the number of drops and the reason
code of the last one; the host link (USB, WiFi, idle or none); the devices of this coordinator heard
in the last 2 hours; the transmit jobs done and failed; TX or RX only, the uptime and the reason of
the last reset (power, reset, SW, USB, panic, WDT, low V).

## Transmit

Only a `TX=1` build transmits, and only on the tado network of a freedo installation with no
commercial bridge on the air: two coordinators on PAN 0xABCD do not work (PORTING.md, Operation).
