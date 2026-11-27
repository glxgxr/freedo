# freedo SX1262 probe

Receive test: does an SX1262 (Heltec WiFi LoRa 32 V4 and its clones) hear the 868 MHz network of
tado V3+? The sketch never transmits. It only prints what it receives. The radio settings come
from `tadoRegs` in freedo `cc1101/cc1101.go`.

## Build (in a container)

    ./build.sh

The result is `build/sx1262_probe.ino.merged.bin` (bootloader, partitions and app in one file,
offset 0x0). Tools: arduino-cli 1.6.0, ESP32 core 3.3.11 (as for the freedo bridge firmware) and
RadioLib 7.8.1.

## Flash

    esptool --chip esp32s3 --port <PORT> write-flash 0x0 build/sx1262_probe.ino.merged.bin

Or in the browser with https://espressif.github.io/esptool-js/ (Chrome or Edge, address 0x0). If the
board does not answer: hold PRG, press RST, release PRG.

## Output

Serial monitor on the USB port, 115200 baud. The output also goes to UART0.

- `# ...` lines are status messages. Every 60 s the configuration and the counters follow.
- A received frame: `<seconds> [CRC-OK rssi=-71 mode=HW] <hex>`. The format is the one of
  `freedo-dev rx`, so the freedo tools read it.
- PRG switches between `HW` (the SX1262 checks the CRC) and `RAW` (the sketch checks it).
- `beginFSK failed: -2`: no SX1262 answers on the SPI bus, the pins of the board differ.

A capture holds the addresses of your own devices. Do not post it as it is: mask them first (see
freedo `CONTRIBUTING.md`).
