#!/bin/sh
# Builds the probe in a container. The result is build/sx1262_probe.ino.merged.bin.
# Flashing is a separate step with esptool or esptool-js.
set -eu
cd "$(dirname "$0")"

CLI_VERSION=1.6.0
CORE_VERSION=3.3.11   # the core of the freedo bridge firmware
RADIOLIB_VERSION=7.8.1
CORE_URL=https://espressif.github.io/arduino-esp32/package_esp32_index.json
# FlashSize 8M runs on the 16 MB flash of the V4 too; DIO works on every flash chip.
FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc,USBMode=hwcdc,FlashSize=8M,FlashMode=dio,PSRAM=disabled"

CACHE="${HOME}/.cache/arduino-docker"
mkdir -p "$CACHE/bin" build
if [ ! -x "$CACHE/bin/arduino-cli" ]; then
  curl -fsSL "https://github.com/arduino/arduino-cli/releases/download/v${CLI_VERSION}/arduino-cli_${CLI_VERSION}_Linux_64bit.tar.gz" \
    | tar -xz -C "$CACHE/bin" arduino-cli
fi

docker run --rm -u "$(id -u):$(id -g)" \
  -e HOME=/cache/home \
  -e ARDUINO_DIRECTORIES_DATA=/cache/data \
  -e ARDUINO_DIRECTORIES_USER=/cache/user \
  -e ARDUINO_DIRECTORIES_DOWNLOADS=/cache/staging \
  -e ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS="$CORE_URL" \
  -v "$CACHE:/cache" -v "$PWD:/src/sx1262_probe" -w /src/sx1262_probe \
  python:3.12-slim sh -eu -c "
    cli=/cache/bin/arduino-cli
    \$cli core list | grep -q 'esp32:esp32 *${CORE_VERSION}' || { \$cli core update-index; \$cli core install esp32:esp32@${CORE_VERSION}; }
    \$cli lib list | grep -q '^RadioLib *${RADIOLIB_VERSION}' || \$cli lib install RadioLib@${RADIOLIB_VERSION}
    \$cli compile --fqbn '${FQBN}' --warnings default --output-dir build .
  "
ls -l build/*.merged.bin
sha256sum build/*.merged.bin
md5sum build/*.merged.bin
