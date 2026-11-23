#!/bin/sh
# Builds the bridge firmware in a container. The result is build/sx1262_bridge.ino.merged.bin
# (bootloader, partitions and app, written at offset 0x0). Flashing is a separate step.
#
#   ./build.sh        receive only: no path reaches SetTx, banner (sx1262-rx-<commit>)
#   TX=1 ./build.sh   with transmit, banner (sx1262-<commit>)
set -eu
cd "$(dirname "$0")"

CLI_VERSION=1.6.0
CORE_VERSION=3.3.11   # the core of the freedo bridge firmware
U8G2_VERSION=2.36.19  # the OLED status display
CORE_URL=https://espressif.github.io/arduino-esp32/package_esp32_index.json
# FlashSize 8M runs on the 16 MB flash of the V4 too; DIO works on every flash chip.
FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc,USBMode=hwcdc,FlashSize=8M,FlashMode=dio,PSRAM=disabled"

TX="${TX:-0}"
case "$TX" in
  0) kind=sx1262-rx ;;
  1) kind=sx1262 ;;
  *) echo "TX must be 0 or 1" >&2; exit 1 ;;
esac
rev=$(git rev-parse --short HEAD)
[ -z "$(git status --porcelain -- .)" ] || rev="$rev-dirty"
BUILD="$kind-$rev"

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
  -v "$CACHE:/cache" -v "$PWD:/src/sx1262_bridge" -w /src/sx1262_bridge \
  python:3.12-slim sh -eu -c "
    cli=/cache/bin/arduino-cli
    \$cli core list | grep -q 'esp32:esp32 *${CORE_VERSION}' || { \$cli core update-index; \$cli core install esp32:esp32@${CORE_VERSION}; }
    \$cli lib list | grep -q '^U8g2 *${U8G2_VERSION}' || \$cli lib install U8g2@${U8G2_VERSION}
    \$cli compile --fqbn '${FQBN}' --warnings default --output-dir build \
      --build-property 'compiler.cpp.extra_flags=-DFIRMWARE_BUILD=${BUILD} -DSX1262_TX=${TX}' .
  "
grep -ao 'CC1101BRIDGE v[0-9.]* ([^)]*)' build/sx1262_bridge.ino.bin
ls -l build/sx1262_bridge.ino.merged.bin
sha256sum build/sx1262_bridge.ino.merged.bin
md5sum build/sx1262_bridge.ino.merged.bin
