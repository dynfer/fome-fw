#!/bin/bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
FW_DIR=$(cd "$SCRIPT_DIR/../../.." && pwd -P)
REPO_DIR=$(cd "$FW_DIR/.." && pwd -P)
G0_DIR="$FW_DIR/ext/g0_firmware"

export USE_OPENBLT=yes

if git -C "$REPO_DIR" ls-files --error-unmatch firmware/ext/g0_firmware >/dev/null 2>&1; then
  git -C "$REPO_DIR" submodule sync firmware/ext/g0_firmware
  if [ "${UPDATE_G0_FIRMWARE-yes}" = "yes" ]; then
    git -C "$REPO_DIR" submodule update --init --recursive --remote firmware/ext/g0_firmware
  else
    git -C "$REPO_DIR" submodule update --init --recursive firmware/ext/g0_firmware
  fi
elif [ "${UPDATE_G0_FIRMWARE-yes}" = "yes" ] && [[ -d "$G0_DIR/.git" || -f "$G0_DIR/.git" ]]; then
  git -C "$G0_DIR" pull --ff-only
fi

git -C "$REPO_DIR" submodule update --init --depth=1 firmware/ext/build-tools

UNAME_SM=$(uname -sm)
case "$UNAME_SM" in
  "Darwin "*)
    COMPILER_PLATFORM=arm-gnu-toolchain-11.3.rel1-darwin-x86_64-arm-none-eabi
    JOBS=$(sysctl -n hw.ncpu)
    ;;
  "Linux x86_64")
    COMPILER_PLATFORM=arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi
    JOBS=$(getconf _NPROCESSORS_ONLN 2>/dev/null || nproc)
    ;;
  *)
    echo "Unsupported compiler platform: $UNAME_SM"
    exit 1
    ;;
esac

G0_TRGT="$FW_DIR/ext/build-tools/$COMPILER_PLATFORM/bin/arm-none-eabi-"
[ -x "${G0_TRGT}g++" ] || { echo "Compiler not found at ${G0_TRGT}g++"; exit 1; }
[ -d "$G0_DIR" ] || { echo "Missing G0 firmware checkout at $G0_DIR"; exit 1; }

make -C "$G0_DIR" clean
make -C "$G0_DIR" -j"${JOBS:-1}" TRGT="$G0_TRGT" USE_OPT="-Os -ggdb -fomit-frame-pointer -falign-functions=16 --specs=nosys.specs" for_fome_image

G0_IMAGE_HEADER="$G0_DIR/for_fome/g0_firmware_image.h"
G0_SPI_APP_PROTOCOL="$G0_DIR/spi_app_protocol.cpp"
if [ ! -f "$G0_SPI_APP_PROTOCOL" ]; then
  G0_SPI_APP_PROTOCOL="$G0_DIR/source/spi_app_protocol.cpp"
fi
G0_APP_VERSION=$(sed -nE 's/.*(APP_VERSION|appVersion) = ([0-9]+)U;.*/\2/p' "$G0_SPI_APP_PROTOCOL" | head -n1)
[ -n "$G0_APP_VERSION" ] || { echo "Unable to determine G0 app version"; exit 1; }
if ! grep -q 'build_g0_extension_version' "$G0_IMAGE_HEADER"; then
  printf '\nstatic const unsigned int build_g0_extension_version = %sU;\n' "$G0_APP_VERSION" >> "$G0_IMAGE_HEADER"
fi

cd "$SCRIPT_DIR"
bash ../common_make.sh atlas ARCH_STM32H7
