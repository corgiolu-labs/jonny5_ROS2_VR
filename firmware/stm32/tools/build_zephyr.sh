#!/usr/bin/env bash
# Build the JONNY5 firmware with plain Zephyr + CMake (no PlatformIO), on Linux.
# Used by CI and handy as a cross-check of the PlatformIO build.
#
#   firmware/stm32/tools/build_zephyr.sh [board ...]      (default: nucleo_g474re nucleo_f446re)
#
# Needs: cmake, ninja, gcc-arm-none-eabi, python3 with pyelftools pyyaml
# jsonschema pykwalify packaging. Zephyr + the needed modules are fetched
# (shallow) into $J5_ZEPHYR_CACHE (default ~/.cache/j5-zephyr) at the revisions
# pinned by the Zephyr release's west.yml.
set -euo pipefail

ZEPHYR_VERSION="${J5_ZEPHYR_VERSION:-v4.2.1}"   # PlatformIO framework-zephyr 3.40201
CACHE="${J5_ZEPHYR_CACHE:-$HOME/.cache/j5-zephyr}/$ZEPHYR_VERSION"
APP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${J5_BUILD_DIR:-$APP/build-zephyr}"
BOARDS=("${@:-nucleo_g474re nucleo_f446re}")
[ $# -eq 0 ] && BOARDS=(nucleo_g474re nucleo_f446re)

fetch() {  # url dir revision
  [ -d "$2/.git" ] && return 0
  mkdir -p "$2"
  git -C "$2" init -q
  git -C "$2" remote add origin "$1"
  git -C "$2" fetch -q --depth 1 origin "$3"
  git -C "$2" checkout -q FETCH_HEAD
}

rev() {  # west.yml revision of a project
  awk -v n="$1" '$0 ~ "- name: "n"$" {f=1; next} f && /revision:/ {print $2; exit} f && /- name:/ {exit}' \
    "$CACHE/zephyr/west.yml"
}

mkdir -p "$CACHE"
if [ ! -d "$CACHE/zephyr/.git" ]; then
  git clone -q --depth 1 --branch "$ZEPHYR_VERSION" https://github.com/zephyrproject-rtos/zephyr "$CACHE/zephyr"
fi
fetch https://github.com/zephyrproject-rtos/hal_stm32 "$CACHE/modules/hal/stm32" "$(rev hal_stm32)"
fetch https://github.com/zephyrproject-rtos/cmsis "$CACHE/modules/hal/cmsis" "$(rev cmsis)"
fetch https://github.com/zephyrproject-rtos/CMSIS_6 "$CACHE/modules/hal/cmsis_6" "$(rev cmsis_6)"
fetch https://github.com/zephyrproject-rtos/picolibc "$CACHE/modules/lib/picolibc" "$(rev picolibc)"

export ZEPHYR_BASE="$CACHE/zephyr"
export ZEPHYR_TOOLCHAIN_VARIANT="${ZEPHYR_TOOLCHAIN_VARIANT:-gnuarmemb}"
export GNUARMEMB_TOOLCHAIN_PATH="${GNUARMEMB_TOOLCHAIN_PATH:-/usr}"
MODULES="$CACHE/modules/hal/stm32;$CACHE/modules/hal/cmsis;$CACHE/modules/hal/cmsis_6;$CACHE/modules/lib/picolibc"

# The Zephyr SDK ships picolibc prebuilt; with a distro GCC build it from the module.
LIBC_CONF="$OUT/libc.conf"
mkdir -p "$OUT"
printf 'CONFIG_PICOLIBC=y\nCONFIG_PICOLIBC_USE_MODULE=y\n' > "$LIBC_CONF"

status=0
for board in "${BOARDS[@]}"; do
  dir="$OUT/$board"
  echo "=== $board"
  rm -rf "$dir"
  if cmake -GNinja -B "$dir" -S "$APP/zephyr" -DBOARD="$board" \
       -DZEPHYR_MODULES="$MODULES" -DEXTRA_CONF_FILE="$LIBC_CONF" > "$dir.cmake.log" 2>&1 \
     && ninja -C "$dir" > "$dir.build.log" 2>&1; then
    grep -E "^ +(FLASH|RAM):" "$dir.build.log" | sort -u
    grep -E "warning:" "$dir.build.log" | grep -v "/modules/lib/picolibc/" | sort -u || true
  else
    echo "BUILD FAILED ($board):"
    grep -E "error|Error" "$dir.cmake.log" "$dir.build.log" 2>/dev/null | head -30
    status=1
  fi
done
exit $status
