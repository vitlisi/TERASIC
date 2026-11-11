#!/bin/sh
# Build candidates on the PC only; never installs or loads on the DE10.
set -eu
project=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
source_dir=${WIFI_KERNEL_SOURCE:-/tmp/terasic-wifi-linux-socfpga-4.5}
build_dir=${WIFI_KERNEL_BUILD:-/tmp/terasic-wifi-build}
records=$project/maintenance/wifi
command -v arm-linux-gnueabihf-gcc >/dev/null
test -f "$source_dir/Makefile"
test -f "$records/kernel-4.5.0.config.gz"
mkdir -p "$build_dir"
gzip -dc "$records/kernel-4.5.0.config.gz" > "$build_dir/.config"
"$source_dir/scripts/config" --file "$build_dir/.config" \
    --module CFG80211 --module MAC80211 --module MT7601U \
    --enable MAC80211_RC_MINSTREL --enable MAC80211_RC_MINSTREL_HT \
    --enable MAC80211_RC_DEFAULT_MINSTREL \
    --disable MAC80211_MESH --disable MAC80211_LEDS --disable MAC80211_DEBUGFS
# GCC 10+ defaults to -fno-common: the legacy HOST dtc needs -fcommon.
# Explicit ARMv7/FPU flags avoid the old cc-option ARMv5 fallback with GCC 11.
# Disable Ubuntu GCC's default PIC/PIE: the old ARM module loader rejects GOT.
# LOCALVERSION= suppresses git's automatic '+'; this does NOT prove ABI match.
make -C "$source_dir" O="$build_dir" ARCH=arm \
    CROSS_COMPILE=arm-linux-gnueabihf- LOCALVERSION= olddefconfig
make -C "$source_dir" O="$build_dir" ARCH=arm \
    CROSS_COMPILE=arm-linux-gnueabihf- LOCALVERSION= \
    HOSTCFLAGS='-O2 -fcommon' KCFLAGS='-march=armv7-a -mfpu=vfpv3-d16 -fno-pic -fno-pie' \
    -j4 modules
echo 'Build only. Verify exported symbols, module metadata and source compatibility before loading.'
