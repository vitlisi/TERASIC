#!/bin/sh
# Inventory only: no /dev/mem, motor commands, network changes or credential reads.
set -eu
robot_id=${1:-unassigned}
case "$robot_id" in ''|*[!a-zA-Z0-9_-]*) echo 'Uso: sh scripts/inspect_robot_setup.sh robot02' >&2; exit 2;; esac
printf 'robot_id=%s\n' "$robot_id"
printf 'inventory_utc='; date -u '+%Y-%m-%dT%H:%M:%SZ'
printf 'hostname='; hostname
uname -a
for tool in connmanctl wpa_supplicant udhcpc avahi-daemon sshd g++ modprobe; do
    if command -v "$tool" >/dev/null 2>&1; then command -v "$tool"; else printf 'missing_tool=%s\n' "$tool"; fi
done
if command -v lsusb >/dev/null 2>&1; then lsusb; fi
if command -v ip >/dev/null 2>&1; then ip address; ip route; fi
if command -v modinfo >/dev/null 2>&1; then
    modinfo -F filename mt7601u 2>/dev/null || true
    modinfo -F vermagic mt7601u 2>/dev/null || true
fi
if [ -r /lib/firmware/mt7601u.bin ]; then sha256sum /lib/firmware/mt7601u.bin; fi
if command -v systemctl >/dev/null 2>&1; then
    for unit in connman avahi-daemon sshd.socket ssh.socket ssh sshd; do
        printf 'service=%s state=' "$unit"
        systemctl is-active "$unit" 2>/dev/null || true
    done
fi
for file in /boot/zImage /boot/socfpga.dtb /boot/soc_system.rbf /boot/u-boot.scr; do
    if [ -f "$file" ]; then sha256sum "$file"; fi
done
printf '%s\n' 'inventory_only=yes motor_access=no'
