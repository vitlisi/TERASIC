#!/bin/sh
# Read-only checks on the robot: no /dev/mem, motor commands or credentials.
set -u
project=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
expected_name=${1:-$(hostname)}
wifi_interface=${2:-wlan0}
case "$expected_name:$wifi_interface" in
    *[!a-zA-Z0-9_.:-]*|'':*) echo "Uso: sh $0 [nome_robot] [interfaccia_wifi]" >&2; exit 2 ;;
esac
errors=0
warnings=0
ok() { printf 'OK   %s\n' "$*"; }
fail() { printf 'FAIL %s\n' "$*"; errors=$((errors + 1)); }
warn() { printf 'WARN %s\n' "$*"; warnings=$((warnings + 1)); }

printf 'Verifica configurazione Linux del robot — sola lettura\n'
printf 'Data UTC: '; date -u
printf 'Sistema: '; uname -srmo
if [ "$(hostname)" = "$expected_name" ]; then
    ok "Hostname: $expected_name"
else
    fail "Hostname attuale $(hostname), atteso $expected_name"
fi
if [ -r /etc/hostname ]; then
    static_name=$(tr -d '\r\n' < /etc/hostname)
    if [ "$static_name" = "$(hostname)" ]; then
        ok 'Hostname statico coerente con quello attuale'
    else
        warn "Hostname statico $static_name diverso da quello attuale"
    fi
fi

for tool in ip connmanctl sshd g++ modprobe; do
    if command -v "$tool" >/dev/null 2>&1; then ok "Strumento: $tool"; else fail "Strumento assente: $tool"; fi
done
if command -v systemctl >/dev/null 2>&1; then
    for service in connman avahi-daemon; do
        if systemctl is-active --quiet "$service"; then ok "Servizio: $service"; else fail "Servizio inattivo: $service"; fi
    done
    ssh_active=no
    for unit in sshd.socket ssh.socket sshd.service ssh.service; do
        if systemctl is-active --quiet "$unit" 2>/dev/null; then ssh_active=yes; ok "SSH disponibile tramite $unit"; fi
    done
    [ "$ssh_active" = yes ] || fail 'Nessuna unita SSH attiva'
else
    warn 'systemctl assente: verificare i servizi con il gestore della propria immagine'
fi

if command -v lsusb >/dev/null 2>&1; then
    printf '\nDispositivi USB:\n'; lsusb
    if lsusb | grep -qi '148f:7601'; then
        ok 'Dongle MT7601U del riferimento presente'
        if command -v modinfo >/dev/null 2>&1 && modinfo mt7601u >/dev/null 2>&1; then
            ok "Driver disponibile: $(modinfo -F vermagic mt7601u)"
        else
            fail 'Modulo mt7601u assente'
        fi
        if [ -r /lib/firmware/mt7601u.bin ]; then ok 'Firmware MT7601U presente'; else fail 'Firmware MT7601U assente'; fi
    else
        warn 'Dongle diverso dal MT7601U: verificare il driver del proprio chipset'
    fi
else
    warn 'lsusb assente: inventario USB non disponibile'
fi
if command -v ip >/dev/null 2>&1; then
    printf '\nRete:\n'
    if ip link show dev "$wifi_interface" >/dev/null 2>&1; then
        ok "Interfaccia $wifi_interface presente"
        address=$(ip -4 -o address show dev "$wifi_interface" scope global | awk '{print $4}')
        if [ -n "$address" ]; then ok "IPv4 $wifi_interface: $address"; else fail "Nessun IPv4 su $wifi_interface"; fi
        if ip link show dev "$wifi_interface" | grep -q 'LOWER_UP'; then ok 'Collegamento Wi-Fi attivo'; else fail 'Collegamento Wi-Fi non attivo'; fi
    else
        fail "Interfaccia $wifi_interface assente"
    fi
    ip route
    if ip route show default | grep -q "dev $wifi_interface\( \|$\)"; then
        ok 'Gateway predefinito tramite Wi-Fi'
    else
        warn 'Gateway predefinito non presente sul Wi-Fi; verificare il percorso verso il PC'
    fi
fi
printf '\nProgetto: %s\n' "$project"
for binary in battery_status encoder_manual imu_identity motor_freewheel; do
    if [ -x "$project/build/robot/$binary" ]; then ok "Binario compilato: $binary"; else fail "Binario assente: $binary; eseguire sh scripts/build_robot.sh"; fi
done
printf '\nStato FPGA esposto dal kernel (se disponibile):\n'
found_fpga=no
for file in /sys/class/fpga_manager/*/state /sys/class/fpga/*/status; do
    if [ -r "$file" ]; then found_fpga=yes; printf '%s: ' "$file"; cat "$file"; fi
done
[ "$found_fpga" = yes ] || printf 'Non disponibile tramite queste interfacce sysfs.\n'
printf '\nQuesta verifica riguarda Linux, rete e file del progetto.\n'
printf 'Non certifica bitstream/mappa registri, batteria, IMU, encoder o motori.\n'
printf 'Nessun accesso a /dev/mem e nessun comando motore eseguito.\n'
printf 'Esito: %s errori, %s avvisi.\n' "$errors" "$warnings"
[ "$errors" -eq 0 ]
