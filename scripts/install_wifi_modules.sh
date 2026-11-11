#!/bin/sh
# On-board installation of the verified candidate archive. Does not reboot.
set -eu
trial=${1:?Usage: sh install_wifi_modules.sh /path/to/unpacked/trial}
cd "$trial"
test "$(uname -r)" = 4.5.0
if pidof pitch_pd balance_lqr robot_test balance_car DE10_Nano_bal >/dev/null 2>&1; then
    echo 'STOP: hardware program active' >&2
    exit 1
fi
sha256sum -c SHA256SUMS
target=/lib/modules/4.5.0/updates/terasic_wifi_trial
loader=/etc/modules-load.d/terasic-wifi.conf
# Refuse to replace any prior installation or boot configuration.
test ! -e "$target"
test ! -e "$loader"
for file in crypto/*.ko net/wireless/*.ko net/mac80211/*.ko drivers/net/wireless/mediatek/mt7601u/*.ko; do
    test -f "$file"
    name=${file##*/}
    if find /lib/modules/4.5.0 -type f -name "$name" | read -r existing; then
        echo "STOP: existing module $name" >&2
        exit 1
    fi
done
mkdir -p "$target"
cp -R crypto drivers net "$target/"
depmod -a 4.5.0
mkdir -p /etc/modules-load.d
printf '%s\n' '# USB Wi-Fi MT7601U; original kernel retained' mt7601u > "$loader"
sync
echo 'Installed new modules and mt7601u boot loading. No reboot performed.'
modinfo -F filename mt7601u
