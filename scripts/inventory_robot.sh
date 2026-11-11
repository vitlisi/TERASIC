#!/bin/sh
# Read-only inventory. Does not launch controllers or delete remote files.
set -eu
host=${1:?Uso: sh scripts/inventory_robot.sh utente@indirizzo}
case "$host" in -*|*[!a-zA-Z0-9@._:-]*) echo 'Host non valido' >&2; exit 2;; esac
ssh -o BatchMode=yes -o ConnectTimeout=5 "$host" '
set -eu
cd /home/root/terasic_robot
hostname
uname -r
du -sh . logs 2>/dev/null || true
find . -maxdepth 3 -type f | sort
find . -maxdepth 1 -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.txt" \) -exec sha256sum {} \;
ps -eo pid,comm
'
