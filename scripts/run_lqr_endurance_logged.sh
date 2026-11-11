#!/bin/sh
# One compact CSV and short status log. User starts this script on the board.
set -eu
exec 3<&0 # Preserve SSH heartbeat stdin before starting a background job.
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$project_dir"
config=${1:-config/controllers/lqr_path_40cm_28p5s.txt}
cutoff=${2:-10.5}
max_s=${3:-3600}
tag=${4:-endurance_$(date +%Y%m%d_%H%M%S)_$$}
case "$tag" in *[!a-zA-Z0-9_-]*|'') echo 'Nome prova non valido' >&2;exit 2;; esac
mkdir -p logs/terasic_reale/runs/lqr_endurance
base=logs/terasic_reale/runs/lqr_endurance/$tag
if [ -e "$base.csv" ] || [ -e "$base.status.log" ]; then echo 'Prova esistente: scegliere un altro nome' >&2;exit 2;fi
cp "$config" "$base.config.txt"
child=
logger=
cleanup(){
    trap - INT TERM HUP
    if [ -n "$child" ]; then kill -TERM "$child" 2>/dev/null || true;wait "$child" 2>/dev/null || true;fi
    if [ -n "$logger" ]; then kill "$logger" 2>/dev/null || true;wait "$logger" 2>/dev/null || true;fi
    printf '\nLAUNCHER_END=operator_or_connection_closed\n' >> "$base.status.log"
    sync
    exit 130
}
trap cleanup INT TERM HUP
{
    date
    echo "PROFILE=40cm/180deg/40cm/180deg CYCLE_S=35.5 CUTOFF_V=$cutoff MAX_S=$max_s"
    echo 'Current unavailable; ADC voltage nominal scale, no Ah/Wh/SOC measurement.'
    echo 'Cutoff: filtered2s voltage low for2s -> hold10s -> motors OFF. Operator must support robot.'
    sha256sum build/robot/lqr_endurance "$base.config.txt"
    cat "$config"
} > "$base.status.log"
echo "CSV: $project_dir/$base.csv"
echo "STATUS: $project_dir/$base.status.log"
./build/robot/lqr_endurance run "$config" 28.5 50 "$cutoff" "$max_s" <&3 > "$base.csv" 2>> "$base.status.log" &
child=$!
# Display live progress without piping controller writes through a network stream.
tail -n 0 -f "$base.status.log" &
logger=$!
set +e
wait "$child"
result=$?
set -e
child=
kill "$logger" 2>/dev/null || true
wait "$logger" 2>/dev/null || true
printf '\nLAUNCHER_EXIT=%s\n' "$result" >> "$base.status.log"
sync
trap - INT TERM HUP
exit "$result"
