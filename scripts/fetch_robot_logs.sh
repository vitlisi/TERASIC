#!/usr/bin/env bash
# Copia sul PC solo i nuovi CSV e log di stato della DE10-Nano.
# Uso: bash scripts/fetch_robot_logs.sh [host] [lqr_fermo|lqr_40cm_180_40cm|hinf_fermo|hinf_40cm_180_40cm] [--latest]
set -euo pipefail

board_host=${1:?Usage: bash scripts/fetch_robot_logs.sh user@address [case] [--latest]}
selected_case=${2:-}
latest_only=${3:-}
case "$selected_case" in
    ''|lqr_fermo|lqr_40cm_180_40cm|hinf_fermo|hinf_40cm_180_40cm) ;;
    *) echo 'Caso non supportato' >&2; exit 2 ;;
esac
if [[ -n "$latest_only" && ( "$latest_only" != --latest || -z "$selected_case" ) ]]; then
    echo '--latest richiede un caso LQR esplicito' >&2; exit 2
fi
project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
local_dir=$project_dir/logs/terasic_reale
remote_dir=/home/root/terasic_robot/logs/terasic_reale
manifest=$(mktemp)
trap 'rm -f "$manifest"' EXIT

ssh -o BatchMode=yes -o ConnectTimeout=5 "$board_host" "cd '$remote_dir' && find . -type f" > "$manifest"
latest_base=
if [[ "$latest_only" == --latest ]]; then
    latest_csv=$(rg "^\./runs/$selected_case/tuning/[^/]+\.csv$" "$manifest" | sort | tail -n 1) || true
    if [[ -z "$latest_csv" ]]; then echo 'Nessun CSV tuning trovato' >&2; exit 1; fi
    latest_base=${latest_csv#./}
    latest_base=${latest_base%.csv}
fi

copied=0
skipped=0
while IFS= read -r remote_path; do
    relative=${remote_path#./}
    if [[ -n "$selected_case" && "$relative" != runs/"$selected_case"/* ]]; then continue; fi
    if [[ -n "$latest_base" ]]; then
        case "$relative" in
            "$latest_base".csv|"$latest_base".status.log|"$latest_base".config.txt) ;;
            *) continue ;;
        esac
    fi
    case "$relative" in
        runs/*|hinf/*|diagnostics/*|imu_sign/*|observe/*|pd_lifted/*|pd_ground/*|tuning/*|lqr_mimo/*|lqr_path/*|endurance/*|ultrasonic/*) ;;
        *) continue ;;
    esac
    case "$relative" in
        *.csv|*.status.log|*.config.txt|*.json) ;;
        *) continue ;;
    esac
    if [[ ! "$relative" =~ ^[A-Za-z0-9_./-]+$ ]] || [[ "$relative" == *..* ]]; then
        printf 'Nome non valido, salto: %s\n' "$relative" >&2
        continue
    fi
    destination=$local_dir/$relative
    if [[ -e "$destination" ]]; then
        ((skipped+=1))
        continue
    fi
    mkdir -p "$(dirname "$destination")"
    scp -o BatchMode=yes -o ConnectTimeout=5 "$board_host:$remote_dir/$relative" "$destination"
    printf 'Copiato: %s\n' "$relative"
    ((copied+=1))
done < "$manifest"

printf 'Nuovi file: %d; gia presenti: %d\n' "$copied" "$skipped"
