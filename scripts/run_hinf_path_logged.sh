#!/bin/sh
# On-board launcher with separate logs. Does not detach from SSH or run offline.
# Usage: sh scripts/run_hinf_path_logged.sh shadow/run config.txt seconds max_pwm
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
binary=${HINF_PATH_BINARY:-$project_dir/build/robot/hinf_path}
mode=${1:-}
case "$mode:$#" in
    shadow:4) category=observe ;;
    run:4) category=tuning ;;
    *) echo "Uso: $0 shadow/run config.txt secondi max_pwm" >&2; exit 2 ;;
esac
if [ ! -x "$binary" ]; then echo "Compilare prima $binary sulla scheda." >&2; exit 1; fi
log_dir=${ROBOT_LOG_DIR:-$project_dir/logs/real/hinf}
mkdir -p "$log_dir"
log_base=$log_dir/hinf_path_${mode}_$(date +%Y%m%d_%H%M%S)_$$
cp "$2" "$log_base.config.txt"
echo "Configurazione: $log_base.config.txt"
echo "CSV: $log_base.csv"
echo "Stato: $log_base.status.log"
{
    printf 'Avvio: '; date
    printf 'Argomenti:'; printf ' %s' "$@"; printf '\n'
    printf 'Protocollo dichiarato: %s\n' "${HINF_TEST_TAG:-non specificato}"
    echo 'startup_protocol=inclined_initial_support_free_recovery; fixed_pitch_reference; gyro_bias_only'
    echo 'Supporto iniziale: ruote a terra, telaio libero di staccarsi; sostenere prima dello stop.'
    if [ "$mode" != observe ]; then
        echo 'Configurazione:'
        cat "$2"
    fi
} > "$log_base.status.log"
sha256sum "$binary" "$log_base.config.txt" >> "$log_base.status.log"
record_battery() {
    [ "$mode" = run ] || return 0
    {
        printf '\nBatteria %s (a motori spenti): ' "$1"
        date
        battery_result=0
        if [ -x "$project_dir/build/robot/battery_status" ]; then
            "$project_dir/build/robot/battery_status" || battery_result=$?
        else
            echo 'Diagnostico battery_status non disponibile: avvio non consentito.'
            battery_result=1
        fi
        printf 'exit_code_battery=%s\n' "$battery_result"
        return "$battery_result"
    } >> "$log_base.status.log" 2>&1
}
child=
logger=
stopper=
cleanup() {
    trap - INT TERM HUP
    if [ -n "$child" ]; then
        kill -TERM "$child" 2>/dev/null || true
        wait "$child" 2>/dev/null || true
    fi
    if [ -n "$stopper" ]; then
        kill "$stopper" 2>/dev/null || true
        wait "$stopper" 2>/dev/null || true
    fi
    if [ -n "$logger" ]; then
        kill "$logger" 2>/dev/null || true
        wait "$logger" 2>/dev/null || true
    fi
    record_battery DOPO || true
    printf '\nLauncher interrotto; exit_code=130\n' >> "$log_base.status.log"
    sync
    exit 130
}
trap cleanup INT TERM HUP
if record_battery PRIMA; then
    :
else
    result=$?
    : > "$log_base.csv"
    printf '\nSTART_BLOCKED=battery_check_failed; exit_code=%s\n' "$result" >> "$log_base.status.log"
    cat "$log_base.status.log" >&2
    sync
    exit "$result"
fi
"$binary" "$@" > "$log_base.csv" 2>> "$log_base.status.log" &
child=$!
if [ "$mode" = run ] && [ "${HINF_AUTO_STOP_PROFILE:-0}" = 1 ]; then
    printf '\nauto_stop_profile=enabled; fallback_wall_timeout_approx_s=40\n' >> "$log_base.status.log"
    sh "$project_dir/scripts/watch_profile_stop.sh" "$child" "$log_base.status.log" &
    stopper=$!
fi
# The controller writes directly to a regular file. A separate reader displays
# progress; losing SSH/tail cannot give the controller a broken output pipe.
tail -n +1 -f "$log_base.status.log" >&2 &
logger=$!
result=0
wait "$child" || result=$?
child=
if [ -n "$stopper" ]; then
    kill "$stopper" 2>/dev/null || true
    wait "$stopper" 2>/dev/null || true
    stopper=
fi
record_battery DOPO || true
kill "$logger" 2>/dev/null || true
wait "$logger" 2>/dev/null || true
logger=
printf '\nexit_code=%s\n' "$result" >> "$log_base.status.log"
sync
tail -n 12 "$log_base.status.log"
exit "$result"
