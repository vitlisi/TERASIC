#!/bin/sh
# On-board launcher with separate logs. Does not detach from SSH or run offline.
# Usage: sh scripts/run_pd_logged.sh pd/pd-outer observe/run PD_arguments
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
controller=${1:-}
shift || exit 2
case "$controller" in
    pd) binary=$project_dir/build/robot/pitch_pd_fixed; run_count=8 ;;
    pd-outer) binary=$project_dir/build/robot/pitch_pd_outer; run_count=10 ;;
    *) echo 'Uso: run_pd_logged.sh pd/pd-outer observe/run argomenti_PD' >&2; exit 2 ;;
esac
mode=${1:-}
case "$mode:$#" in
    observe:3) : ;;
    run:$run_count) : ;;
    *) echo 'Consultare --help del binario PD per gli argomenti.' >&2; exit 2 ;;
esac
if [ ! -x "$binary" ]; then echo "Compilare prima $binary sulla scheda." >&2; exit 1; fi
log_dir=${ROBOT_LOG_DIR:-$project_dir/logs/real/$controller}
mkdir -p "$log_dir"
log_base=$log_dir/${controller}_stand_${mode}_$(date +%Y%m%d_%H%M%S)_$$
# PD arguments are the configuration; this is not a controller TXT profile.
printf '%s\n' "$@" > "$log_base.args.txt"
echo "Configurazione: $log_base.args.txt"
echo "CSV: $log_base.csv"
echo "Stato: $log_base.status.log"
{
    printf 'Avvio: '; date
    printf 'Argomenti:'; printf ' %s' "$@"; printf '\n'
    printf 'Protocollo dichiarato: %s\n' "${LQR_TEST_TAG:-non specificato}"
    echo 'startup_protocol=inclined_initial_support_free_recovery; fixed_pitch_reference; gyro_bias_only'
    echo 'Supporto iniziale: ruote a terra, telaio libero di staccarsi; sostenere prima dello stop.'

} > "$log_base.status.log"
sha256sum "$binary" "$log_base.args.txt" >> "$log_base.status.log"
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
cleanup() {
    trap - INT TERM HUP
    if [ -n "$child" ]; then
        kill -TERM "$child" 2>/dev/null || true
        wait "$child" 2>/dev/null || true
    fi
    if [ -n "$logger" ]; then
        kill "$logger" 2>/dev/null || true
        wait "$logger" 2>/dev/null || true
    fi
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
# The controller writes directly to a regular file. A separate reader displays
# progress; losing SSH/tail cannot give the controller a broken output pipe.
tail -n +1 -f "$log_base.status.log" >&2 &
logger=$!
result=0
wait "$child" || result=$?
child=
record_battery DOPO || true
kill "$logger" 2>/dev/null || true
wait "$logger" 2>/dev/null || true
logger=
printf '\nexit_code=%s\n' "$result" >> "$log_base.status.log"
sync
tail -n 12 "$log_base.status.log"
exit "$result"
