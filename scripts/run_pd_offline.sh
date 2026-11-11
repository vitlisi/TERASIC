#!/bin/sh
# Eseguire SULLA DE10: sh run_pd_offline.sh observe|run attesa_s durata_s
# Launcher esterno: non modifica filtro, guadagni o protezioni di pitch_pd.
set -u
mode=${1:-}
delay=${2:-}
duration=${3:-}
case "$mode" in observe|run) ;; *) echo 'Uso: sh run_pd_offline.sh observe|run attesa_s durata_s' >&2; exit 2 ;; esac
if [ "$#" -ne 3 ]; then echo 'Richiesti esattamente tre argomenti.' >&2; exit 2; fi
for number in "$delay" "$duration"; do
    case "$number" in ''|*[!0-9]*) echo 'Tempi interi richiesti.' >&2; exit 2 ;; esac
done
if [ "$delay" -lt 5 ] || [ "$delay" -gt 300 ] || [ "$duration" -lt 1 ] || [ "$duration" -gt 30 ]; then
    echo 'Attesa 5..300 s; durata controllo 1..30 s.' >&2; exit 2
fi
project=/home/root/terasic_robot
cd "$project" || exit 1
controller=${PD_CONTROLLER:-pitch_pd}
case "$controller" in pitch_pd) tag=pd_pitch ;; pitch_pd_fixed) tag=pd_riferimento_fisso ;; *) echo "PD_CONTROLLER non valido" >&2; exit 2;; esac
binary=./build/robot/$controller
if [ ! -x "$binary" ]; then echo 'Manca il binario eseguibile pitch_pd.' >&2; exit 1; fi
# Nuova sessione obbligatoria: nohup da solo non basta con SIGHUP gestito dal PD.
if [ -t 0 ] || [ -t 1 ] || [ -t 2 ]; then
    echo 'Avviare con nohup setsid sh run_pd_offline.sh ... < /dev/null > offline_launcher.log 2>&1 &' >&2
    exit 2
fi
lock="$project/.pd_offline_lock"
if ! mkdir "$lock" 2>/dev/null; then
    echo 'Un launcher offline è già attivo o resta un lock da verificare.' >&2
    exit 1
fi
child=''
log_dir="logs/terasic_reale/runs/$tag/tuning"
if [ "$mode" = observe ]; then log_dir="logs/terasic_reale/runs/$tag/observe"; fi
mkdir -p "$log_dir" || { rmdir "$lock"; exit 1; }
base="$log_dir/offline_$(date +%Y%m%d_%H%M%S)_$$"
printf '%s\n' "$$" > "$lock/pid"
printf '%s\n' "$base" > "$lock/log_base"
finish() {
    sync
    rm -f "$lock/pid" "$lock/log_base"
    rmdir "$lock" 2>/dev/null || :
}
stop() {
    trap '' INT TERM
    if [ -n "$child" ]; then
        kill -TERM "$child" 2>/dev/null || :
        wait "$child" 2>/dev/null || :
    fi
    printf '\nLauncher interrotto; stop richiesto al PD.\n' >> "$base.status.log"
    exit 130
}
trap finish 0
trap stop INT TERM
printf 'Log: %s.csv\nModalità: %s. Attesa: %s s. Controllo: %s s.\n' "$base" "$mode" "$delay" "$duration"
printf 'Launcher PID=%s; mode=%s; delay_s=%s; duration_s=%s\n' "$$" "$mode" "$delay" "$duration" > "$base.status.log"
printf 'Avvio: %s\n' "$(date)" >> "$base.status.log"
# Numeric arguments and controller name were validated above.
motors=false
[ "$mode" != run ] || motors=true
cat > "$base.json" <<EOF
{"wheel_convention":"physical_sx_dx_v2","controller":"$controller","mode":"$mode","duration_s":$duration,"delay_s":$delay,"motors_enabled":$motors,"kp":5,"kd":0.8,"max_pwm_percent":15}
EOF
sha256sum "$binary" >> "$base.status.log"
# La durata richiesta NON include l'attesa e i circa 2 s di calibrazione.
remaining=$delay
while [ "$remaining" -gt 0 ]; do
    sleep 1
    remaining=$((remaining - 1))
done
printf 'Avvio calibrazione: %s\n' "$(date)" >> "$base.status.log"
if [ "$mode" = observe ]; then
    "$binary" observe "$duration" > "$base.csv" 2>> "$base.status.log" &
else
    "$binary" run 1 1 5.0 0.8 15 "$duration" > "$base.csv" 2>> "$base.status.log" &
fi
child=$!
wait "$child"
code=$?
child=''
printf '\nexit_code=%s\nFine: %s\n' "$code" "$(date)" >> "$base.status.log"
printf 'Terminato: exit_code=%s, CSV=%s.csv\n' "$code" "$base"
exit "$code"
