#!/bin/sh
# Stop only the explicit child PID when its logged temporal profile completes.
set -eu
[ "$#" -eq 2 ] || exit 2
profile_pid=$1
profile_log=$2
case "$profile_pid" in ''|*[!0-9]*|0|1) exit 2;; esac
polls=0
while kill -0 "$profile_pid" 2>/dev/null; do
    if grep -q 'PROFILO TEMPORALE TERMINATO:' "$profile_log"; then
        printf '\nauto_stop_reason=profile_completed\n' >> "$profile_log"
        kill -TERM "$profile_pid" 2>/dev/null || true
        exit 0
    fi
    polls=$((polls+1))
    if [ "$polls" -ge 800 ]; then
        printf '\nauto_stop_reason=wall_timeout_approx40s\n' >> "$profile_log"
        kill -TERM "$profile_pid" 2>/dev/null || true
        exit 1
    fi
    sleep 0.05
done
