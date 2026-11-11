#!/bin/sh
# Compile only. Run on the DE10 for ARM executables, or on the PC for checks.
set -eu
project=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
destination=${1:-"$project/build/robot"}
mkdir -p "$destination"
compiler=${CXX:-g++}
for entry in \
    controllers/pitch_pd.cpp:pitch_pd \
    controllers/pitch_pd_fixed.cpp:pitch_pd_fixed \
    controllers/pitch_pd_outer.cpp:pitch_pd_outer \
    controllers/balance_lqr_mimo.cpp:balance_lqr_2 \
    controllers/balance_hinf.cpp:balance_hinf \
    controllers/balance_hinf_path.cpp:hinf_path \
    controllers/balance_lqr_path_simple.cpp:path_simple \
    controllers/balance_lqr_endurance.cpp:lqr_endurance \
    hardware/battery_status.cpp:battery_status \
    hardware/pitch_live.cpp:pitch_live \
    hardware/robot_test.cpp:robot_test \
    hardware/ultrasonic_live.cpp:ultrasonic_live
do
    source=${entry%:*}
    binary=${entry#*:}
    "$compiler" -std=c++98 -O2 -Wall -Wextra -Werror \
        "$project/src_cpp/terasic_reale/$source" -o "$destination/$binary" -lrt
    printf 'Compilato: %s\n' "$destination/$binary"
done

# Diagnostics compiled separately; never executed by this script.
for name in fpga_readonly encoder_manual imu_identity imu_measure motor_pulse motor_freewheel; do
    "$compiler" -std=c++98 -O2 -Wall -Wextra -Werror \
        "$project/diagnostics/$name.cpp" -o "$destination/$name" -lrt
    printf 'Compilato diagnostico: %s\n' "$destination/$name"
done

# Frozen transferred controllers, compiled only; no hardware is started.
for name in balance_lqr_ga path_lqr_ga balance_hinf_tuned_g035_r050 path_hinf_tuned_g035_r050 balance_lqr_guarded balance_lqr_stribeck motor_compensated_freewheel motor_compensated_freewheel_imu; do
    "$compiler" -std=c++98 -O2 -Wall -Wextra -Werror \
        "$project/diagnostics/$name.cpp" -o "$destination/$name" -lrt
    printf 'Compilato: %s\n' "$destination/$name"
done
