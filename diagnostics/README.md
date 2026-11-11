# Diagnostics and qualified experimental routes

These files include offline tools and physical programs. Cleanup did not run hardware.

| Tools | Purpose |
|---|---|
| `fpga_readonly.cpp`, `encoder_manual.cpp`, `imu_*.cpp` | Board registers and sensor diagnostics |
| `motor_pulse.cpp`, `motor_freewheel.cpp` | Physical motor protocols |
| `compare_freewheel.py`, `replay_stateful_motor.py` | Offline replay, fixed chassis and no floor contact |
| `simulate_compensated_freewheel.py` | Current Stribeck physics and exact C++ bounded compensation |
| `run_compensated_freewheel*_trial.py` | Guarded physical compensated raised-wheel acquisition |
| `analyze_compensated_freewheel.py`, `plot_compensated_freewheel.py` | Measured compensated acquisitions and matched simulation |
| `balance_lqr_stribeck.cpp`, `run_guarded_balance_trial.py`, `analyze_stribeck_trial.py` | Separate guarded compensated LQR route |
| `balance_lqr_ga.cpp`, `path_lqr_ga.cpp` | Transferred fixed GA gains and one-sample feedback delay |
| `balance_hinf_tuned_g035_r050.cpp`, `path_hinf_tuned_g035_r050.cpp` | Retained tuned H-infinity variant, not an automatic final selection |
| `inverse_body_torque.py` | Supplementary inverse dynamics only |

Frozen compensation inputs live in `config/motors/`, local qualification records
in `config/qualification/`. Qualification hashes are evidence for their original
builds; changed binaries must be checked before a new physical run. Remote board
paths were not changed or contacted during cleanup.

The generated `stribeck_compensation_data.h` remains byte-identical. Regenerate:

```sh
env/bin/python -B diagnostics/generate_stribeck_compensation_data.py \
  --candidate config/motors/robot01_compensation_candidate.json \
  --output /tmp/stribeck_compensation_data.h
```

The compensator limits correction to +/-0.5 physical PWM percentage points.
Stribeck physics and compensation are separate choices. Do not label an
uncompensated real acquisition as compensated data. Conditional research fits
were not adopted; obsolete fit drivers and their result data were deleted.
