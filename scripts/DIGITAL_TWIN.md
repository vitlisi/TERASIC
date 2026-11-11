# Real-core digital twin

Run from the repository root:

```sh
env/bin/python -B scripts/digital_twin.py simulate --case stand
env/bin/python -B scripts/digital_twin.py simulate --case path
env/bin/python -B scripts/digital_twin.py simulate --case hinf
env/bin/python -B scripts/digital_twin.py simulate --case hinf-path
env/bin/python -B scripts/simulate_ga_twin.py simulate --case both
```

The offline bridge reuses real parsers, estimators, state conventions, allocation
and reference logic without invoking hardware entry points. The GA wrapper uses
`diagnostics/digital_twin_ga_bridge.cpp` and the same one-sample feedback delay
as the transferred real GA programs. Passing GA gains to the ordinary twin alone
would omit that delay. Its summary records the variant and source hashes.

Defaults retain distinct SX/DX motors, current Stribeck profile, MPU6500 model,
12 V and an assumed 20 g payload at 10 cm. They are explicit assumptions, not a
matched acquisition. Select measured voltage, initial conditions and the real
configuration for each final comparison. `--legacy-motor-friction` is only for
historical reproduction. `--nominal-motors` also disables the calibrated profile.

The standard bridge does not enable the experimental bounded compensator.
Its qualified raised-wheel route is `diagnostics/simulate_compensated_freewheel.py`;
compensated physical LQR uses the separate `balance_lqr_stribeck` program.
Final H-infinity/GA compensation integration must be checked before those trials.

`compare --simulation ... --real ... --output ...` requires configuration snapshots
and physical wheel-convention status. It cannot validate unmeasured slip or torque.
Do not crop away calibration or initial recovery in final acquisition figures.
The loaded-body discrepancy remains unresolved; no exploratory fitted correction
was promoted during cleanup.
