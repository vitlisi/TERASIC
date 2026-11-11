# Equal nominal motors versus calibrated Stribeck motors

Status updated: use recovered historical inputs only. No new digital-twin acquisition is planned. See `../historical_inputs/inputs_manifest.json` and `docs/REPORT_FIGURES_EN.md`.

## Required figures

- Full pulse response: applied PWM and left/right speed
- Real minus simulated speed; full traces and separate transition zooms

## Required inputs and checks

- New uncompensated real raised-wheel pulses
- Same applied physical PWM, measured voltage and timestamps for both model variants
- Nominal XML is the equal-motor hypothesis, not a recovered historical measurement

## Existing tools

- `diagnostics/compare_freewheel.py`
- `diagnostics/replay_stateful_motor.py`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/01_digital_twin/model_evolution`.
