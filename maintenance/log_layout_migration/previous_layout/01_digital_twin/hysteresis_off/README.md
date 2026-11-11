# Uncompensated raised-wheel hysteresis

Status updated: use recovered historical inputs only. No new digital-twin acquisition is planned. See `../historical_inputs/inputs_manifest.json` and `docs/REPORT_FIGURES_EN.md`.

## Required figures

- Steady wheel speed versus applied PWM, left and right
- From-rest and prestarted branches, positive and negative directions
- Near-threshold zoom and starting/stopping threshold summary

## Required inputs and checks

- PWM increments 0.5 percentage points; not a 0.5 s sampling interval
- Recovered real stepped acquisitions; preserve the original prestart, dwell, steady window and voltage range
- Matched fixed-chassis, no-floor-contact simulation
- Historical static points are replayed offline by scripts/plot_digital_twin_report.py

## Existing tools

- `src_python/stribeck_motor.py`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/01_digital_twin/hysteresis_off`.
