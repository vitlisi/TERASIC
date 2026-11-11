# Raised-wheel PWM pulses, compensation OFF

Status updated: use recovered historical inputs only. No new digital-twin acquisition is planned. See `../historical_inputs/inputs_manifest.json` and `docs/REPORT_FIGURES_EN.md`.

## Required figures

- Applied PWM, wheel speed overlay and signed simulation-minus-real error
- Complete acquisitions from t=0; separate low-PWM transition zooms

## Required inputs and checks

- Recovered real acquisition; fixed chassis and no ground contact
- Nominal +/-4%, +/-6%, +/-8% pulses, both signs and repeat order declared
- Current Stribeck physics, quantized encoder and same speed filter

## Existing tools

- `diagnostics/replay_stateful_motor.py`
- `diagnostics/compare_freewheel.py`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/01_digital_twin/pulses_off`.
