# Raised-wheel PWM pulses, bounded compensation ON

Status updated: use recovered historical inputs only. No new digital-twin acquisition is planned. See `../historical_inputs/inputs_manifest.json` and `docs/REPORT_FIGURES_EN.md`.

## Required figures

- Nominal versus applied PWM
- Real/simulated left/right speed and signed error
- OFF/ON performance comparison and transition zooms

## Required inputs and checks

- Recovered compensated real acquisition
- Exact same C++ bounded compensator on both platforms
- Preserve requested/applied PWM, voltage and encoder counts

## Existing tools

- `diagnostics/run_compensated_freewheel_trial.py`
- `diagnostics/analyze_compensated_freewheel.py`
- `diagnostics/plot_compensated_freewheel.py`
- `diagnostics/simulate_compensated_freewheel.py`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/01_digital_twin/pulses_on`.
