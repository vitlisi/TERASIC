# Real LQR stand stabilization

Status: awaiting new inputs and protocol verification.

## Required figures

- Full recovery and stabilization signals
- Pitch/rate RMS, drift and PWM metrics with declared windows

## Required inputs and checks

- New autonomous real trial
- Pin controller and friction-compensation setting

## Existing tools

- `scripts/run_lqr_mimo_logged.sh`
- `diagnostics/run_guarded_balance_trial.py`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/06_lqr_real/stand`.
