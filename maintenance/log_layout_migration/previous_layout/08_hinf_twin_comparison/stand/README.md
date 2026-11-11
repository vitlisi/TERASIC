# Matched H-infinity stand comparison

Status: awaiting new inputs and protocol verification.

## Required figures

- Full signal overlay and signed errors
- Declared-window pitch/rate/PWM metrics

## Required inputs and checks

- New real H-infinity stand acquisition
- Select matching baseline/tuned core and configuration
- Compensation integration is not automatic; verify before selecting ON

## Existing tools

- `scripts/run_hinf_logged.sh`
- `scripts/digital_twin.py`

## Reused acquisition references

- `05_hinf_simulation/stand`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/08_hinf_twin_comparison/stand`.
