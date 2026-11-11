# Matched LQR path comparison

Status: awaiting new inputs and protocol verification.

## Required figures

- Full signal overlay and signed errors
- Reference/odometry trajectory comparison and tracking metrics

## Required inputs and checks

- Reuse real LQR path acquisition
- Same reference timeline, limits, estimator and compensation setting

## Existing tools

- `scripts/digital_twin.py`

## Reused acquisition references

- `06_lqr_real/path`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/07_lqr_twin_comparison/path`.
