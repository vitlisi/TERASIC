# GA versus baseline: timed path

Status: awaiting new inputs and protocol verification.

## Required figures

- Full tracking/balance/PWM signals
- Trajectory and common-metric comparison

## Required inputs and checks

- Same path schedule and guidance/estimator conventions
- Use retained GA-specific delayed core; no new optimization implicitly scheduled
- Independent validation is distinct from selection/training performance

## Existing tools

- `scripts/simulate_ga_twin.py`
- `scripts/digital_twin.py`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/09_ga/path`.
