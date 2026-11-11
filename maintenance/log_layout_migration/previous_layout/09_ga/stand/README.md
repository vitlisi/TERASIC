# GA versus baseline: stand stabilization

Status: awaiting new inputs and protocol verification.

## Required figures

- Full pitch/rate/PWM/odometry overlay
- Common-window performance and saturation metrics

## Required inputs and checks

- Retained transferred GA gains and one-sample feedback delay
- Same physical model, initial conditions, voltage, noise seed, duration and PWM limits
- State the chosen delay policy; do not attribute a delay difference solely to gain optimization
- Report measured improvement or deterioration, without assuming GA wins

## Existing tools

- `scripts/simulate_ga_twin.py`
- `scripts/digital_twin.py`
- `scripts/ga/compare_lqr_ga.py`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/09_ga/stand`.
