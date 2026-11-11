# Real pitch-only PD stabilization

Status: awaiting new inputs and protocol verification.

## Required figures

- Pitch/reference and pitch angular velocity
- Left/right applied PWM, odometry position and velocity

## Required inputs and checks

- New real trial with pitch-only PD; fixed gains, reference, limits and startup documented
- Do not claim real ground-truth position from encoder odometry

## Existing tools

- `scripts/run_pd_offline.sh`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/02_pd_real/stand`.
