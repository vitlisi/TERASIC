# Real LQR timed path

Status: awaiting new inputs and protocol verification.

## Required figures

- Full pitch/rate/PWM and path-reference tracking
- Odometry trajectory and final odometry error

## Required inputs and checks

- New real path trial; same controller/reference as MuJoCo
- Real odometry is not independent slip/ground-truth validation

## Existing tools

- `scripts/run_lqr_path_simple_logged.sh`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/06_lqr_real/path`.
