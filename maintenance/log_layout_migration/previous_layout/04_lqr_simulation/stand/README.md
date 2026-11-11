# LQR stabilization: MuJoCo and available Simulink export

Status: awaiting new inputs and protocol verification.

## Required figures

- Pitch/reference, pitch angular velocity, odometry, applied PWM
- Matched measured-state overlay if a compatible Simulink export is supplied

## Required inputs and checks

- Same state/input units, controller gains, limits and initial conditions
- MATLAB not found on this PC; Simulink run/export pending
- Do not present analytic linear simulation as nonlinear Simulink/Stribeck validation

## Existing tools

- `scripts/digital_twin.py`
- `maintenance/matlab/design_balance_lqr.m`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/04_lqr_simulation/stand`.
