# Pitch PD plus outer encoder position/velocity loop

Status: awaiting new inputs and protocol verification.

## Required figures

- Pitch and outer-loop pitch target
- Position/velocity errors and applied PWM
- Comparison with pitch-only PD under matched conditions

## Required inputs and checks

- User confirmed position/velocity outer loop, separate from friction LUT
- Python outer loop exists; equivalent real C++ implementation and parity checks still required
- Same frozen Stribeck plant for both simulated controllers; same declared friction-compensation setting

## Existing tools

- `src_python/balance_controller.py`
- `src_python/mujoco_sim.py`

Record every run in a fresh subdirectory of `<platform>/runs/`.
Keep raw CSV, status, model/controller snapshots, measured voltage,
startup/fixture, compensation setting, encoder/filter details and stop evidence.
A reusable source is referenced by path and SHA256, not copied and relabeled.
A CSV file is an input candidate, not an automatically validated result.

Figure output: `/home/vittorio/terasic_robot/deliverables/final_campaign/03_pd_outer_loop/stand`.
