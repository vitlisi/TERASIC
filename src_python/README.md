# MuJoCo simulator and Python controllers

See [installation and simulation commands](../README.md#installation-and-offline-simulation).

| File | Role |
|---|---|
| `mujoco_sim.py` | CLI, simulated experiments and logging |
| `mujoco_robot.py` | Physical model, sensor interfaces, PWM-to-actuator conversion |
| `balance_controller.py` | PD and shared C++ estimator/controller bridge |
| `motion_lqr.py` | Six-state/two-input model and LQR |
| `ga_lqr.py` | Q/R genes, uncertainty scenarios, evaluation and GA operators |
| `hinf_controller.py` | Imported dynamic MATLAB controller, state/wheel transformation and delay |
| `lqr_controller.py` | Riccati solver and longitudinal baseline, still imported by the six-state design |
| `path_reference.py` | Path references and stop requests |
| `sensor_models.py`, `battery_model.py`, `ultrasonic_model.py` | Explicit sensor, supply and ranging approximations |
| `viewer_hud.py` | Viewer overlays |

Physics runs at 1 kHz and control at 100 Hz. Simulated sensor estimates are
separate from geometric truth used for evaluation. The Python six-state model
uses wheel angular coordinates, pitch/yaw and rates; the real MIMO core uses
longitudinal distance, pitch/yaw and rates. Use the explicit transformations;
do not copy gains between these coordinate systems directly.

`--mode lqr-motion --motion-profile real-path` follows the timed 40 cm /180° /
40 cm sequence; `--mode lqr-path` uses the XY path configuration. These are
simulation modes and do not install hardware controllers. New manual records
are under `logs/mujoco/runs/`; model assumptions are in `models/` and `config/`.

Current motor physics: `stribeck_motor.py`, with distinct SX/DX parameters in `config/mujoco_robot01_stribeck.json`. Use the real-core twin for matched final comparisons. No exploratory loaded correction was adopted.
