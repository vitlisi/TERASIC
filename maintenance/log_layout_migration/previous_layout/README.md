# Final report campaign

Prepared folders are not completed experiments. No acquisition runs automatically.

Figures: English; full recorded traces from t=0; separate additional zooms.

| Section | Test | Input requirement |
|---|---|---|
| Digital twin and unloaded motor validation | `01_digital_twin/model_evolution` | real, mujoco_nominal, mujoco_stribeck |
| Digital twin and unloaded motor validation | `01_digital_twin/hysteresis_off` | real, mujoco |
| Digital twin and unloaded motor validation | `01_digital_twin/pulses_off` | real, mujoco |
| Digital twin and unloaded motor validation | `01_digital_twin/pulses_on` | real, mujoco |
| Digital twin and unloaded motor validation | `01_digital_twin/hysteresis_on` | real, mujoco |
| Real pitch PD | `02_pd_real/stand` | real |
| PD with outer position/velocity feedback | `03_pd_outer_loop/stand` | real, mujoco |
| LQR stabilization in simulation | `04_lqr_simulation/stand` | mujoco, simulink |
| H-infinity stabilization in simulation | `05_hinf_simulation/stand` | mujoco, simulink |
| Real LQR performance | `06_lqr_real/stand` | real |
| Real LQR performance | `06_lqr_real/path` | real |
| Real versus MuJoCo LQR | `07_lqr_twin_comparison/stand` | real, mujoco |
| Real versus MuJoCo LQR | `07_lqr_twin_comparison/path` | real, mujoco |
| Real versus MuJoCo H-infinity | `08_hinf_twin_comparison/stand` | real, mujoco |
| Real versus MuJoCo H-infinity | `08_hinf_twin_comparison/path` | real, mujoco |
| GA-selected LQR versus baseline | `09_ga/stand` | mujoco_baseline, mujoco_ga |
| GA-selected LQR versus baseline | `09_ga/path` | mujoco_baseline, mujoco_ga |
