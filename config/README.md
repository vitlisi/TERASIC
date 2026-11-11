# Configuration files

Changing these files does not automatically configure the real robot.

| Path | Contents |
|---|---|
| `mujoco_sensors_mpu6500.json` | Simulated IMU/encoder rates, noise, delay and counts |
| `mujoco_motors_sx_dx_20261030.json` | Separate left/right effective motor profiles |
| `mujoco_battery_3s_3000mah.json` | Assumed battery/discharge model, not a calibrated fuel gauge |
| `mujoco_ultrasonic_hcsr04.json` | Range, cone and sampling assumptions |
| `controllers/` | Real C++ key/value TXT profiles, distinct from simulation JSON |
| `paths/` | XY reference geometry, duration and guidance limits |
| `ga/` | Population, seed, Q/R search and evaluation protocol |
| `references/` | Original datasheets/manuals |

Read provenance fields before interpreting a simulated parameter as a measured physical quantity. The frozen Stribeck profile is `mujoco_robot01_stribeck.json`; compensation inputs live in `motors/`. Exact retained input hashes and final campaign status are in [the model manifest](final_campaign_model.json). Local qualification records live in `qualification/` and remain evidence for their original binaries, not blanket qualification of future builds.
