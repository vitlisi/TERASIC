# Simulated XY path configuration

Edit `curve_test.json`: `waypoints_xy_m` are local[X,Y] points in metres relative to release; the first is[0,0]. +Y is initially forward, +X right. A C2 cubic spline interpolates the points; it can depart from straight connecting segments, and near-zero tangents/cusps are rejected. `end_heading_deg` is relative to the initial heading (0=+Y,90=−X).

`start_s` is waiting time; `travel_duration_s` excludes that wait. Quintic timing starts/ends with zero speed/acceleration; along-path speed is not constant. Guidance uses estimated pose, never simulator truth. `guidance.lateral_gain_rad_per_m` and `max_heading_correction_deg` bound heading correction; positive numeric speed/yaw-rate limits reject excessive references rather than rescaling them. `null` disables those optional checks. Decorative viewer geometry does not generate contact forces.

```sh
env/bin/python src_python/mujoco_sim.py --mode lqr-path --path-file config/paths/curve_test.json --path-duration-s 8 --duration 15 --headless --exit-on-stop
env/bin/python scripts/validate_path.py --path-file config/paths/curve_test.json
env/bin/python -m unittest discover -s tests -p 'test_path_reference.py' -v
```

Inspect `--help` before changing reference/stop settings. The timed real-robot path is a different configuration; these JSON files do not deploy hardware behavior. Short simulations can end before post-path settling is assessed. Model, contact and sensor assumptions remain approximations.

## Retained evidence links

- [curve_test.json](curve_test.json)

## Historical commands and identifiers

These commands belong to the dated protocol, not a new physical-run authorization. Review actual CLI help and archived supply/identity assumptions before reuse.

```bash
env/bin/python src_python/mujoco_sim.py --mode lqr-path \
  --path-file config/paths/curve_test.json --path-duration-s 8 --duration 14 \
  --imu-model mpu6500 --encoder-model motor \
  --battery-discharge --max-pwm 100 --tag test_velocita
```

```bash
env/bin/python src_python/mujoco_sim.py --mode lqr-path \
  --path-file config/paths/curve_test.json --duration 40 \
  --imu-model mpu6500 --encoder-model motor \
  --battery-discharge --tag percorso_curve
```

```bash
cp config/paths/curve_test.json config/paths/mio_percorso.json
```

```bash
env/bin/python src_python/mujoco_sim.py --mode lqr-path \
  --path-file config/paths/curve_test.json --duration 40 \
  --imu-model mpu6500 --encoder-model motor --battery-discharge \
  --path-obstacle --path-obstacle-progress 0.65 --tag path_ultrasuoni
```

```bash
env/bin/python scripts/validate_path.py --path-file config/paths/curve_test.json
env/bin/python -m unittest discover -s tests -p 'test_path_reference.py' -v
python3 scripts/plot_path.py logs/mujoco/PERCORSO.csv
```


## Retained field/file identifiers

`end_heading_deg`, `start_s`, `travel_duration_s`, `waypoints_xy_m`.

The original README is preserved at its repository-relative path inside `maintenance/review_20261107/original_readmes.tar.gz`. Numerical/source evidence is not rewritten by this English guide.
