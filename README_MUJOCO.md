# MuJoCo — installation and test commands

Run all commands on the **PC**, from the repository root.
Full requirements: [hardware and software](docs/REQUIREMENTS.md).
For the physical board, use [README_REAL_ROBOT.md](README_REAL_ROBOT.md).

## Installation

Download the repository from GitHub using **Code → Download ZIP**, extract it,
and open a terminal in the directory containing `requirements.txt`.
Reference environment: Ubuntu 22.04, Python 3.10.12, g++ 11.4.0.

```sh
sudo apt update
sudo apt install python3 python3-venv python3-pip build-essential libgl1 libglfw3
python3 -m venv env
env/bin/python -m pip install -r requirements.txt
```

`--viewer` opens the 3D window and requires a desktop with working OpenGL.
Remove it to run headless. Each command below is a separate test; choose the
one you want to execute.

## Stand stabilization

Declared duration: 10 s; initial error: −3.5°; simulated supply: 12 V;
sensor seed: 0. PWM limits are percentages.

```sh
# Pitch PD.
env/bin/python -B scripts/run_experiment.py simulate --controller pd --case stand --duration 10 --initial-error -3.5 --voltage 12 --max-pwm 20 --seed 0 --viewer

# PD with outer position/velocity loop.
env/bin/python -B scripts/run_experiment.py simulate --controller pd-outer --case stand --duration 10 --initial-error -3.5 --voltage 12 --max-pwm 20 --seed 0 --viewer

# LQR.
env/bin/python -B scripts/run_experiment.py simulate --controller lqr --case stand --duration 10 --initial-error -3.5 --voltage 12 --max-pwm 20 --seed 0 --viewer

# H-infinity, PWM cap 12%.
env/bin/python -B scripts/run_experiment.py simulate --controller hinf --case stand --duration 10 --initial-error -3.5 --voltage 12 --max-pwm 12 --seed 0 --viewer

# GA-LQR with frozen gains already included in the project.
env/bin/python -B scripts/run_experiment.py simulate --controller ga --case stand --duration 10 --initial-error -3.5 --voltage 12 --max-pwm 20 --seed 0 --viewer
```

The PD previews are Python implementations independent of the real firmware.
LQR, H-infinity and GA use their respective C++ cores. The model includes
Stribeck friction and distinct SX/DX parameters; these commands do not enable
friction compensation.

## 40 cm / 180° turn / 40 cm path

Motion profile: 28.5 s; total simulated recording: 33 s.
The runner does not support PD paths. H-infinity and GA path runs remain
experimental evaluations and do not qualify a physical robot path.

```sh
# LQR.
env/bin/python -B scripts/run_experiment.py simulate --controller lqr --case path --duration 33 --max-pwm 20 --viewer

# H-infinity.
env/bin/python -B scripts/run_experiment.py simulate --controller hinf --case path --duration 33 --max-pwm 20 --viewer

# Frozen GA-LQR.
env/bin/python -B scripts/run_experiment.py simulate --controller ga --case path --duration 33 --max-pwm 20 --viewer
```

## Explicit configuration and repeated tests

To reproduce a real test, use its TXT configuration snapshot, voltage,
duration, PWM cap and initial conditions. `--config` applies to LQR/H-infinity/GA.
Example using the supplied LQR profile:

```sh
env/bin/python -B scripts/run_experiment.py simulate --controller lqr --case stand --config config/controllers/lqr_mimo.txt --duration 10 --max-pwm 20 --name prova_lqr_01 --viewer
```

`--name` adds a label to filenames. Each execution receives a unique ID;
there is no need to create a directory for each test.

```sh
env/bin/python -B scripts/run_experiment.py simulate --help
```

## Log and plot locations

| Controller | Logs | SVG/PDF/PNG images |
|---|---|---|
| PD | `logs/mujoco/pd/` | `logs/mujoco/pd/plot/` |
| PD outer | `logs/mujoco/pd-outer/` | `logs/mujoco/pd-outer/plot/` |
| LQR | `logs/mujoco/lqr/` | `logs/mujoco/lqr/plot/` |
| H-infinity | `logs/mujoco/hinf/` | `logs/mujoco/hinf/plot/` |
| GA-LQR | `logs/mujoco/ga/` | `logs/mujoco/ga/plot/` |

Keep CSV, configuration and JSON files with the same prefix together.
`*experiment.json` lists the run files and exit code. Normal termination does
not imply that all settling criteria have been met.

## New GA optimization with ±2% uncertainties

This search is separate from `--controller ga`, which uses fixed gains.

```sh
env/bin/python -u -B scripts/ga/tune_lqr_ga.py \
  --config config/ga/lqr_motor_uncertainties.json \
  --output-dir logs/mujoco/ga/uncertainties_2pct --no-plots
```

Configuration: 12 individuals, 10 generations, 5 resampled scenarios per
generation, 11 common selection scenarios and 11 independent validation scenarios.
Uncertainties of ±2% affect motor inertia, viscous friction, joint friction and
SX/DX motor gain. IMU/encoder noise is included; duration is 10 s, initial pitch
3°, PWM cap 50%, and feedback delay one sample. Each search retains its own
checkpoint directory.

The completed result is documented in
[RESULTS.md](logs/mujoco/ga/uncertainties_2pct/ga_20261110_113003_e69593/RESULTS.md).
The gains from this new search **do not automatically replace** those used by
`--controller ga` or the real robot launchers.

## Historical digital-twin figures

Regenerate figures from saved measurements without acquiring data on the robot:

```sh
env/bin/python -B scripts/plot_digital_twin_report.py
```

Output: `logs/real/digital_twin/plot/`. Measurements are stored in
`logs/real/digital_twin/historical_inputs/`.

## Comparison with a real run

After simulating conditions matching the real run:

```sh
env/bin/python -B scripts/run_experiment.py compare --name lqr \
  --simulation logs/mujoco/lqr/NOME_SIMULAZIONE.csv \
  --real logs/real/lqr/NOME_REALE.csv
```

Replace the CSV filenames. Matching `.config.txt` snapshots and the robot's
`.status.log` are required. The comparison is saved in `logs/real/lqr/plot/`
with a new ID. This is not a firmware parity comparison for the PD previews.
