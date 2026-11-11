# Terasic self-balancing robot

Run PD, LQR, H-infinity and frozen GA-LQR in local MuJoCo, compile the C++
controllers for a Terasic DE10-Nano, and collect real robot logs.
Hardware, compiler versions, Python dependencies and the USB Wi-Fi setup are
listed in [requirements](docs/REQUIREMENTS.md).

## Usage guides

- [MuJoCo: installation and commands for each test](README_MUJOCO.md)
- [Real robot: initial setup, SSH, transfer and tests](README_REAL_ROBOT.md)

## 1. Download and install on the PC

On GitHub choose **Code → Download ZIP**, extract it, then open a terminal in
the extracted project root (the directory containing this README).
Alternatively clone the repository:

```sh
git clone https://github.com/vitlisi/TERASIC.git
cd TERASIC
```

Reference environment: Ubuntu 22.04, Python 3.10.12 and g++ 11.4.0.
On Ubuntu install the system tools, then the Python environment:

```sh
sudo apt update
sudo apt install python3 python3-venv python3-pip build-essential \
  openssh-client avahi-utils picocom usbutils iproute2 network-manager \
  libgl1 libglfw3
python3 -m venv env
env/bin/python -m pip install -r requirements.txt
```

The viewer needs a working graphical desktop and OpenGL. Omit `--viewer` for
headless simulations. Python packages are installed on the PC; the real board
runs C++ and does not require this Python environment.

## 2. Where each run saves its files

New commands below write directly into one directory per controller and source:

```text
logs/
  real/
    pd/
    pd-outer/
    lqr/
    hinf/
    ga/
  mujoco/
    pd/
    pd-outer/
    lqr/
    hinf/
    ga/
```

Choose the source first (`real` or `mujoco`), then the controller. A normal run creates **no run subfolder**.
Timestamp and unique identifier distinguish repeated runs; `stand`/`path` and
real `shadow`/`run` are in filenames. The launcher prints the output paths.

| File | Meaning |
|---|---|
| `*.csv` | Recorded time series; retain the whole trace |
| `*.json` | Simulation configuration, model metadata or summary |
| `*experiment.json` | Simulation run ID, exit code and list of output files |
| `*.status.log` | Real startup, battery checks, diagnostics, stop reason and exit code |
| `*.config.txt` | Snapshot of the real LQR/H-infinity/GA configuration |
| `*.args.txt` | PD command arguments, including gains, signs and reference |

Keep all files with the same run prefix together. MuJoCo plots are published
under `logs/mujoco/<controller>/plot/`. Real plots go in
`logs/real/<controller>/plot/`. SVG, PNG and PDF share this single plot folder.
Optional `--name my_test` adds a filename label; it does not create directories.
`ROBOT_LOG_DIR` can override the real output directory.

Existing acquisitions have been moved into this layout too. Their filenames
include the original test type and a stable identifier to keep repeated tests
separate. Digital-twin calibration data are under `logs/real/digital_twin/`;
associated model analyses are under `logs/mujoco/digital_twin/`.
These calibration datasets retain their internal structure because the report
scripts consume related groups of measurements.

GA optimization uses `logs/mujoco/ga/optimization/` or
`logs/mujoco/ga/uncertainties_2pct/`: each search retains checkpoints, candidate
history, common selection and held-out validation. Ordinary frozen GA controller
runs save directly under `logs/mujoco/ga/` or `logs/real/ga/`.
The ±2% search has completed; its final results and configuration are saved in
[the GA result summary](logs/mujoco/ga/uncertainties_2pct/ga_20261110_113003_e69593/RESULTS.md).
Plots are in `logs/mujoco/ga/plot/`; a checksummed backup is in `archives/ga/`.
The temporary old-path compatibility link has been removed.
A migration inventory is in `maintenance/log_layout_migration/manifest.json`.
Raw log contents are preserved; old paths inside acquisition metadata describe
the original recording location. Existing figures have also been moved into the corresponding `plot/` folders. See [logs/README.md](logs/README.md).

## 3. MuJoCo runs — PC terminal

Run these commands from the project root. Stand runs last 10 s by default.

```sh
# Pitch PD and PD with outer position/velocity loop (stand only).
env/bin/python -B scripts/run_experiment.py simulate --controller pd --viewer
env/bin/python -B scripts/run_experiment.py simulate --controller pd-outer --viewer

# Stabilization with LQR, H-infinity and frozen GA gains.
env/bin/python -B scripts/run_experiment.py simulate --controller lqr --case stand --viewer
env/bin/python -B scripts/run_experiment.py simulate --controller hinf --case stand --max-pwm 12 --viewer
env/bin/python -B scripts/run_experiment.py simulate --controller ga --case stand --viewer

# 40 cm / 180 degrees / 40 cm: profile 28.5 s, total simulation 33 s.
env/bin/python -B scripts/run_experiment.py simulate --controller lqr --case path --viewer
env/bin/python -B scripts/run_experiment.py simulate --controller hinf --case path --viewer
env/bin/python -B scripts/run_experiment.py simulate --controller ga --case path --viewer
```

Options include `--duration 10 --initial-error -3.5 --voltage 12 --seed 0
--max-pwm 20`. Select an explicit C++ controller profile with
`--config config/controllers/NAME.txt`. See `simulate --help` for supported
ranges. Record the actual voltage, limit, seed and config for comparisons.

The LQR/H-infinity/GA routes execute their C++ control cores. The two PD
previews are independent Python implementations, not firmware parity tests.
The model uses distinct SX/DX motor parameters and Stribeck friction;
friction compensation is OFF unless explicitly enabled in a separate experiment.
H-infinity path and frozen GA path are experimental simulations, not validated
physical paths. MATLAB/Simulink is separate and is not installed by pip.

## 4. Configure a new physical robot

Follow [new robot configuration](docs/NEW_ROBOT_CONFIGURATION.md) and, for a
wired first connection, [direct Ethernet with DHCP](docs/ETHERNET_DIRECT_DHCP.md).
The reference board uses Linux 4.5.0 ARMv7, an MT7601U USB dongle (`148f:7601`),
firmware `mt7601u.bin`, ConnMan, Avahi and SSH. Connect PC and board to the same
LAN and verify the board's SSH host-key fingerprint through UART before SSH.
Use its actual DHCP address; historical IP addresses are not portable.

The C++ code runs on the **HPS ARM processor** of the DE10-Nano and accesses
FPGA peripherals. Transferring C++ sources does not program the FPGA. The SD
image, FPGA bitstream and device tree must already match this robot's register
map. The setup checker does not install or certify a bitstream.
A new specimen requires its own verified IMU/motor/encoder signs, geometry,
counts per revolution, pitch reference, motor calibration, supply and limits.
The supplied gains and calibration belong to the reference robot.

## 5. Transfer and compile — PC, then board terminal

On the **PC**, package current sources rather than copying the Python
environment or the complete dataset to the small board filesystem:

```sh
BUNDLE="terasic_board_runtime_$(date +%Y%m%d_%H%M%S).tar.gz"
env/bin/python -B scripts/package_board_runtime.py --output "deliverables/$BUNDLE"
BOARD=root@ACTUAL_BOARD_ADDRESS
ssh "$BOARD" 'hostname; uname -a; df -h /home/root'
scp "deliverables/$BUNDLE" "deliverables/$BUNDLE.sha256" "$BOARD":/home/root/
ssh "$BOARD"
```

Replace `ACTUAL_BOARD_ADDRESS` before running. On the **board SSH terminal**,
set `BUNDLE` to the exact filename printed on the PC:

```sh
cd /home/root
BUNDLE=terasic_board_runtime_REPLACE_WITH_TIMESTAMP.tar.gz
sha256sum -c "$BUNDLE.sha256"
# Fresh install only: stop if this directory already exists.
test ! -e terasic_robot_final || { echo 'Use a fresh staging directory'; exit 1; }
tar -xzf "$BUNDLE"
cd terasic_robot_final
sha256sum -c SHA256SUMS
sh scripts/inspect_robot_setup.sh my_robot
sh scripts/check_robot_setup.sh my_robot wlan0
sh scripts/build_robot.sh
```

The archive contains sources, controller profiles, launchers and hashes.
Compilation creates ARM programs in `build/robot/` and does not start motors.
If updating an existing installation, extract into a separate staging directory
and review it before replacing files. A PC build produces PC binaries, not ARM.

## 6. Real runs — directly on the board (socfpga console or SSH)

At the board prompt (for example `root@socfpga`), execute the commands
locally. UART console and SSH provide access to the same board shell; do not
prefix these launch commands with `ssh`. First enter the runtime directory:

```sh
cd /home/root/terasic_robot_final
```

Then perform a motor-off check. For PD
this mode is called `observe`; for the other launchers it is `shadow`.

```sh
sh scripts/run_pd_logged.sh pd observe 3 0.320
sh scripts/run_pd_logged.sh pd-outer observe 3 0.320
sh scripts/run_lqr_mimo_logged.sh shadow config/controllers/lqr_mimo_shadow.txt 3 15
sh scripts/run_hinf_logged.sh shadow config/controllers/hinf_stand.txt 3 12
sh scripts/run_lqr_ga_logged.sh shadow config/controllers/ga_stand.txt 3 20
```

The following armed commands are reference-robot experiments. Before running,
verify calibration, voltage and configuration, keep the area clear, wheels on
the ground, chassis free to recover and an operator ready to catch it. Support
it before stopping. Every armed run can fall. Network connectivity, a compiled
binary or a successful simulation does not establish physical readiness.

```sh
# PD: gyro sign, motor sign, Kp, Kd, PWM cap, seconds, pitch reference.
sh scripts/run_pd_logged.sh pd run 1 1 5 0.8 15 10 0.320
# Outer PD adds Kx=2 and Kv=8.
sh scripts/run_pd_logged.sh pd-outer run 1 1 5 0.8 15 10 0.320 2 8

# Stand stabilization.
sh scripts/run_lqr_mimo_logged.sh run config/controllers/lqr_mimo.txt 10 20
sh scripts/run_hinf_logged.sh run config/controllers/hinf_stand.txt 10 12
sh scripts/run_lqr_ga_logged.sh run config/controllers/ga_stand.txt 10 20

# LQR path: 28.5 s profile plus 4.5 s final hold in the current binary.
sh scripts/run_lqr_path_simple_logged.sh run config/controllers/lqr_path_40cm_28p5s.txt 28.5 20
# Experimental frozen GA path: records the 28.5 s profile only.
sh scripts/run_lqr_ga_path_logged.sh run config/controllers/ga_path.txt 28.5 20
```

Real PD trials stopped at the attitude limit after about 3.57 s and 4.69 s.
Hold the robot motionless during roughly 2 s calibration near its verified
fixed reference; the startup guard rejects error above 5 degrees.
H-infinity stand at 12% completed one 10 s trial and failed another: it is not
a reliable tuning for an arbitrary robot. H-infinity path remains **shadow
only**; inspect the experimental candidate without arming:

```sh
sh scripts/run_hinf_path_logged.sh shadow config/controllers/hinf_path_40cm_28p5s_yaw_candidate.txt 28.5 12
```

All logged armed launchers require `build/robot/battery_status`. Inspect the
printed `.status.log`, `exit_code=` and stop reason after each run. A zero-byte
CSV usually means startup or battery checks blocked control. PD records
arguments in `.args.txt`; the other controllers snapshot their TXT profile.
Launchers run in the foreground and propagate interruption to the child;
keep the board terminal open during acquisition, whether using UART or SSH.

## 7. Retrieve real logs and create figures — PC terminal

After the physical program has stopped, use a PC terminal in the project root.
Choose the controller and copy its flat real log directory:

```sh
BOARD=root@ACTUAL_BOARD_ADDRESS
CONTROLLER=lqr
mkdir -p logs/real
scp -r "$BOARD:/home/root/terasic_robot_final/logs/real/$CONTROLLER" logs/real/
```

Transfer is manual. The board stores real logs only; MuJoCo and plots belong
on the PC. Copy after the run ends. Set `CONTROLLER` to `pd`, `pd-outer`, `lqr`,
`hinf` or `ga`. See [the real robot guide](README_REAL_ROBOT.md#9-logs-transfer-and-plots--pc-terminal)
for our reference SSH command and copying all controllers.
Select the actual CSV filename printed by the launcher:

```sh
env/bin/python -B scripts/plot_real.py logs/real/lqr/ACTUAL_FILE.csv \
  --output-dir logs/real/lqr/plot
```

The plotter creates SVG/PDF/PNG. The CSV filename is used as the plot basename, so repeated runs keep distinct plots.
For a matched real versus MuJoCo comparison, simulate the real trial's config,
voltage, duration, startup and PWM cap first, then supply both CSVs:

```sh
env/bin/python -B scripts/run_experiment.py compare --name lqr \
  --simulation logs/mujoco/lqr/ACTUAL_SIMULATION.csv \
  --real logs/real/lqr/ACTUAL_REAL.csv
```

The comparison tool requires the real `.config.txt` and `.status.log` next to
the CSV. Comparisons are separate derived analyses with their own output ID.
Encoder position is odometry, not independent ground truth; torque is model-derived.

## 8. GA optimization and historical figures

Running the frozen GA controller above does not rerun the optimizer. To start
an offline optimization with motor uncertainties ±2%, on the PC:

```sh
env/bin/python -u -B scripts/ga/tune_lqr_ga.py \
  --config config/ga/lqr_motor_uncertainties.json \
  --output-dir logs/mujoco/ga/uncertainties_2pct --no-plots
```

This uses 12 individuals, 10 generations, 5 scenarios resampled each generation,
11 common selection scenarios and 11 held-out validation scenarios. PWM cap is
50%, duration 10 s, initial pitch 3 degrees and feedback delay one sample.
Its costs cannot be directly compared with the separate 20% PWM search.
Do not start a duplicate if this campaign is already running.
Every search retains its own folder; this is necessary to preserve checkpoints
and the separation of training, selection and validation. No hardware profile
is replaced automatically.

Regenerate the retained digital-twin report figures without another acquisition:

```sh
env/bin/python -B scripts/plot_digital_twin_report.py
```

Digital-twin report figures are in `logs/real/digital_twin/plot/`.
See [data formats](docs/DATA_FORMATS.md), [offline checks](tests/README.md),
[GA tools](scripts/ga/README.md) and [digital-twin details](scripts/DIGITAL_TWIN.md).
