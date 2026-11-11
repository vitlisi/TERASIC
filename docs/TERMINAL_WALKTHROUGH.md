# Terminal walkthrough and campaign run register

We execute one step at a time and retain actual results. Preparing folders is
separate from running an experiment. The operator runs the agreed commands;
no armed test is launched merely because SSH is connected.

## Reference-board observations — 9 November 2026

- SSH identity: `socfpga`, Linux 4.5.0, ARMv7; board `g++` available.
- No active controller process was found in the latest read-only inspection.
- Approximately 121 MiB filesystem space available; Python 3 was not found.
- Existing root-project PD/LQR/H-infinity binaries date from October/early November.
  Build the current source bundle separately before the new controller campaign.
- Historical digital-twin data recovered; 11 English SVGs already generated.
- PD `observe_20261109_01`: 300 samples, 3 s observation, zero PWM and zero
  encoder movement, final motor control 0x06 on both wheels. This is not a
  balance-performance run.

## 1. SSH from the current workstation

For this already verified reference board on this workstation:

```sh
ssh -o HostKeyAlias=192.168.50.2 root@10.185.186.166
```

The current address is an observed DHCP address, not a permanent assignment.
The alias is specific to the public key already verified on this workstation.
For another board, verify its own SSH fingerprint through UART and use its actual
address without borrowing this alias; see the general setup guide.

After connecting, these checks do not start motors:

```sh
hostname
uname -srmo
pwd
df -h /home/root
```

## 2. Transfer the lightweight board sources

For a fresh extraction of the full PC project, first create the runtime archive
if it is not already provided:

```sh
env/bin/python -B scripts/package_board_runtime.py
```

Then run on the PC in the project root, in a second terminal:

```sh
scp -o HostKeyAlias=192.168.50.2 \
  deliverables/terasic_board_runtime.tar.gz \
  deliverables/terasic_board_runtime.tar.gz.sha256 \
  root@10.185.186.166:/home/root/
```

This archive contains current C++ sources/headers, controller configurations and
shell launchers. It excludes the PC environment, datasets, figures and PC binaries.
Do not transfer the full report archive to this board with its limited free space.
The source bundle is created on the PC using:

```sh
env/bin/python -B scripts/package_board_runtime.py --output deliverables/terasic_board_runtime_v2.tar.gz
```

Use a new output filename for a new version; the packager refuses overwrites.

## 3. Verify, extract and compile on the board

From the board SSH terminal:

```sh
cd /home/root
sha256sum -c terasic_board_runtime.tar.gz.sha256
```

Only if `/home/root/terasic_robot_final` does not already exist:

```sh
test ! -e /home/root/terasic_robot_final || { echo 'Destination already exists; inspect it first.'; exit 1; }
tar -xzf terasic_board_runtime.tar.gz
cd /home/root/terasic_robot_final
sha256sum -c SHA256SUMS
sh scripts/build_robot.sh
```

This preserves `/home/root/terasic_robot` and its original measurements.
Compilation does not start any motor program. The new controller binary hashes
and configuration-check results will be recorded before a physical trial.
The real Python remote qualification helpers target their original qualified
builds; they are not an automatic authorization for these new binaries.

## 4. MuJoCo from a PC terminal

After the board installation step has been checked, simulations can run on the
PC independently. Example with the real LQR C++ core:

```sh
env/bin/python -B scripts/run_experiment.py simulate \
  --controller lqr --case stand \
  --name 07_lqr_twin_comparison/stand --viewer
```

For LQR path, the controller TXT fixes a 28.5 s motion profile; MuJoCo runs
for 33 s by default so the post-profile balance is also recorded:

```sh
env/bin/python -B scripts/run_experiment.py simulate \
  --controller lqr --case path --duration 33 \
  --name 07_lqr_twin_comparison/path
```

The command prints a RUN_ID and creates:

```text
logs/final_campaign/<NAME>/mujoco/runs/<RUN_ID>/
deliverables/final_campaign/<NAME>/<RUN_ID>/
```

Use `--name custom/another_test` for a different experiment. Other controller
commands are in the root README. A simulation preview is not a matched physical
validation until its configuration, initial conditions, supply and compensation
setting agree with a completed real acquisition.

## 5. Real acquisition and plots

Each active trial is prepared separately with the current fixture and readiness.
The original board LQR path binary continued balancing after its 28.5 s motion
profile until manually stopped. The corrected source now limits the armed run
to 33 s total (28.5 s profile plus 4.5 s final balance); this behavior applies
only after the corrected binary is rebuilt and installed on the board. The
historical 57 s recording retains its original duration and provenance.

To update the existing board installation without replacing its logs or other
controllers, copy the one changed source from a PC terminal:

```sh
scp -o HostKeyAlias=192.168.50.2 \
  src_cpp/terasic_reale/controllers/balance_lqr_path_simple.cpp \
  root@10.185.186.166:/home/root/terasic_robot_final/src_cpp/terasic_reale/controllers/
```

Then compile and verify from the board terminal:

```sh
cd /home/root/terasic_robot_final
sha256sum src_cpp/terasic_reale/controllers/balance_lqr_path_simple.cpp
test ! -e build/robot/path_simple_33s_candidate
test ! -e build/robot/path_simple_before_33s_20261109
g++ -std=c++98 -O2 -Wall -Wextra -Werror \
  src_cpp/terasic_reale/controllers/balance_lqr_path_simple.cpp \
  -o build/robot/path_simple_33s_candidate -lrt
./build/robot/path_simple_33s_candidate check config/controllers/lqr_path_40cm_28p5s.txt
cp -p build/robot/path_simple build/robot/path_simple_before_33s_20261109
mv build/robot/path_simple_33s_candidate build/robot/path_simple
sha256sum build/robot/path_simple build/robot/path_simple_before_33s_20261109
```

Expected source SHA256 is
`8efe40e62b4a9c02d3a115b278ca5ca03baddcde5671b91c889d588cab599276`.
The earlier `c5a5001e...` source used for the already recorded LQR 33 s trial
did not yet bound the H-infinity path. The installed LQR binary remains a valid
artifact of that trial; the current shared source adds the same 33 s limit to
the H-infinity path when it is separately rebuilt.
Compilation and `check` do not actuate motors. The corrected runtime prints the
33 s limit in its status log. The old log remains under its original run ID.
The board launcher will receive an explicit fresh `ROBOT_LOG_DIR` under:

```text
/home/root/terasic_robot_final/logs/final_campaign/<TEST>/real/runs/<RUN>/
```

It saves CSV, status and controller snapshot. On the PC, retrieve the entire run
folder, then run `scripts/plot_real.py` for English SVG/PDF/PNG. For a compatible
real/MuJoCo pair, use `scripts/run_experiment.py compare`. The exact filenames
printed by each run will be added to this register and the relevant report test
folder. No missing results will be replaced by example data.

## Run register

| Step | State | Evidence |
|---|---|---|
| Historical digital-twin figures | Complete; no new acquisition | `deliverables/final_campaign/01_digital_twin/historical_report/` |
| PD observe, motors disabled | Complete; not performance validation | `logs/final_campaign/02_pd_real/stand/real/runs/observe_20261109_01/` |
| Current SSH connection/read-only inventory | Verified | `maintenance/log_layout_migration/previous_layout/session_checks/ssh_inventory_20261109.txt` |
| Current board runtime source bundle | Prepared on PC | `deliverables/terasic_board_runtime_v6.tar.gz` |
| Bundle transfer and ARM compilation | Operator completed on board | New runtime LQR and H-infinity logs received from `/home/root/terasic_robot_final/` |
| LQR stand | Recorded, 10 s, exit code 0 | `logs/final_campaign/06_lqr_real/stand/real/runs/trial_001/` |
| LQR path | One profile completed; control continued to about 57 s and was interrupted; other attempts stopped early | `logs/final_campaign/06_lqr_real/path/real/runs/trial_002/` |
| H-infinity stand | One complete 10 s run at 12% cap after a failed 12% attempt; prior 15% and 20% attempts also failed | `logs/final_campaign/08_hinf_twin_comparison/stand/real/runs/trial_001_retest/` |
| H-infinity path | Offline yaw reserve 6% completed 33 s across four starts with about 1 cm return error; real candidate remains `shadow_only 1` and not validated | `docs/HINF_PATH_YAW_RESERVE_20261110.md` |
| PD pitch-only | Recorded; attitude stop after about 3.57 s | `logs/final_campaign/02_pd_real/stand/real/runs/trial_001/` |
| PD outer loop | Recorded; attitude stop after about 4.69 s | `logs/final_campaign/03_pd_outer_loop/stand/real/runs/trial_002/` |
