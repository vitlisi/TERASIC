# Real robot — initial setup, SSH and tests

This guide distinguishes the **PC terminal** from the **board terminal**.
If you are already at the `root@socfpga` prompt, for example through the UART
console, execute board commands directly there; there is no need to open SSH.
`socfpga` is the hostname; other robots may display a different prompt.
The C++ program runs on the DE10-Nano HPS ARM processor and accesses FPGA
peripherals. Transferring source code does not program the FPGA: the robot's
SD image, device tree and bitstream must match the expected register map.

Requirements: [hardware and versions](docs/REQUIREMENTS.md).
Detailed procedure: [new board setup](docs/NEW_ROBOT_CONFIGURATION.md).
Simulations: [README_MUJOCO.md](README_MUJOCO.md).

## 1. Prepare the PC and first console

On the **Ubuntu PC**, from the repository root:

```sh
sudo apt update
sudo apt install openssh-client avahi-utils picocom usbutils iproute2 network-manager python3-venv
python3 -m venv env
env/bin/python -m pip install -r requirements.txt
ls /dev/serial/by-id/
```

Connect the UART console supported by the board. Replace the example serial
device with the actual one. Console settings are 115200 baud, 8N1:

```sh
picocom --baud 115200 --databits 8 --parity n --stopbits 1 /dev/ttyUSB0
```

From the **board console**, without starting controllers:

```sh
hostname
uname -a
ip -br address
lsusb
g++ --version
ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
systemctl status connman avahi-daemon sshd.socket --no-pager
```

Record the SSH key fingerprint and compare it when first connecting from the PC.
If the image uses another host key or SSH unit, identify it through the console;
the names above refer to the reference configuration.

## 2. Initial Ethernet connection

Connect the PC and DE10 using Ethernet. The board must have a DHCP client running.
On the **PC**, identify the interface:

```sh
ip -br link
nmcli connection show
```

If the profile does not exist, create it once, replacing `INTERFACCIA_PC`
with the actual PC Ethernet interface name:

```sh
nmcli connection add type ethernet ifname INTERFACCIA_PC con-name terasic-direct-dhcp ipv4.method shared ipv4.addresses 192.168.50.1/24 ipv4.never-default yes ipv6.method disabled
nmcli connection up terasic-direct-dhcp
ip neigh
```

The `192.168.50.0/24` subnet must be free of conflicts. Read the board's IP
from its console using `ip -br address`; do not assume it is `192.168.50.2`.
Details: [direct Ethernet and DHCP](docs/ETHERNET_DIRECT_DHCP.md).

## 3. First SSH login

On the **PC**, replace the address with the board's actual IP and compare
the proposed fingerprint with the one read through UART:

```sh
BOARD=root@INDIRIZZO_IP_SCHEDA
ssh "$BOARD"
```

Example command used for our board from the reference PC:

```sh
ssh -o HostKeyAlias=192.168.50.2 root@10.185.186.166
```

`10.185.186.166` is a previously observed DHCP address; check the current IP
before reusing it. `HostKeyAlias=192.168.50.2` selects the host key already
verified and stored on our PC for that same board. It does not assign an IP
and must not be copied for a different board. On a new PC, verify the fingerprint
through UART and use the generic command above.

`root` is the account on the reference image; use the actual account if the
image differs. Do not store passwords or private keys in the repository.
After login, check `hostname` and `uname -a` to identify the board.

## 4. Wi-Fi used in this project

USB dongle: **MT7601U, ID 148f:7601**; WPA2 hotspot at **2.4 GHz**;
PC connected to the same LAN. On the **board**, check:

```sh
lsusb
ls -l /lib/firmware/mt7601u.bin
lsmod
modinfo mt7601u
```

Firmware `mt7601u.bin`, modules `cfg80211`, `mac80211`, `mt7601u` and their
dependencies must be compatible with the board's Linux 4.5.0 kernel. If missing,
follow the detailed setup guide before installing the package: a matching kernel
version number alone does not guarantee ABI compatibility. Eduroam/802.1X is
not the authenticated configuration verified in this project.

On the **board**, open:

```sh
connmanctl
```

Then enter the following at the interactive ConnMan prompt. Replace
`ID_SERVIZIO_MOSTRATO_DA_SERVICES` with the service ID from the current scan:

```text
enable wifi
scan wifi
services
agent on
connect ID_SERVIZIO_MOSTRATO_DA_SERVICES
```

Enter the password when prompted, then type `quit`. Check:

```sh
ip -br address
ip route
```

Choose a unique hostname for a new board; `robot02` is an example:

```sh
hostnamectl set-hostname robot02
systemctl enable --now avahi-daemon
```

If ConnMan changes the name, merge `AllowHostnameUpdates=false` into the
existing `[General]` section of `/etc/connman/main.conf`, as described in the
detailed guide. After reboot, verify Wi-Fi reconnection, DHCP and SSH.
From the **PC**, connect first by IP and then optionally by hostname:

```sh
ssh root@INDIRIZZO_WIFI_SCHEDA hostname
getent ahostsv4 robot02.local
ssh root@robot02.local hostname
```

Repeat the check with Ethernet disconnected. Wi-Fi provides SSH/SCP access;
the control loop remains local to the DE10.

## 5. Transfer sources — PC terminal

Do not transfer `env/` or the complete dataset to the microSD. Create a small
runtime package instead:

```sh
BUNDLE="terasic_board_runtime_$(date +%Y%m%d_%H%M%S).tar.gz"
env/bin/python -B scripts/package_board_runtime.py --output "archives/$BUNDLE"
BOARD=root@INDIRIZZO_IP_SCHEDA
ssh "$BOARD" 'hostname; uname -a; df -h /home/root'
scp "archives/$BUNDLE" "archives/$BUNDLE.sha256" "$BOARD":/home/root/
ssh "$BOARD"
```

## 6. Install and compile — directly on the board (socfpga)

Replace the bundle filename below with the one printed on the PC:

```sh
cd /home/root
BUNDLE=terasic_board_runtime_SOSTITUIRE_TIMESTAMP.tar.gz
sha256sum -c "$BUNDLE.sha256"
test ! -e terasic_robot_final || { echo 'Usare una nuova directory di staging'; exit 1; }
tar -xzf "$BUNDLE"
cd terasic_robot_final
sha256sum -c SHA256SUMS
sh scripts/inspect_robot_setup.sh robot02
sh scripts/check_robot_setup.sh robot02 wlan0
sh scripts/build_robot.sh
```

Use the actual hostname instead of `robot02`. For an existing installation,
extract into a separate staging directory and review changes before replacing
files. The build produces ARM executables under `build/robot/` without starting
motors. The language standard is C++98; record the board's `g++` version.

## 7. Motor-off checks — directly on the board (socfpga)

At the board's `root@socfpga` prompt, enter the runtime installation directory:

```sh
cd /home/root/terasic_robot_final
pwd
```

This path corresponds to the installation described above. If you extracted
the project elsewhere, use its actual directory containing `scripts/`,
`config/` and `build/robot/`. Then choose a motor-off check:

```sh
sh scripts/run_pd_logged.sh pd observe 3 0.320
sh scripts/run_pd_logged.sh pd-outer observe 3 0.320
sh scripts/run_lqr_mimo_logged.sh shadow config/controllers/lqr_mimo_shadow.txt 3 15
sh scripts/run_hinf_logged.sh shadow config/controllers/hinf_stand.txt 3 12
sh scripts/run_lqr_ga_logged.sh shadow config/controllers/ga_stand.txt 3 20
```

`0.320` is the reference robot's pitch reference, not a universal value.
A new board or robot requires verification of IMU, motor and encoder signs,
counts per revolution, geometry, pitch reference, power supply and limits.

## 8. Armed tests — directly on the board (socfpga)

Paste the following commands **directly into the DE10 terminal** at the
`root@socfpga` prompt, including when connected through UART. Do not prefix
them with `ssh` or copy the prompt text. First enter the runtime directory:

```sh
cd /home/root/terasic_robot_final
```

Execute one command at a time after checking the hardware and calibration.
Keep the wheels on the ground, the area clear and the chassis free to recover,
with an operator ready to support the robot before stopping. Every run can fall.
The parameters below reproduce reference-robot experiments; they do not qualify
other robots.

```sh
# PD: gyro sign, motor sign, Kp, Kd, PWM%, duration in s, pitch reference in degrees.
sh scripts/run_pd_logged.sh pd run 1 1 5 0.8 15 10 0.320

# PD outer: adds Kx=2 and Kv=8.
sh scripts/run_pd_logged.sh pd-outer run 1 1 5 0.8 15 10 0.320 2 8

# Stand LQR.
sh scripts/run_lqr_mimo_logged.sh run config/controllers/lqr_mimo.txt 10 20

# Experimental stand H-infinity.
sh scripts/run_hinf_logged.sh run config/controllers/hinf_stand.txt 10 12

# Frozen GA-LQR: does not automatically use the new ±2% result.
sh scripts/run_lqr_ga_logged.sh run config/controllers/ga_stand.txt 10 20

# LQR path: 28.5 s profile plus 4.5 s final hold.
sh scripts/run_lqr_path_simple_logged.sh run config/controllers/lqr_path_40cm_28p5s.txt 28.5 20

# Experimental GA path: records only the 28.5 s profile.
sh scripts/run_lqr_ga_path_logged.sh run config/controllers/ga_path.txt 28.5 20
```

PD and PD outer trials stopped at the attitude limit after approximately
3.57 s and 4.69 s. During PD calibration, hold the robot motionless near its
verified reference. H-infinity at 12% both completed a trial and failed another.
The H-infinity path remains **shadow only**, without arming:

```sh
sh scripts/run_hinf_path_logged.sh shadow config/controllers/hinf_path_40cm_28p5s_yaw_candidate.txt 28.5 12
```

Launchers require `battery_status` for armed tests and run in the foreground.
Keep the board terminal open during the run, whether using UART or SSH.
Always check `.status.log`, the stop reason and `exit_code=`. An empty CSV may
mean that a preflight check blocked startup.

## 9. Logs, transfer and plots — PC terminal

On the board, logs are written directly to `logs/real/<controller>/`.
Each run has a unique prefix: CSV, status and a `.config.txt` snapshot, or
`.args.txt` for PD. Keep the complete file group.

The board only needs `logs/real/<controller>/`: MuJoCo and `plot/` directories
belong on the PC. **Transfer is manual**, not automatic. Wait for the physical
run to finish before copying its files. Open a **PC terminal**, not the
`root@socfpga` shell, and enter your local project directory.

Example for our verified board/workstation pair, copying LQR logs:

```sh
cd ~/terasic_robot
mkdir -p logs/real
scp -r -o HostKeyAlias=192.168.50.2 root@10.185.186.166:/home/root/terasic_robot_final/logs/real/lqr logs/real/
```

The destination is `~/terasic_robot/logs/real/lqr/`. Change `lqr` in the remote
path to `pd`, `pd-outer`, `hinf` or `ga` to copy another controller. The entire
controller directory is copied, keeping CSV, status, configuration, arguments
and any other acquisition evidence together. Source files remain on the board.
Repeating a copy refreshes matching filenames on the PC.

To copy **all real controller logs** from the same board:

```sh
cd ~/terasic_robot
mkdir -p logs/real
scp -r -o HostKeyAlias=192.168.50.2 'root@10.185.186.166:/home/root/terasic_robot_final/logs/real/*' logs/real/
```

The IP and host-key alias above belong to the previously verified reference
setup. Check the current DHCP address. For another board or workstation, use
its actual address and verified key, without borrowing this alias:

```sh
BOARD=root@ACTUAL_BOARD_IP
CONTROLLER=lqr
mkdir -p "logs/real"
scp -r "$BOARD:/home/root/terasic_robot_final/logs/real/$CONTROLLER" logs/real/
```

If using the original board installation, replace `/home/root/terasic_robot_final`
with `/home/root/terasic_robot` in the remote path.

Generate figures from the selected CSV, replacing the filename:

```sh
env/bin/python -B scripts/plot_real.py logs/real/lqr/NOME_FILE.csv
```

The plotter automatically saves SVG/PDF/PNG under `logs/real/lqr/plot/`.
The CSV filename distinguishes plots from repeated runs. For other controllers,
change the CSV path. These plotting commands do not access hardware or start motors.
