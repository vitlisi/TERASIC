# Terasic system requirements: hardware, software and Wi-Fi

Project reference configuration, documented as of 10 November 2026.
This inventory accompanies `requirements.txt`, which remains installable with pip.
Versions observed on the PC are distinguished from historical board records.
Where the repository does not record an exact version, it is marked for inspection.

## 1. Hardware components

| Component | Required / used configuration | Available version or identification |
|---|---|---|
| Controller board | Terasic DE10-Nano, SoC FPGA platform with ARMv7 HPS | Read the PCB revision and exact FPGA part number from the board; not recorded in this project |
| Robot and power board | Terasic balancing assembly with FPGA interfaces for motors, encoders and battery | Inspect the specific kit and power-board model/revision |
| Actuators | Two geared motors, wheels and associated power drivers | Inspect motor, gearbox and driver part numbers; software gains alone do not identify them |
| Encoders | One encoder per wheel, acquired through the FPGA | Verify counts per revolution and signs on the actual robot |
| IMU | MPU-6500 accelerometer and gyroscope over I2C | Software checks identity through WHO_AM_I |
| Power supply | Robot battery pack, compatible board supplies and charger | Model uses nominal 12 V; read battery chemistry, capacity and electrical ratings from the component |
| Battery acquisition | ADC chain and conversion expected by the robot software | Required by the real launchers' preflight checks |
| Ultrasonic sensor | Required for `ultrasonic_live` utilities and associated tests | Inspect model and wiring; not automatically integrated into real path stopping |
| Boot storage | microSD with Linux image and FPGA configuration compatible with the robot | Record capacity and image version/hash |
| Robot Wi-Fi | USB dongle with MediaTek MT7601U chipset | USB ID **148f:7601**; inspect dongle brand/revision |
| Wi-Fi network | WPA2 hotspot/access point at 2.4 GHz, with PC on the same LAN | Verified using an hotspot named Router66; that name is not required |
| Bench connections | Ethernet cable, compatible USB UART console connection/adapter and required USB cables | Console **115200 baud, 8N1**; verify the board's device and electrical levels |
| PC | Linux workstation for simulation, host compilation, figures and SSH/SCP | Observed local environment: Ubuntu **22.04.5 LTS** |
| Visualization | Graphical desktop and working OpenGL/GLFW libraries | Required for the MuJoCo viewer, not for headless simulation |

The DE10-Nano revision alone does not identify the robot bitstream.
The FPGA must already provide the PWM, encoder, watchdog and peripheral
registers expected by the source code. The current wheel mapping is
`physical_sx_dx_v2`, in `src_cpp/terasic_reale/hardware/robot_wheel_map.h`,
with MMIO base `0xFF200000`. Building C++ does not program the FPGA.
A complete replica also needs the bitstream, device tree and SD image
versions/hashes; these are not uniquely identified in the repository.

## 2. PC software

| Software | Reference version | Purpose |
|---|---|---|
| Ubuntu | 22.04.5 LTS observed on the PC | Project Linux environment |
| Python | **3.10.12** observed; README requires 3.10 or later | Scripts, simulation, analysis and packaging; other versions require dependency checks |
| Host GCC / g++ | **11.4.0**, Ubuntu package `11.4.0-1ubuntu1~22.04.3` | Compile simulation C++ cores and host checks |
| MuJoCo | **3.14.0** | Local simulator |
| NumPy | **2.2.6** | Numerical computation |
| Matplotlib | **3.10.9** | SVG, PNG and PDF figures |
| ReportLab | **5.1.0** | PDF report generation |
| pip and venv | Version not pinned | Isolated installation using modules from the selected Python |
| OpenSSH client | Version not recorded | `ssh`, `scp`, key verification |
| Avahi utilities | Version not recorded | `.local` name resolution and mDNS diagnostics |
| NetworkManager | Version not recorded | Direct Ethernet connection with PC-shared DHCP |
| picocom, usbutils, iproute2 | Versions not recorded | Serial console, `lsusb`, addresses and routing |
| tar, gzip, sha256sum | Versions not recorded | Packaging and integrity verification |

Basic Ubuntu installation, in the PC terminal:

```sh
sudo apt update
sudo apt install python3 python3-venv python3-pip build-essential \
  openssh-client avahi-utils picocom usbutils iproute2 network-manager \
  libgl1 libglfw3
python3 -m venv env
env/bin/python -m pip install -r requirements.txt
```

pip resolves indirect Python dependencies. `requirements.txt` pins the four
direct dependencies; it is not a complete lock of every package and system
library. Do not copy `env/` to the ARM board.

Start PD with visualization from the project root on the PC:

```sh
env/bin/python -B scripts/run_experiment.py simulate --controller pd \
  --name 02_pd_real/simulation_preview --viewer
```

## 3. DE10-Nano software and compiler

| Item | Requirement / documented version |
|---|---|
| Operating system | DE10-Nano Linux **4.5.0** image, **ARMv7** architecture; inspect exact distribution/image name and version |
| Native compiler | `g++` available on the reference board; **exact version not recorded**, inspect with `g++ --version` |
| Language and flags | **C++98**, `-std=c++98 -O2 -Wall -Wextra -Werror`, link with `-lrt` |
| Runtime | libc, libstdc++ and POSIX realtime library compatible with the board compiler |
| Tools | POSIX `sh`, system utilities, tar/gzip, sha256sum; `build_robot.sh` does not require `make` |
| Hardware access | `/dev/mem`, robot I2C and FPGA peripherals; adequate permissions, reference setup uses root |
| Network and services | ConnMan/`connmanctl`, SSH server, Avahi for `.local`, DHCP client, iproute2 |
| Kernel modules | `modprobe`, `modinfo`, `depmod`, modules compatible with the running kernel and ABI |

Python 3 was not found on the reference board: real controllers are C++
programs; simulations and figures run on the PC. GCC 11.4.0 on the PC does
not establish the compiler version on the board. A PC binary is not an ARM
executable for the DE10.

From the root of the runtime transferred to the board:

```sh
sh scripts/inspect_robot_setup.sh my_robot
sh scripts/check_robot_setup.sh my_robot wlan0
sh scripts/build_robot.sh
```

The build generates controllers, battery status and diagnostics under
`build/robot/`; it does not start motors. Tests also require profiles in
`config/controllers/` and calibration of IMU/motor/encoder signs, geometry,
pitch reference, power supply and limits for the specific robot.

## 4. Wi-Fi actually used

The connection used is **USB MT7601U dongle on DE10 → WPA2 2.4 GHz hotspot
← PC**, with SSH for the shell and SCP for sources/logs. Control runs locally
on the DE10: Wi-Fi does not close the PD/LQR/H-infinity control loop.
The ESP32 mentioned in historical documentation is not the Wi-Fi bridge used
in this configuration. An additional Ethernet–Wi-Fi router is not required
for this USB dongle setup.

Driver requirements:

- Firmware **`/lib/firmware/mt7601u.bin`** on the board.
- **`cfg80211`**, **`mac80211`**, **`mt7601u`** modules and crypto dependencies matching the kernel.
- Retained package: `maintenance/wifi/wifi-install-4.5.0.tar.gz`.
- Documented installation under `/lib/modules/4.5.0/updates/terasic_wifi_trial`, indexed with `depmod` and loaded at boot through `/etc/modules-load.d/terasic-wifi.conf`.
- Verify kernel ABI/configuration and manifest before installing; the number `4.5.0` alone does not guarantee compatibility.

Historical records report reconnection after reboot, SSH with Ethernet
disconnected, and bidirectional transfers with matching hashes. Driver EEPROM
warnings remain documented for further investigation. An eduroam scan alone
does not establish connectivity: **802.1X authentication was not completed**.
Use WPA2 at 2.4 GHz to reproduce the verified connection.

On the board, open `connmanctl` and enter the following at its prompt:

```text
enable wifi
scan wifi
services
agent on
connect SERVICE_ID_MOSTRATO_DA_SERVICES
```

Enter the password when prompted, then type `quit`. The service ID depends
on the dongle and network. PC and board must be able to reach each other on
the LAN; hotspot client isolation must allow SSH. Addresses are assigned by
DHCP and must be inspected rather than copied from old logs. Avahi enables
names such as `my_robot.local`; alternatively use the actual IP. First verify
the board's public SSH host key through UART. Passwords and private keys are
not part of the deliverable requirements.

Board verification commands:

```sh
uname -r
lsusb
lsmod
ls -l /lib/firmware/mt7601u.bin
modinfo mt7601u
ip -br address
ip route
systemctl status connman avahi-daemon sshd.socket --no-pager
```

The reference configuration uses `sshd.socket`; other systems may have
different service names. Repeat checks after reboot and test SSH/SCP with
Ethernet disconnected. Setup guide:
[NEW_ROBOT_CONFIGURATION.md](NEW_ROBOT_CONFIGURATION.md).

## 5. Toolchain for rebuilding Wi-Fi modules (maintenance only)

Not required at every startup if the correct modules are already installed.

| Item | Retained reference |
|---|---|
| Kernel sources | `https://github.com/terasic/linux-socfpga.git`, branch `socfpga-4.5` |
| Recorded commit | `261670baa10c7c71308db76bca9e7cdea816f945` (identifier reported in the preparation file) |
| ARM cross compiler | `arm-linux-gnueabihf-gcc`, candidate build version **11.4.0** |
| Kernel configuration | `maintenance/wifi/kernel-4.5.0.config.gz` |
| Candidate metadata | `vermagic: 4.5.0 SMP mod_unload ARMv7 p2v8` |
| PC tools | GNU make, host compiler, ARM toolchain and tools required by the legacy kernel configuration |
| Procedure | `scripts/build_wifi_modules.sh`, `scripts/package_wifi_modules.py`, `scripts/install_wifi_modules.sh` |

The script sets `ARCH=arm`, `CROSS_COMPILE=arm-linux-gnueabihf-`,
`LOCALVERSION=`, `HOSTCFLAGS='-O2 -fcommon'` and
`KCFLAGS='-march=armv7-a -mfpu=vfpv3-d16 -fno-pic -fno-pie'`.
These options address legacy kernel behavior with modern GCC. Successful
compilation does not replace symbol and ABI verification. Local sources:
`maintenance/wifi/build-preparation.json` and
`maintenance/wifi/candidate-modules.json`.

## 6. Information still needed for an identical replica

A fully reproducible inventory for the professor also needs physical details
not recorded in the sources: DE10 and power-board PCB revisions, FPGA part
number, motor/encoder part numbers, battery pack, SD image/bitstream/device
tree identities and hashes, native ARM compiler version, network service
versions, and MATLAB/Quartus releases and toolboxes. No values have been
invented for these fields.

PC inventory:

```sh
cat /etc/os-release
g++ --version
env/bin/python --version
env/bin/python -m pip freeze
ssh -V
```

Board inventory, without starting controllers:

```sh
uname -a
cat /etc/os-release
g++ --version
ldd --version
lsusb
connmanctl --version
avahi-daemon --version
df -h /home/root
```

Some utilities may not support `--version` on the legacy image. Record the
installed package using the distribution's tools instead. Also use
`scripts/inspect_robot_setup.sh` for a structured inventory.
See [README.md](../README.md) for runtime transfer, controller commands and
figure generation, and [ETHERNET_DIRECT_DHCP.md](ETHERNET_DIRECT_DHCP.md)
for initial Ethernet setup and recovery.
