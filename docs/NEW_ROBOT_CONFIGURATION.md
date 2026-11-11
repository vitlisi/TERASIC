# First-time Linux and USB Wi-Fi setup

Use a unique board ID. The verified reference image is DE10-Nano Linux4.5.0
ARMv7 with a USB148f:7601 MT7601U dongle, firmware `mt7601u.bin`, ConnMan,
Avahi and SSH. Different chipsets/kernel images need separate driver checks.
This setup guide does not start motors or calibrate a new robot automatically.
The reference setup was checked in November 2026. Historical per-board logs were removed; do not inherit reference motor parameters as measurements of a new robot.

## 1. Prepare the Linux PC and first console

Use a bootable board SD image, compatible dongle, UART console adapter and
Ethernet cable. Record board/SD/kernel/FPGA versions, MAC addresses, hostname,
dongle identity and calibration status. Keep passwords/private keys out of the
project and delivery archive. Example PC tools on Ubuntu:

```sh
sudo apt install openssh-client avahi-utils picocom usbutils
ls /dev/serial/by-id/
ip -br link
```

Use the actual UART device,115200 baud,8N1. The adapter is a console connection,
not an SSH service. From the board console inspect:

```sh
hostname
uname -a
ip -br address
lsusb
command -v connmanctl
systemctl status connman avahi-daemon sshd.socket --no-pager
```

Service names vary. This reference uses `sshd.socket`; an inactive `sshd.service`
alone is not proof SSH is disabled. Verify the actual installed unit. Set the
board clock from a known source if its initial date is stale.

## 2. Use Ethernet before Wi-Fi

See [direct Ethernet with PC DHCP](ETHERNET_DIRETTA_DHCP.md). On an unconfigured
image use UART to inspect its address/DHCP client first. Never assume the old
192.168.50.2 address is available. Before trusting SSH on a new board, compare
its public host-key fingerprint through UART:

```sh
ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
```

Then connect from the PC to the actual Ethernet address. Do not reuse robot01's
historical host-key alias for another board. A hostname resolving through a
wrong interface is not connectivity: inspect `ip route get ADDRESS` on the PC.

## 3. Verify the USB dongle and driver

On the board:

```sh
lsusb
ip -br link
lsmod
ls -l /lib/firmware/mt7601u.bin
```

If the kernel-matched MT7601U driver is missing, the prepared package is
`maintenance/wifi/wifi-install-4.5.0.tar.gz`. Verify its manifest/kernel against
the running board before installation. The build/package/install sources are
`scripts/build_wifi_modules.sh`, `scripts/package_wifi_modules.py` and
`scripts/install_wifi_modules.sh`; preserve their kernel/firmware dependencies.
Do not install a module merely because `/boot/zImage` has a matching name:
check the running kernel configuration and ABI. Reboot persistence must be
verified after installation; USB detection alone is not a working WLAN.

## 4. Connect with interactive ConnMan

On the board run `connmanctl`, then type the following at its prompt (do not
type `connmanctl>` itself):

```text
enable wifi
scan wifi
services
agent on
connect SERVICE_ID_FROM_CURRENT_SERVICES
```

Choose the exact service ID shown by the current scan; it contains the dongle
MAC and SSID encoding and is not a reusable ID for all boards/networks. Enter
the Wi-Fi password only when the local agent requests it. Do not save it in a
README or terminal transcript sent to the professor. `Agent registered` only
registers the credential agent; it does not mean the network is connected.
After success, type `quit` and inspect address/routes:

```sh
ip -br address
ip route
```

For a stale-service `Connect` error, rescan and use the current service ID.
Do not keep copying an obsolete ID. If necessary reopen ConnMan and register
the agent again; inspect `systemctl status connman` and its journal. Connected
link-local addressing alone is not a DHCP lease. On the PC join the same LAN
and verify SSH by IP before debugging mDNS.

## 5. Assign identity and verify reboot

Example board identity (replace `robot02` for another specimen):

```sh
hostnamectl set-hostname robot02
hostnamectl --transient set-hostname robot02
systemctl enable --now avahi-daemon
hostnamectl status
```

ConnMan may replace the transient name. On the reference image
`/etc/connman/main.conf` retains the following setting (merge with existing
configuration rather than blindly overwriting it):

```ini
[General]
AllowHostnameUpdates=false
```

Inspect `/etc/hosts` for an old board name. If hostnamectl is unavailable, update
the image's hostname using its supported method. After reboot verify the name,
automatic Wi-Fi reconnection, DHCP and SSH from the PC:

```sh
getent ahostsv4 robot02.local
ssh root@robot02.local hostname
```

Use the current board's verified public host key and connection address. Historical per-board SSH profiles were removed during cleanup. Verify hostname on login before treating a shared dongle MAC or old DHCP address as board identity.

## 6. Copy the project and compile

Copy source/configuration/model inputs to `/home/root/terasic_robot` using the
verified board connection. Exclude `env/`, caches and private credentials. Use
the delivery package's manifest to verify the transferred source version.
On the board, from that directory:

```sh
sh scripts/build_robot.sh
sh /home/root/terasic_robot/scripts/check_robot_setup.sh robot02
```

Builds produce ARM executables under `build/robot/` and do not run them. A PC
build is not an ARM build. The general setup checker prints OK/FAIL/WARN for
identity, tools, services, driver/firmware, network and compiled diagnostics;
it does not open `/dev/mem`, run motors, read credentials or certify sensor/
FPGA calibration. Its exit code indicates errors; warnings need inspection.

## 7. Inventory and acceptance

From the PC, use `scripts/inspect_robot_setup.sh ID` to preserve a board
inventory. Accept setup only after a reboot with correct hostname, Wi-Fi lease,
verified host key, SSH connectivity and passing setup check. Record unresolved
clock/FPGA/encoder/IMU questions explicitly. Ethernet/UART is often more reliable
for fixed-frame acquisition; network setup does not authorize motor tests.

Physical characterization requires a separate protocol, fixture/readiness and
verified wheel/encoder/IMU signs. Recent robot01 physical work is closed. The
complete original setup narrative is retained under `maintenance/review_20261107/`.
