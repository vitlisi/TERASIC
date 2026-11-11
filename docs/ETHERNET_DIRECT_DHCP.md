# Direct PC–robot Ethernet using PC-side DHCP

Verified on Ubuntu22.04/NetworkManager,6November2026. The board must run a DHCP
client on Ethernet. Keep192.168.50.0/24 free of conflicting routes; the PC can
retain Wi-Fi for Internet. Commands configure networking, not motors.

Find the PC interface (`enp0s31f6` on the original laptop; replace as needed):

```sh
ip -br link
nmcli connection show
```

Create this profile once if absent:

```sh
nmcli connection add type ethernet ifname enp0s31f6 con-name terasic-direct-dhcp ipv4.method shared ipv4.addresses 192.168.50.1/24 ipv4.never-default yes ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 100
```

Activate an existing profile rather than creating duplicates:

```sh
nmcli connection up terasic-direct-dhcp
nmcli -t -f NAME,DEVICE connection show --active
ip -br address
getent ahostsv4 socfpga.local
ip neigh show dev enp0s31f6
```

Select the board's actual Ethernet address, not a Wi-Fi address returned by mDNS. Old observed leases are not reservations. Verify the board's public host-key fingerprint through UART before trusting SSH, following [the general setup](CONFIGURAZIONE_ROBOT_WIFI.md).

```sh
ssh -o ConnectTimeout=30 -o ServerAliveInterval=15 -o ServerAliveCountMax=3 root@ACTUAL_BOARD_ADDRESS hostname
```

Use the current board's verified identity; no historical host-key alias is required. If no lease appears, inspect the PC shared profile and board DHCP client through UART. Never infer identity from a reused dongle alone.
