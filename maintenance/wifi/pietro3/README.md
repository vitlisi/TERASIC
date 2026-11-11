# Historical network migration —30October2026

This records the exact historical SSID `3 Pietro`, configured over115200-baud FTDI UART. Credentials were stored only on the board, not in this repository. Router66 was retained. The observed192.168.1.36/37 addresses were DHCP assignments, not reserved addresses.

ConnMan initially associated with link-local addressing; udhcpc obtained a lease. The retained fallback scripts apply only to that specific historical network. Do not install them blindly on a new network. The active student setup procedure is in `docs/CONFIGURAZIONE_ROBOT_WIFI.md`; verify board identity and current interface/route rather than assuming archived addresses.

## Historical commands and identifiers

These commands belong to the dated protocol, not a new physical-run authorization. Review actual CLI help and archived supply/identity assumptions before reuse.

```sh
ssh -o HostKeyAlias=192.168.50.2 root@192.168.1.36
```


The original README is preserved at its repository-relative path inside `maintenance/review_20261107/original_readmes.tar.gz`. Numerical/source evidence is not rewritten by this English guide.
