# Offline verification

Run from the repository root; the C++ tests use RAM-backed registers and never
real FPGA motor addresses.

```sh
PYTHONDONTWRITEBYTECODE=1 env/bin/python -B -m unittest discover -s tests -v
PYTHONDONTWRITEBYTECODE=1 env/bin/python -B scripts/test_cpp.py
```

Retained tests cover controller/parser parity, delay/state/wheel conventions,
latched stop, launchers, sensor/battery interfaces, references, GA selection and
inverse-dynamics math. `test_ga_twin.py` checks the transferred GA delay against
its frozen gains. Necessary tools used by tests are retained.

The entire suite is not green: some historical settling/performance expectations
and an old damping-metadata expectation fail with the current default physics.
These tests remain visible, not deleted to manufacture a pass. This cleanup
freezes the existing model; it does not change the physics to satisfy those
expectations or claim loaded fidelity. See the exact
[cleanup checks](../maintenance/cleanup_20261109/verification.md).
