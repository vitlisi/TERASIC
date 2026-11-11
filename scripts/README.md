# Operational scripts

`digital_twin.py` runs the real C++ LQR or H-infinity cores offline in MuJoCo.
`simulate_ga_twin.py` uses the transferred GA cores including one feedback-sample delay.
Both use the frozen Robot01 Stribeck profile by default. See [usage](DIGITAL_TWIN.md).

`build_robot.sh` compiles standard controllers, transferred GA/H-infinity variants,
compensated LQR and raised-wheel diagnostics. Compilation does not execute them.
`run_*_logged.sh` are physical launchers and must only be used for the prepared test.
They do not universally enable compensation; pin the exact binary and config.
`test_cpp.py` builds and runs offline RAM-backed tests.

Plotting helpers retain full traces; final figures must use English labels and
start at acquisition t=0. Old campaign-specific figure generators were removed.
GA and H-infinity design tools remain in `ga/` and `hinf/`; no new optimization is scheduled.

`prepare_report_campaign.py` recreates the final report data/figure tree and checks frozen input hashes without starting a run. The simulation plotters retain all logged phases by default; `--free-only` explicitly requests a separate free-motion detail.

Use `run_experiment.py simulate` for named MuJoCo runs with automatic figure export and `run_experiment.py compare` for paired existing logs. `plot_real.py` plots existing hardware CSVs on the PC. `ROBOT_LOG_DIR` selects the six standard C++ controller launchers' output directory. `package_project.py` builds a downloadable archive without local environments/credentials. Full examples are in the root README.

`plot_digital_twin_report.py` reproduces the recovered historical digital-twin SVGs, including nominal/calibrated motor comparisons, 0.5% static branches, independent moving-LUT data and native bounded-compensator pulse overlay. It never accesses hardware or refits parameters.
