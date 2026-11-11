# Real-robot C++ software

| Controller source | Board binary |
|---|---|
| `controllers/pitch_pd.cpp` | `pitch_pd` |
| `controllers/pitch_pd_fixed.cpp` | `pitch_pd_fixed` |
| `controllers/balance_lqr_mimo.cpp` | `balance_lqr_2` |
| `controllers/balance_lqr_path_simple.cpp` | `path_simple` |
| `controllers/balance_lqr_endurance.cpp` | `lqr_endurance` |

Build from the repository root on the DE10 with `sh scripts/build_robot.sh`.
Compilation does not start hardware; outputs go to `build/robot/`. On a PC,
these are PC executables, not ARM executables. Configuration files are in
`config/controllers/`; [scripts](../../scripts/README.md) lists launchers.

The timed path is 40 cm /180° /40 cm over 28.5 s. Endurance adds a final180°
pivot (35.5 s cycle). Simulated ultrasonic path stopping is not automatically
integrated into these physical programs. Keep all shared core headers: some
retain historical family names but are still imported by current controllers.

Current wheel convention is `physical_sx_dx_v2`, implemented by
`hardware/robot_wheel_map.h`: SX is physical left, DX physical right, looking
forward. Older logs keep their legacy convention. The fixed pitch reference
is not the inclined startup support's angle. H-infinity matrices and state
transforms have separate validation limits; a compiled binary is not a physical
validation. The final stand/path campaign is pending the user's diagram list; no new hardware run was started during cleanup.
