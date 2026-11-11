# Real H-infinity controller

`controllers/balance_hinf.cpp` embeds the 12-state export from
`models/hinf/simscape_2pwm_20261026.json`, runs at 100 Hz and retains a one-sample
measurement delay. Numerical parity tests verify the Python/C++ state and wheel
transforms. This is not a general physical stability or loaded-fidelity claim.
`controllers/balance_hinf_path.cpp` provides the separate timed-path route.

The real estimator state is `[s,pitch,yaw,v,pitch_rate,yaw_rate]`. The export uses
wheel-angle coordinates; physical left/right outputs and PWM units must retain
the explicit transformations. Matrices and export hashes are checked by
`scripts/hinf/export_hinf_cpp.py --check` from the workspace root.

Baseline profiles are in `config/controllers/hinf_*.txt`. The separately retained
`diagnostics/*hinf_tuned_g035_r050.cpp` wrappers use their corresponding tuned
profiles and header. Final selection and compensation integration must be
explicitly pinned before the new campaign. See the workspace README and
`scripts/DIGITAL_TWIN.md`; old trial-specific result links were removed.
