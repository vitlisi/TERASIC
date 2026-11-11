# Hardware controller profiles

These are key/value TXT files read by C++, not simulator JSON or executables.
`lqr_mimo.txt` is the six-state/two-PWM baseline; `lqr_mimo_shadow.txt` forbids
armed mode. `lqr_path_40cm_28p5s.txt` contains the timed 40 cm /180° /40 cm path
also used by endurance. `lqr_near_vertical.txt` retains K/reference but narrows
the startup window to0.5°. The MIMO startup limit can be tightened, not enlarged
beyond5°; runtime protections remain separate.

The retained geometry is wheel radius0.032 m and track0.16444 m. CPR1325 is the retained encoder calibration input. The fixed pitch reference is+0.320°;
do not replace it with the support/table inclination. Correcting odometry
does not synthesize or validate new gains.

`hinf_shadow.txt` contains geometry/sensor/reference data; the dynamic H-infinity
matrices are embedded in its binary and are not a static K in this TXT. Its
`shadow_only 1` prevents actuation. All existing parameter files remain unchanged
by the English-documentation cleanup. `physical_sx_dx_v2` names physical left
and right; retain legacy names in older logs and inspect their convention field.

Transferred fixed GA profiles: `ga_stand.txt`, `ga_path.txt`; use the GA-specific one-sample delayed bridge. The tuned H-infinity `hinf_g035_r050_*` profiles are preserved separately from baseline profiles. Final selections and compensation settings are pending.
