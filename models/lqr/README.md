# LQR configuration and screening inputs

Historical hardware profiles are `hardware_mimo_trial_20261019.txt`,
`hardware_mimo_shadow_20261019.txt` and `hardware_path_simple_40cm_28p5s_20261019.txt`.
Use `config/controllers/` to identify the later retained operational profiles.

`hardware_position_trial_20261018.txt` supplies the longitudinal baseline to the
MIMO design/tests. `path_screening_inputs.json` contains equilibrium, K and
feedforward values for C++ path/endurance screening (including its historical
common-velocity factor1.5). `matrices_common_pwm_corrected.json` is retained
for comparison/metadata and audit tests; it does not synthesize current
six-state MuJoCo gains. Do not remove it as an apparently obsolete duplicate.

JSON files do not program hardware. MATLAB/Simscape originals are under
`maintenance/matlab/`; optional SLX audits may skip if that local material is
absent. Wheel track is0.16444 m. Geometry correction is not controller retuning.
