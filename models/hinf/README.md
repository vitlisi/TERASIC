# Imported MATLAB H-infinity controller

`simscape_2pwm_20261026.json` retains the imported order12 controller: six state
inputs, two normalized right/left PWM outputs and0.01 s period. Its matrices
were compared with the supplied MAT/SLX workspace and nominal linear stability
was checked. `--mode hinf` uses explicit state/wheel transformations in MuJoCo;
this does not establish real-robot validity.

`stabilization_gain_trial_20261028.json` is a separate historical tuning
(common gain1, pitch-rate input0.75), retaining order12 and10 ms delay. It does
not inherit a new gamma guarantee. It passed only restricted offline cases,
failed the±5° stress cases, and the later real trial failed before shutdown.
The JSON remains unchanged to preserve its pre-trial identity. Do not promote
it based on earlier metadata alone. See [H-infinity workflow](../../scripts/hinf/README.md).
