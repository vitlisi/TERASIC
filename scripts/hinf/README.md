# MATLAB H-infinity synthesis and Simscape preparation

This workflow designs two-PWM stabilization from the supplied analytic Simscape
model. MATLAB, Control System Toolbox and Robust Control Toolbox are required;
nonlinear multibody simulation also needs Simulink and Simscape Multibody.
These MATLAB commands were documented but are not re-executed by the current
Python-only documentation review.

## Synthesize

From the repository root in MATLAB:

```matlab
addpath(fullfile(pwd,'scripts','hinf'));
result = design_hinf_2pwm();
```

`terasic_hinf_parameters.m` retains the analytic parameter/state conventions.
The state is `[theta; psi; phi; theta_dot; psi_dot; phi_dot]`, in rad/rad/s;
theta is longitudinal displacement divided by wheel radius, psi pitch and phi
yaw. Inputs are normalized `[PWM_R; PWM_L]`. Sampling is0.01 s, track0.1644 m,
and the original Simscape wheel offset0.082 m requires geometric verification.

The generalized plant includes a one-sample external measurement delay, additive
input disturbances and measurement-noise channels with declared design scales.
These weights are design assumptions, not measured noise or a verified
parametric uncertainty model. External saturation is not guaranteed by linear
synthesis; gamma only describes the stated weighted problem.

`generated/hinf_2pwm.mat` contains controller, generalized plant, weights and
matrices; the corresponding JSON exports Ak/Bk/Ck/Dk, delay and conventions.
`hinfsyn` already returns the stabilizing feedback sign: do not add another
negative K. Compute output from the previous controller state, then update it;
initialize dynamic/delay states consistently.

## Prepare comparison copies

```matlab
source = fullfile(pwd,'maintenance','matlab', ...
    'simulink_aggiornato_20261026','Terasic_Simscape_LQR_2PWM.slx');
folder = fullfile(pwd,'scripts','hinf','generated');
design = fullfile(folder,'hinf_2pwm.mat');
lqr_model = prepare_hinf_simscape(source,design, ...
    fullfile(folder,'Terasic_LQR_comparison_2PWM.slx'),'lqr');
hinf_model = prepare_hinf_simscape(source,design, ...
    fullfile(folder,'Terasic_Hinf_2PWM.slx'),'hinf');
out_lqr = sim(lqr_model);
out_hinf = sim(hinf_model);
```

Close models before preparation. Destinations must be new. Both copies receive
matching workspace parameters/STL paths; the H-infinity copy replaces static
feedback with a Discrete State-Space block, retaining the external delay once.
Verify the original yaw-estimator convention and gyro-integrated pitch before
claiming matched nonlinear behavior. `out.x` is estimated state, `out.u` is PWM
before saturation, and `out.PWM` is applied PWM. The original `disturnance` Step
is a measurement offset, not a mechanical push; use a force/torque input for
a physical disturbance.

## Transfer and limits

MuJoCo import is described in `MUJOCO.md`; historical review details remain in
`REVISIONE_20261026.md`. Explicitly transform state coordinates, wheel order and
pitch/yaw references. Imported MATLAB matrix dimensions alone are insufficient.
The current hardware/loaded-twin status is in the [main guide](../../README.md).
The matrices are retained unchanged; no new robustness certificate or anti-windup
is implied. Compare identical cases and store all requested signals/versions.
