# Data conventions and interpretation

File names and CSV column identifiers remain unchanged for compatibility.
SX means physical left and DX physical right looking in the forward direction
only when the record declares `physical_sx_dx_v2`. Older FPGA labels may be
reversed; never silently relabel historical data.

## Time and provenance

Python simulator CSV: `phase=free` selects the released segment; `t_free_s`
is seconds since release and `t_s` may include support/calibration. Real-core
simulation uses `t_s` from control start. Physical CSV uses monotonic `t_ns`;
subtract its first value and multiply by1e-9. Do not infer equal time axes from
sample number when real scheduling varies. Keep status/config/JSON and hashes
with each CSV, including rejected/aborted trials.

## Python simulator telemetry

| Field | Meaning | Stored unit |
|---|---|---|
| `pitch_true_deg` | Geometric chassis pitch (evaluation truth) |deg|
| `pitch_est_deg` | IMU-derived feedback pitch |deg|
| `pitch_target_deg` | Generated pitch reference |deg|
| `distance_ref_m` | Desired longitudinal distance |m|
| `position_encoder_m` | Encoder/pitch-derived longitudinal estimate |m|
| `pose_true_x_m`, `pose_true_y_m` | True local displacement |m|
| `pose_est_x_m`, `pose_est_y_m` | Estimated local displacement |m|
| `velocity_ref_m_s` | Desired longitudinal velocity |m/s|
| `velocity_encoder_m_s` | Feedback longitudinal velocity estimate |m/s|
| `velocity_world_forward_m_s` | True forward velocity for evaluation |m/s|
| `x_theta_rad`, `x_psi_rad`, `x_rho_rad` | Estimated mean wheel angle, pitch, yaw |rad|
| corresponding `*_dot_rad_s` fields | Estimated angular rates |rad/s|
| `yaw_true_rad`, `yaw_ref_rad` | Geometric yaw and desired yaw |rad|
| `pwm_sx_applied_percent`, `pwm_dx_applied_percent` | Signed applied actuator commands after allocation/quantization |%|

Local+Y is initially forward, +X right. Plots may convert metres to cm/mm and
radians to degrees; their axis labels document the conversion. A path's
planned reference is not measured motion. Geometric truth must not be used as
sensor feedback unless an explicitly labeled ideal-feedback experiment does so.
Initial-pose offset experiments align displayed references/estimates with truth;
retain their reset/target metadata.

## Real and shared-core balance telemetry

| Field | Meaning |
|---|---|
| `pitch_acc_deg`, `pitch_fused_deg` | Accelerometer/fused pitch relative to the fixed controller reference, degrees |
| `gyro_deg_s` | Bias-corrected pitch rate, degrees/s |
| `position_m`, `velocity_m_s` | Feedback longitudinal estimates, not external position measurements |
| `yaw_encoder_rad`, `yaw_rate_encoder_rad_s` | Encoder-derived yaw/rate in real balance logs |
| `enc_sx_raw`, `enc_dx_raw` | Raw encoder counts; inspect origin/sign/CPR metadata |
| `imu_us`, `late_us` | IMU read duration and scheduling lateness, microseconds |
| `battery_v`, `battery_raw`, `battery_age_s` | ADC-derived supply, raw counts and age; not a calibrated state-of-charge estimate |
| `armed` | Whether physical actuation mode was enabled |

The real MIMO state is `[s, pitch, yaw, v, pitch_rate, yaw_rate]` in SI units.
Python six-state designs may instead use longitudinal wheel angle; use explicit
radius/sign transformations before importing gains. Fixed pitch reference is
+0.320deg in the retained real configuration. Startup-support angle and table
slope do not redefine that reference.

## Command stages

- `*_raw_percent`: unconstrained controller request.
- `*_limited_percent`: pair allocation/saturation result.
- `*_applied_percent`: physical readback in qualified real trials, or the
  quantized bridge output in base simulation logs; inspect version-specific metadata.
- Identification `probe_requested_percent` and `probe_allocated_percent`:
  external requested probe and the value retained after common headroom limits.
- `pwm_*_probe_target_percent`: post-probe command before hardware quantization.
- `pwm_*_delayed_drive_percent`: queued actuator input in optional delayed-model
  simulations; the bridge's applied column remains before that added delay.

In shadow, physical PWM is zero; hypothetical computed probe allocation is not
an applied identification input. The final armed identification attempt ended
before2s probe onset and therefore contains no nonzero probe samples.

## Motor LUT and inference limits

A virtual LUT request `r` is not physical PWM; its chosen target is `omega=k*r`.
Raised-wheel speed data do not directly identify loaded torque. Cold start and
already-moving plateaus differ; invalid/unsettled/voltage-flagged points remain
recorded but must not silently enter inverse support. Preserve the frozen LUT
hash and the distinction between calibration and independent green validation.
Torque columns derived from PWM/voltage or model inertia are estimates, never
load-cell/current measurements. Prescribed real supply in simulation is an
input, not a battery-model validation output.

## H-infinity and learning records

`hinf_feedback_0..5` is delayed feedback in transformed order
`[s/r, pitch, yaw, v/r, pitch_rate, yaw_rate]`; `hinf_xc_0..11` is dynamic
controller state after update. Check wheel order and normalized-duty versus
PWM-percent conversion. Training cost/reward, selection results and independent
validation are distinct. PPO lineage steps cannot be summed across unrelated
runs; completing training or surviving a rollout is not a stabilization certificate.
