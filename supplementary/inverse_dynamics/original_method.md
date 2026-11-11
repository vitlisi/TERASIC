# Robot01 — inverse body dynamics from existing LQR records

Authoritative results: `analysis04/`. Earlier analysis01–03 are retained development outputs; analysis04 additionally propagates the set-valued static-friction interval through the signed local-polynomial filter weights. No physical acquisition or model/controller change was performed.

This implements the professor's body-as-torque-sensor proposal as an initial conditional planar estimate, rather than claiming an independent torque measurement. Two real compensated 10 s LQR records are analyzed with the same frozen assumptions. A matched synthetic sensor record is processed as an estimator control. Neither real run is fitted.

## Equations and torque definition

Forward translation is world +Y; positive pitch theta leans forward about world -X. Let phi be the mean wheel angle relative to the chassis in the forward rolling convention. Under no slip and constant axle height,

```
x_axle = R * (phi + theta) + constant
x_accel = R * (phi_accel + theta_accel)
r_y = c_y*cos(theta) + c_z*sin(theta)
r_z = -c_y*sin(theta) + c_z*cos(theta)
tau_body = m*g*r_y - m*r_z*x_accel - (I_xx + m*(c_y²+c_z²))*theta_accel
```

`tau_body` is the sum of effective torques transmitted at the two wheel/chassis joints, positive for forward wheel drive. It is not electrical drive torque alone. The independently verified forward-dynamics equation also includes `+m*r_y*z_accel`; this term is set to zero in the real reconstruction because fixed axle height is assumed, not measured.

For consistency with the current MuJoCo equivalent-armature model, compare the inferred body torque to

```
tau_model = sum(electrical_drive - viscous_and_dry_losses - J_armature*phi_wheel_accel)
electrical_drive = (0.441 Nm / 12 V) * measured_voltage * applied_PWM/100
```

Omitting the equivalent-armature inertial term confuses the drive torque with the torque transmitted to the chassis. This is a comparison to the current equivalent model, not proof that all physical rotor/gearbox inertia effects are represented correctly. Raw applied PWM is used; compensation is already included and is not applied a second time.

## Reconstruction and assumptions

- Raw SX/DX counters use physical signs -1/+1 and 1,325 counts/revolution. No rollover correction: FPGA counters saturate.
- Raw counts plus pitch reproduce logged encoder position to approximately 1e-9 m, checking sign and pitch compensation. Logged 50 ms filtered velocity is not differentiated.
- Timestamp-aware local cubic least squares gives encoder velocity/acceleration and the derivative of measured gyro pitch rate. Fused pitch supplies gravity orientation. Nominal window: 11 samples (~0.11 s); 7 and 15 samples test sensitivity. These are offline, acausal estimates, not a proposed real-time observer.
- Sensor alignment uses the nominal CAD equilibrium as in the existing matched simulations. It remains unmeasured. Gyro filter delay is not inverted, and gyro and encoder derivatives are not guaranteed to represent the same physical instant. This limits high-frequency torque accuracy.
- CAD chassis mass/COM/inertia plus the previously assumed 20 g centered point payload at 0.1 m. Radius 0.032 m. Point payload centroidal inertia is omitted. Fixed frame alignment is preserved when varying mass/COM/inertia; the reference is not silently re-zeroed.
- Planar motion, rigid body, no slip, no external touch/support, negligible yaw/roll and fixed axle height. Operator autonomy was confirmed for both real trials; slip, vertical vibration and true body pose were not independently measured.
- Near zero estimated wheel speed (abs omega <0.1 rad/s), dry friction has a bounded range, not a prescribed sign. The green region propagates the interval through absolute values of the signed filter weights. This is a model-conditional static range, not a statistical confidence interval. Moving-only metrics exclude all windows containing such uncertain static samples.

## Validation and limits

Three automated tests passed: exact cubic derivatives at irregular timestamps; static-friction interval propagation; and body-moment torque reconstruction against independently known forward MuJoCo actuator/passive/friction/armature terms. The latter includes vertical acceleration and has maximum error below 2e-5 Nm over a short near-planar simulated motion. It checks algebra and signs, not physical model correctness.

The synthetic sensor control exposes reconstruction error from quantized counts, sensor filtering, regularization and friction switching even when the generating model is known. Its residual must not be ignored. The real torque curves are therefore diagnostic estimates; do not use them as a certified instantaneous torque sensor or immediately invert them into a LUT.

Parameter sensitivity cases are illustrative one-at-a-time changes, not measured uncertainty bounds: mass +/-10%, COM height +/-10%, pitch inertia +/-20%, alignment +/-0.5 deg, and removal of the assumed payload. None is optimized to the records. `summary.json` records all metrics and source hashes; CSV files retain full traces. Plots show t=0 through the complete 10 s, including one-sided derivative regions at the edges. Metrics use 2–9 s.

## Reproduce

```sh
env/bin/python diagnostics/inverse_body_torque.py --output /tmp/robot01_inverse_torque_new
env/bin/python tests/test_inverse_body_torque.py
```

No gains, Stribeck coefficients, motor torque scale or compensation LUT were changed. Any later identification needs sensor-delay/derivative checks and validation on a separate run; a body residual cannot uniquely identify torque scale, contact, inertia and compliance together.

## Numerical result, nominal 11-sample reconstruction

| Record | Body torque RMS [Nm] | Model center RMS [Nm] | Distance outside static interval RMS [Nm] | Moving-window residual RMS [Nm] | Moving samples / 700 |
|---|---:|---:|---:|---:|---:|
| latest | 0.018883 | 0.023825 | 0.031782 | 0.039962 | 108 / 700 |
| independent_earlier | 0.012392 | 0.019750 | 0.018942 | 0.030307 | 75 / 700 |
| synthetic_sensor_control | 0.001224 | 0.024053 | 0.000422 | 0.000882 | 150 / 700 |

The two real records show substantial residuals even with the frozen static-friction interval. The synthetic sensor-control record has much smaller distance outside that interval and moving-window residual. Because these are different motions and masks, this comparison is a reconstruction sanity check, not a calibrated uncertainty distribution or proof of one faulty parameter. Near-zero-speed center RMS/correlation alone are not torque-validity metrics because static friction is set-valued.

[Latest full trace](analysis04/latest_inverse_torque.png) · [Earlier independent full trace](analysis04/independent_earlier_inverse_torque.png) · [Synthetic sensor control](analysis04/synthetic_sensor_control_inverse_torque.png)
