#!/usr/bin/env python3
"""Identify separate equivalent wheel dynamics from 10/15%; check 20% afterwards.

Only local logs, no hardware. Absolute inertias assume the existing torque scale.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import compare_freewheel as fw

TORQUE_COEFFICIENT = .441/12
GEOMETRIC_WHEEL_INERTIA = .5*.049*.032**2
SCALES = np.array([.001, .015, .01, .015, .01])
LOW = np.array([.00003, 0., 0., 0., 0.])/SCALES
HIGH = np.array([.02, .1, .1, .1, .1])/SCALES


def advance(position, speed, dt, torque, inertia, damping, friction):
    """Exact affine positive-rotation step, including finite-time Coulomb stop."""
    force = torque-friction
    if speed <= 0 and force <= 0:
        return position, 0.
    if damping < 1e-12:
        acceleration = force/inertia
        travel_dt = min(dt, -speed/acceleration) if acceleration < 0 else dt
        return (position+speed*travel_dt+.5*acceleration*travel_dt**2,
                max(0., speed+acceleration*dt))
    rate = damping/inertia
    equilibrium = force/damping
    decay = math.exp(-rate*dt)
    updated = equilibrium+(speed-equilibrium)*decay
    travel_dt = dt
    if updated < 0:
        travel_dt = math.log((speed-equilibrium)/(-equilibrium))/rate
        updated = 0.
    travel = equilibrium*travel_dt+(speed-equilibrium)*(-math.expm1(-rate*travel_dt))/rate
    return position+travel, updated


def predict(parameters, t, pwm, voltage, coefficient=TORQUE_COEFFICIENT):
    if len(parameters) == 3:
        inertia, damping, friction = parameters
        off_damping, off_friction = damping, friction
    else:
        inertia, damping, friction, off_damping, off_friction = parameters
    position, speed = np.zeros(len(t)), np.zeros(len(t))
    for i in range(1, len(t)):
        active = pwm[i-1] != 0
        position[i], speed[i] = advance(position[i-1], speed[i-1], t[i]-t[i-1],
            coefficient*voltage[i-1]*pwm[i-1]/100, inertia,
            damping if active else off_damping, friction if active else off_friction)
    return position, speed


def bounded_lm(residual, initial, size):
    """Small projected Levenberg-Marquardt fit; no optional SciPy dependency."""
    x = np.clip(np.array(initial)/SCALES[:size], LOW[:size], HIGH[:size])
    damping = .001
    r = residual(x*SCALES[:size])
    iterations = 0
    for iterations in range(80):
        jacobian = np.empty((len(r), size))
        for j in range(size):
            shifted = x.copy()
            shifted[j] = min(HIGH[j], x[j]+1e-4)
            if shifted[j] == x[j]:
                shifted[j] = max(LOW[j], x[j]-1e-4)
            jacobian[:, j] = (residual(shifted*SCALES[:size])-r)/(shifted[j]-x[j])
        hessian = jacobian.T@jacobian
        gradient = jacobian.T@r
        accepted = False
        previous = float(r@r)
        delta = np.zeros(size)
        for _ in range(12):
            delta = np.linalg.solve(hessian+damping*np.diag(np.maximum(np.diag(hessian), 1.)), -gradient)
            candidate = np.clip(x+delta, LOW[:size], HIGH[:size])
            updated = residual(candidate*SCALES[:size])
            if updated@updated < previous:
                x, r = candidate, updated
                damping = max(damping/3, 1e-10)
                accepted = True
                break
            damping *= 5
        if not accepted or np.linalg.norm(delta) < 1e-7 or previous-float(r@r) < 1e-7:
            break
    singular = np.linalg.svd(jacobian, compute_uv=False)
    return dict(parameters=(x*SCALES[:size]).tolist(),
        objective_sum_squared_counts=float(r@r), RMSE_counts=float(np.sqrt(np.mean(r*r))),
        iterations=iterations+1, scaled_jacobian_condition=float(singular[0]/max(singular[-1], 1e-15)),
        at_lower_bounds=[int(j) for j in range(size) if x[j]-LOW[j] < 1e-5])


def fit_wheel(t, pwm, voltage, measured, switched):
    # Optimizer receives only samples before t=10 s. No 20% outputs are available.
    windows = [np.flatnonzero((t >= start+.8) & (t < start+4.8)) for start in (0, 5)]
    def residual(parameters):
        predicted, _ = predict(parameters, t, pwm, voltage)
        return np.concatenate([(predicted[ids]-predicted[ids[0]]-
            measured[ids]+measured[ids[0]])*1325/(2*math.pi) for ids in windows])
    trials = []
    for inertia in (.0002, .0007, .002):
        initial = [inertia, .013, .01]
        if switched:
            initial += [.0005, .007]
        trials.append(bounded_lm(residual, initial, len(initial)))
    best = min(trials, key=lambda trial: trial['objective_sum_squared_counts'])
    return dict(best=best, starts=trials)


def profile(parameters):
    inertia, damping, friction, off_damping, off_friction = parameters
    return dict(armature_kg_m2=max(0., inertia-GEOMETRIC_WHEEL_INERTIA),
        damping_Nm_s_rad=damping, frictionloss_Nm=friction,
        off_damping_Nm_s_rad=off_damping, off_frictionloss_Nm=off_friction)


def metrics(real, predicted, t, start, end):
    indices = np.flatnonzero((t >= start) & (t < end))
    # Per-pulse origin: previous pulse area cannot distort this pulse's error.
    difference = (predicted[indices]-predicted[indices[0]])-(real[indices]-real[indices[0]])
    return float(np.sqrt(np.mean((difference*1325/(2*math.pi))**2)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    rows = fw.read_csv(args.real)
    fw.validate_real(rows, args.real.with_suffix('.status.log').read_text())
    cpr = fw.configuration(args.real.with_suffix('.config.txt'))
    if cpr != 1325:
        raise ValueError('This identification protocol uses the verified 1325 CPR conversion')
    t = np.array([r['t_s'] for r in rows])
    voltage = np.array([r['battery_v'] for r in rows])
    voltage[voltage == 0] = voltage[voltage > 0][0]
    train = t < 10.
    results, profiles, predictions, measurements = {}, {}, {}, {}
    for wheel, sign in (('sx', -1), ('dx', 1)):
        counts = np.array([r[f'enc_{wheel}_raw'] for r in rows])
        measured = (counts-counts[0])*sign*2*math.pi/cpr
        pwm = np.array([r[f'pwm_{wheel}_applied_percent'] for r in rows])
        models = {}
        for switched in (False, True):
            family = 'separate_on_off' if switched else 'shared_on_off'
            result = fit_wheel(t[train], pwm[train], voltage[train], measured[train], switched)
            parameters = result['best']['parameters']
            predicted, _ = predict(parameters, t, pwm, voltage)
            result['per_pulse_RMSE_counts'] = [metrics(measured, predicted, t, start, start+4) for start in fw.STARTS]
            models[family] = result
            predictions[(wheel, family)] = predicted
            print(wheel, family, json.dumps(result['best']), flush=True)
        results[wheel] = models
        profiles[wheel] = profile(models['separate_on_off']['best']['parameters'])
        measurements[wheel] = measured
    folder = args.output_dir
    folder.mkdir(parents=True, exist_ok=False)
    # Freeze the selected family and 10/15%-fitted parameters before MuJoCo checks.
    (folder/'parameters.json').write_text(json.dumps(dict(wheels=profiles), indent=2)+'\n')
    audit = dict(source=str(args.real), source_sha256=fw.sha(args.real),
        training='10% and 15% pulses only, t<10s; per-window count displacement residuals',
        check='20% from same already-inspected acquisition: internal check, not new independent validation',
        family='separate ON/OFF passive losses, chosen from observed decay behaviour before fitting',
        coefficient_Nm_per_volt_per_duty=TORQUE_COEFFICIENT, coefficient_measured=False,
        absolute_parameters_conditional_on_torque_scale=True,
        identifiability='Multiplying torque coefficient, J, b and friction by the same factor leaves predicted motion unchanged.',
        bounds_physical_identification=False, bounds=dict(lower=(LOW*SCALES).tolist(), upper=(HIGH*SCALES).tolist()),
        parameter_order=['J_total', 'b_ON', 'friction_ON', 'b_OFF', 'friction_OFF'],
        method='Exact first-order equivalent wheel model with Coulomb stop; bounded LM, 3 initial J values. No motor delay/backlash fitting.',
        results=results, wheel_profiles=profiles, model_original_sha256=fw.sha(fw.XML),
        fitting_script_sha256=fw.sha(__file__), hardware_unchanged=True)
    (folder/'identification.json').write_text(json.dumps(audit, indent=2, allow_nan=False)+'\n')
    aligned = []
    for i, time in enumerate(t):
        r = dict(t_s=float(time))
        for wheel in fw.WHEELS:
            r[wheel+'_real_rad'] = float(measurements[wheel][i])
            for family in ('shared_on_off', 'separate_on_off'):
                r[wheel+'_'+family+'_rad'] = float(predictions[(wheel, family)][i])
        aligned.append(r)
    fw.write_csv(folder/'reduced_model_predictions.csv', aligned)
    print(folder, flush=True)


if __name__ == '__main__':
    main()
