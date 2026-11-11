"""Real-coded GA: frozen nominal DLQR, shared nonlinear trials and robust ranking.

No simulator truth enters feedback. Truth is used only in offline scoring.
"""
import contextlib
import copy
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import tempfile

import mujoco
import numpy as np

from lqr_controller import FULL_STATE_ORDER, solve_dlqr


def save_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def design_hash(design):
    return hashlib.sha256(json.dumps(design, sort_keys=True, allow_nan=False).encode()).hexdigest()


def validate_design(design):
    """Accept only the native six-state, duty-input, 100 Hz convention."""
    d = copy.deepcopy(design)
    if d['full_state_order'] != FULL_STATE_ORDER or d['state_order'] != FULL_STATE_ORDER:
        raise ValueError('Ordine degli stati LQR non compatibile')
    if d['input_order'] != ['duty_sx', 'duty_dx']:
        raise ValueError('Ordine/unita ingressi non compatibili')
    for key, value in [('dt_s', .01), ('wheel_radius_m', .032), ('track_width_m', 0.16444)]:
        # October exports rounded the same wheel track to 0.1644 m.
        compatible = math.isclose(float(d[key]), value, rel_tol=0, abs_tol=1e-9)
        if key == 'track_width_m':
            compatible |= math.isclose(float(d[key]), .1644, rel_tol=0, abs_tol=1e-9)
        if not compatible:
            raise ValueError('Design non compatibile: '+key)
    if not math.isfinite(d['equilibrium_pitch_deg']):
        raise ValueError('Equilibrio non finito')
    arrays = {}
    for key, shape in [('Ad', (6,6)), ('Bd_duty', (6,2)), ('K_duty', (2,6)),
                       ('K_pwm_percent', (2,6)), ('Q', (6,6)), ('R', (2,2))]:
        arrays[key] = np.asarray(d[key], dtype=float)
        if arrays[key].shape != shape or not np.isfinite(arrays[key]).all():
            raise ValueError('Matrice non valida: '+key)
    if not np.allclose(arrays['K_pwm_percent'], 100*arrays['K_duty'], rtol=1e-10, atol=1e-10):
        raise ValueError('K duty / percento incoerenti')
    for key in ('Q','R'):
        if not np.allclose(arrays[key],arrays[key].T) or np.min(np.linalg.eigvalsh(arrays[key])) <= 0:
            raise ValueError(key+' deve essere simmetrica definita positiva')
    k, _ = solve_dlqr(arrays['Ad'],arrays['Bd_duty'],arrays['Q'],arrays['R'])
    if not np.allclose(k,arrays['K_duty'],rtol=1e-8,atol=1e-10):
        raise ValueError('K non corrisponde a Q/R e al modello esportato')
    return d


def load_design(path):
    return validate_design(json.loads(Path(path).read_text()))


def scale_input_penalty(nominal, scale):
    """Scale R and redesign K consistently, keeping Q and physical model fixed."""
    if isinstance(scale, bool) or not math.isfinite(scale) or scale <= 0:
        raise ValueError('Scala R positiva e finita richiesta')
    d = copy.deepcopy(nominal)
    r = np.asarray(d['R'], dtype=float)*scale
    k, poles = solve_dlqr(np.asarray(d['Ad']), np.asarray(d['Bd_duty']),
                         np.asarray(d['Q']), r)
    d.update(R=r.tolist(), K_duty=k.tolist(), K_pwm_percent=(100*k).tolist(),
             closed_loop_poles=[[float(v.real),float(v.imag)] for v in poles],
             input_penalty_scale=float(scale))
    return d


def decode(nominal, genes, delay=1):
    genes = np.asarray(genes, dtype=float)
    if genes.shape not in ((6,), (8,)) or not np.isfinite(genes).all() or np.max(np.abs(genes)) > 1:
        raise ValueError('Richiesti sei (legacy) oppure otto geni finiti in [-1,1]')
    if delay not in (0,1):
        raise ValueError('Ritardo richiesto: zero o un campione')
    d = copy.deepcopy(nominal)
    q = np.diag(np.diag(nominal['Q'])*10**genes[:6])
    a, b, r = (np.asarray(nominal[key]) for key in ('Ad','Bd_duty','R'))
    if len(genes) == 8:
        r = np.diag(np.diag(r)*10**genes[6:])
    k, poles = solve_dlqr(a,b,q,r)
    delayed = np.block([[a, -b@k], [np.eye(6), np.zeros((6,6))]])
    delay_radius = float(np.max(np.abs(np.linalg.eigvals(delayed))))
    if delay and delay_radius >= 1:
        raise ValueError('Poli nominali instabili con un campione di ritardo')
    d.update(Q=q.tolist(), R=r.tolist(), K_duty=k.tolist(), K_pwm_percent=(100*k).tolist(),
             closed_loop_poles=[[float(p.real),float(p.imag)] for p in poles],
             ga=dict(genes_log10=genes.tolist(), nominal_design_sha256=design_hash(nominal),
                     feedback_delay_samples=delay, delayed_spectral_radius=delay_radius,
                     policy=('fixed nominal Ad/Bd; diagonal Q0/R0 * 10**genes' if len(genes)==8
                             else 'legacy: fixed R; diagonal Q0 * 10**genes')))
    return d


def scenarios(count, seed, prefix):
    """Nominal plus fixed independent +/-10% draws; not measured uncertainties."""
    rng = np.random.default_rng(seed)
    result = []
    for index in range(count):
        nominal = index == 0
        result.append(dict(name=f'{prefix}_{index:02d}', seed=seed+index,
            motor_scale=[1.,1.] if nominal else rng.uniform(.9,1.1,2).tolist(),
            friction_scale=1. if nominal else float(rng.uniform(.9,1.1)),
            armature_scale=[1.,1.] if nominal else rng.uniform(.9,1.1,2).tolist(),
            chassis_inertia_scale=1. if nominal else float(rng.uniform(.9,1.1)),
            encoder_noise_std_counts=0. if nominal else float(rng.uniform(.1,1.))))
    return result


def apply_scenario(robot, scenario):
    s = copy.deepcopy(scenario)
    for key, shape in [('motor_scale',(2,)), ('armature_scale',(2,)),
                       ('friction_scale',()), ('chassis_inertia_scale',())]:
        a = np.asarray(s[key], dtype=float)
        if a.shape != shape or not np.isfinite(a).all() or np.any(a <= 0):
            raise ValueError('Fattore scenario non valido: '+key)
    for key in ('damping_scale','joint_friction_scale'):
        a = np.asarray(s.get(key,[1.,1.]),dtype=float)
        if a.shape != (2,) or not np.isfinite(a).all() or np.any(a<=0):
            raise ValueError('Fattore motore non valido: '+key)
    imu_scale = np.asarray(s.get('imu_noise_scale',[1.,1.]),dtype=float)
    if imu_scale.shape != (2,) or not np.isfinite(imu_scale).all() or np.any(imu_scale<0):
        raise ValueError('Scala rumore IMU non valida')
    if (not isinstance(s['seed'], int) or s['seed'] < 0 or
            not math.isfinite(s['encoder_noise_std_counts']) or s['encoder_noise_std_counts'] < 0):
        raise ValueError('Seed/rumore scenario non valido')
    for i, name in enumerate(('motore_sx','motore_dx')):
        robot.m.actuator(name).gear[0] *= s['motor_scale'][i]
    for i, name in enumerate(('perno_sx','perno_dx')):
        robot.m.dof_armature[int(robot.m.joint(name).dofadr[0])] *= s['armature_scale'][i]
    for i, wheel in enumerate(('sx','dx')):
        adr = int(robot.m.joint('perno_'+wheel).dofadr[0])
        damping = s.get('damping_scale',[1.,1.])[i]
        friction = s.get('joint_friction_scale',[1.,1.])[i]
        robot.m.dof_damping[adr] *= damping
        robot.m.dof_frictionloss[adr] *= friction
        # Persist across set_motors(), which switches the ON/OFF coefficients.
        if wheel in robot.wheel_dynamics:
            profile = robot.wheel_dynamics[wheel]
            profile['armature_kg_m2'] *= s['armature_scale'][i]
            for key in ('damping_Nm_s_rad','off_damping_Nm_s_rad'):
                profile[key] *= damping
            for key in ('frictionloss_Nm','off_frictionloss_Nm'):
                profile[key] *= friction
    # Scale all contact surfaces: max-mixed ground friction must change as well.
    robot.m.geom_friction[:] *= s['friction_scale']
    robot.m.body_inertia[robot.body_id] *= s['chassis_inertia_scale']
    mujoco.mj_setConst(robot.m,robot.d)
    mujoco.mj_forward(robot.m,robot.d)
    return s


def trial_inputs(path):
    path = Path(path)
    status = json.loads(path.with_suffix('.status.log').read_text())
    meta = json.loads(path.with_suffix('.json').read_text())
    if status['stop_reason'] == 'interruzione utente':
        raise KeyboardInterrupt('Ricerca GA interrotta dall’utente')
    physical_errors = ('errore: ValueError: Contatore encoder simulato saturo; nessun rollover',
                       'errore: ValueError: Posizioni encoder non finite',
                       'errore: ValueError: Ingresso IMU non valido')
    if ((status['stop_reason'].startswith('errore:') and status['stop_reason'] not in physical_errors)
            or 'calibrazione' in status['stop_reason']):
        raise RuntimeError('Errore infrastruttura/avvio: '+status['stop_reason'])
    with path.open() as stream:
        rows = [r for r in csv.DictReader(stream) if r['phase'] == 'free']
    return status,meta,rows


def summarize_path_trial(path, duration, pwm_cap=50.):
    status,meta,rows = trial_inputs(path)
    completed = bool(status.get('path_completed',False))
    rms = status.get('tracking_rms_pose_error_true_m')
    return dict(failed=not completed,survived=bool(status['success']),path_completed=completed,
        missing_time_s=max(0.,duration-min(duration,len(rows)*.01)),
        cost=rms if rms is not None and math.isfinite(rms) else 1e12,
        cost_units='tracking RMS position error [m]; separate from stabilization cost',
        tracking_rms_position_m=rms,tracking_max_position_m=status.get('tracking_max_pose_error_true_m'),
        tail_max_position_m=status.get('tail_max_pose_error_true_m'),
        actual_release_pitch_deg=meta.get('release_true_pitch_deg'),stop_reason=status['stop_reason'],
        pitch_rms_deg=status.get('pitch_error_rms_deg'),pwm_rms_percent=status.get('pwm_motor_rms_percent'),
        peak_pwm_percent=status.get('max_abs_pwm_percent'),saturation_fraction=status.get('saturation_fraction'),
        csv=str(path))


DEFAULT_COST_WEIGHTS = dict(pitch=4., position=1., yaw=.5, speed=.25, pwm=.05, slew=.02)


def cost_weights(overrides=None):
    weights = dict(DEFAULT_COST_WEIGHTS)
    if overrides is not None:
        if set(overrides)-set(weights):
            raise ValueError('Coefficiente di costo sconosciuto')
        weights.update(overrides)
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or
           not math.isfinite(v) or v < 0 for v in weights.values()):
        raise ValueError('Coefficienti di costo finiti e non negativi richiesti')
    if not any(weights.values()):
        raise ValueError('Almeno un coefficiente di costo deve essere positivo')
    return weights


def summarize_trial(path, duration=10., pwm_cap=50., weights=None):
    weights = cost_weights(weights)
    status,meta,rows = trial_inputs(path)
    fields = ['t_free_s','pitch_true_deg','pitch_target_deg','pose_true_x_m',
              'pose_true_y_m','yaw_true_rad','velocity_world_forward_m_s',
              'pwm_sx_applied_percent','pwm_dx_applied_percent','velocity_world_lateral_m_s']
    values = np.array([[float(r[k]) for k in fields] for r in rows])
    if not len(rows) or not np.isfinite(values).all():
        return dict(failed=True, missing_time_s=duration, cost=1e12, stop_reason=status['stop_reason'])
    t, pitch, target, x, y, yaw, vel, left, right, lateral = values.T
    pitch_error = pitch-target
    position = np.hypot(x,y)
    speed = np.hypot(vel,lateral)
    u = np.column_stack((left,right))/pwm_cap
    # Fixed external cost. It never uses the candidate's own Q as a score.
    instant = (weights['pitch']*(pitch_error/3)**2 + weights['position']*(position/.01)**2
               + weights['yaw']*(yaw/math.radians(2))**2
               + weights['speed']*(speed/.02)**2 + weights['pwm']*np.mean(u*u,axis=1))
    slew = float(np.mean(np.diff(u,axis=0)**2)) if len(u)>1 else 0.
    cost = float(np.mean(instant)+weights['slew']*slew)
    complete = bool(status['success'] and len(rows) >= round(duration/.01))
    tail = t >= duration-2-1e-8
    stable = ((np.abs(pitch_error)<.3) & (position<.01) &
              (np.abs(yaw)<math.radians(2)) & (speed<.02))
    settled = bool(complete and tail.sum() >= 200 and np.all(stable[tail]))
    recovery, consecutive = None, 0
    for time, good in zip(t,stable):
        consecutive = consecutive+1 if good else 0
        if consecutive >= 100:
            recovery = max(0.,round(float(time)-.99,6))
            break
    return dict(failed=not settled, survived=complete, stabilized=settled,
        missing_time_s=max(0.,duration-min(duration,len(rows)*.01)), cost=cost,
        cost_weights=weights,
        stop_reason=status['stop_reason'], actual_release_pitch_deg=meta.get('release_true_pitch_deg'),
        pitch_rms_deg=float(np.sqrt(np.mean(pitch_error**2))),
        position_rms_m=float(np.sqrt(np.mean(position**2))),
        pwm_rms_percent=float(np.sqrt(np.mean(np.column_stack((left,right))**2))),
        peak_pwm_percent=float(np.max(np.abs([left,right]))),
        saturation_fraction=float(np.mean(np.max(np.abs([left,right]),axis=0)>=pwm_cap-1e-8)),
        recovery_time_s=recovery, tail_max_pitch_error_deg=float(np.max(np.abs(pitch_error[tail]))) if tail.any() else None,
        tail_max_position_m=float(np.max(position[tail])) if tail.any() else None,
        csv=str(path))


def rank_trials(trials):
    costs = [t['cost'] for t in trials]
    return [sum(int(t['failed']) for t in trials),
            sum(t['missing_time_s'] for t in trials), float(np.mean(costs)+.5*max(costs))]


def evaluate(design, scenario_list, protocol, folder=None):
    from mujoco_sim import parse_args, run
    results = []
    for scenario in scenario_list:
        mode = protocol.get('mode','lqr-motion')
        argv = ['--headless','--mode',mode,'--move-distance-m','0',
                '--duration',str(protocol['duration_s']), '--max-pwm',str(protocol['max_pwm']),
                '--release-pitch-deg',str(protocol['release_pitch_deg']),
                '--feedback-delay-samples',str(protocol['feedback_delay_samples']),
                '--imu-model','mpu6500','--encoder-model','motor',
                '--sensor-seed',str(scenario['seed']), '--tag',scenario['name']]
        if mode == 'lqr-path':
            argv += ['--path-file',str(protocol['path_file']),'--ultrasonic-stop-distance-m','.10']
        for key in ('disturbance_force_y_n','disturbance_start_s',
                    'disturbance_duration_s','disturbance_period_s'):
            if key in protocol:
                argv += ['--'+key.replace('_','-'),str(protocol[key])]
        with tempfile.TemporaryDirectory(prefix='terasic-ga-') as temp:
            destination = Path(folder)/scenario['name'] if folder is not None else Path(temp)
            with contextlib.redirect_stdout(io.StringIO()):
                code = run(parse_args(argv),destination,design_override=design,scenario=scenario)
            paths = list(destination.glob('*.csv'))
            if len(paths) != 1:
                raise RuntimeError('Log trial mancante/ambiguo: '+str(destination))
            summarize = summarize_path_trial if mode=='lqr-path' else summarize_trial
            kwargs = {} if mode=='lqr-path' else dict(weights=protocol.get('cost_weights'))
            trial = summarize(paths[0],protocol['duration_s'],protocol['max_pwm'],**kwargs)
            if code and trial.get('survived'):
                raise RuntimeError('Esito simulatore incoerente')
            trial['scenario'] = scenario['name']
            if folder is None:
                trial.pop('csv',None)
            results.append(trial)
    return rank_trials(results), results


def breed(population, ranks, rng, elite=2):
    order = sorted(range(len(population)),key=lambda i:tuple(ranks[i]))
    children = [population[i].copy() for i in order[:elite]]
    def select():
        drawn = rng.integers(0,len(population),3)
        return population[min(drawn,key=lambda i:tuple(ranks[i]))]
    while len(children) < len(population):
        first, second = select(), select()
        child = first.copy()
        if rng.random() < .8:
            child = np.where(rng.random(population.shape[1])<.5,first,second)
        if rng.random() < .3:
            mask = rng.random(population.shape[1])<1/population.shape[1]
            child = child + mask*rng.normal(0.,.2,population.shape[1])
        children.append(np.clip(child,-1.,1.))
    return np.asarray(children)
