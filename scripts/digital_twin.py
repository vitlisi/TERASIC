#!/usr/bin/env python3
"""Reproducible real-core MuJoCo trials and paired real/simulation plots.

No network or hardware access. Hardware configuration files are read unchanged.
"""
import argparse
import csv
import ctypes
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time as wall_clock
from zoneinfo import ZoneInfo

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src_python'))
from mujoco_robot import MujocoRobot, DEFAULT_MOTOR_FRICTION_PROFILE
from twin_payload import add_centered_payload
from twin_parameters import apply_parameters

STATES = ['position_m', 'pitch_rad', 'yaw_rad', 'velocity_m_s',
          'pitch_rate_rad_s', 'yaw_rate_rad_s']
FAULTS = {1: 'attitude/IMU envelope', 2: 'invalid encoder/state',
          3: 'odometry/velocity envelope', 4: 'nonfinite command'}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config_values(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        parts = line.split('#')[0].split()
        if parts:
            if len(parts) != 2 or parts[0] in values:
                raise ValueError('Invalid/duplicate configuration field: '+line)
            values[parts[0]] = float(parts[1])
    return values


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')


def read_csv(path):
    with Path(path).open() as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    with Path(path).open('w') as stream:
        writer = csv.DictWriter(stream, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def load_bridge(folder, kind):
    library = Path(folder)/(kind+'.so')
    command = ['g++', '-std=c++98', '-O2', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC']
    if kind in ('path', 'hinf-path'):
        command.append('-DTWIN_PATH')
        if kind == 'hinf-path':
            command.append('-DTWIN_HINF_PATH')
    elif kind == 'hinf':
        command.append('-DTWIN_HINF')
    subprocess.run(command+[str(ROOT/'scripts/digital_twin_bridge.cpp'), '-o', str(library), '-lrt'], check=True)
    core = ctypes.CDLL(str(library))
    core.twin_create.argtypes = [ctypes.c_char_p, ctypes.c_double, ctypes.c_double]
    core.twin_create.restype = ctypes.c_void_p
    core.twin_destroy.argtypes = [ctypes.c_void_p]
    core.twin_step.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]+[ctypes.c_double]*8+[ctypes.POINTER(ctypes.c_double)]
    core.twin_step.restype = ctypes.c_int
    return core


def sample_imu(robot, frame_offset_deg):
    acc, gyro = robot.read_imu()
    # Constant sensor-frame alignment, declared as an unmeasured assumption.
    angle = math.radians(frame_offset_deg)
    rotation = np.array([[math.cos(angle), 0, -math.sin(angle)],
                         [0, 1, 0], [math.sin(angle), 0, math.cos(angle)]])
    counts = np.clip(np.rint(rotation@acc/9.81*16384), -32768, 32767)
    rates = np.clip(np.rint(np.degrees(rotation@gyro)*131), -32768, 32767)/131
    return (math.degrees(math.atan2(-counts[0], counts[2])), rates,
            float(np.linalg.norm(counts)/16384), float(counts[2]))


def simulate_case(args, kind, folder, build):
    is_path = kind in ('path', 'hinf-path')
    is_hinf = kind in ('hinf', 'hinf-path')
    source = (args.hinf_path_config if kind == 'hinf-path' else args.hinf_config if kind == 'hinf' else args.stand_config if kind == 'stand' else args.path_config).resolve()
    cfg = config_values(source)
    folder.mkdir()
    snapshot = folder/'controller.config.txt'
    snapshot.write_bytes(source.read_bytes())
    cap = args.hinf_pwm if is_hinf else args.stand_pwm if kind == 'stand' else args.path_pwm
    if is_path:
        cap = min(cap, cfg['wheel_pwm_cap'])
    duration = args.path_duration if kind == 'hinf-path' else args.hinf_duration if kind == 'hinf' else args.stand_duration if kind == 'stand' else args.path_duration
    robot = MujocoRobot()
    robot.set_supply_voltage(args.voltage)
    body = robot.m.body('struttura_centrale')
    equilibrium = math.degrees(-math.atan2(body.ipos[1], body.ipos[2]))
    payload = add_centered_payload(robot, args.payload_mass_g/1000, args.payload_height_m)
    parameters = json.loads(args.parameters.read_text()) if args.parameters else {}
    if args.parameters:
        (folder/'motor_parameters.json').write_bytes(args.parameters.read_bytes())
    if parameters.get('actuation_delay_ms',0) or parameters.get('release_delay_ms',0):
        raise ValueError('Closed-loop launcher does not implement experimental actuation/release delays')
    experiment = apply_parameters(robot, parameters) if parameters else None
    friction_path = getattr(args, 'motor_friction_profile', None)
    if friction_path:
        robot.configure_motor_friction(json.loads(Path(friction_path).read_text()))
        (folder/'motor_friction_profile.json').write_bytes(Path(friction_path).read_bytes())
    effective_imu = parameters.get('imu', args.imu)
    sensor_configuration = None
    if effective_imu == 'mpu6500':
        sensor_configuration = robot.configure_sensors('mpu6500', 'ideal', args.sensor_config, args.seed)
        (folder/'imu_sensor_profile.json').write_bytes(args.sensor_config.read_bytes())
    frame_offset = cfg['pitch_reference_deg']-equilibrium
    robot.start_support(equilibrium+args.initial_error)
    for _ in range(2000):
        robot.step()
    calibration = []
    for _ in range(200):
        calibration.append(sample_imu(robot, frame_offset))
        for _ in range(10):
            robot.step()
    initial = float(np.mean([s[0] for s in calibration]))-cfg['pitch_reference_deg']
    bias = np.mean([s[1] for s in calibration], axis=0)
    if abs(initial) > cfg.get('startup_max_error_deg', 5):
        raise ValueError('Startup error outside hardware envelope')
    joint_q = [int(robot.m.joint(n).qposadr[0]) for n in ('perno_sx', 'perno_dx')]
    joint_v = [int(robot.m.joint(n).dofadr[0]) for n in ('perno_sx', 'perno_dx')]
    origin = robot.d.qpos[joint_q].copy()
    origin_xy = robot.d.qpos[robot.root_q:robot.root_q+2].copy()
    core = load_bridge(build, kind)
    handle = core.twin_create(os.fsencode(snapshot), initial, cap)
    if not handle:
        raise ValueError('Hardware parser rejected configuration')
    output = (ctypes.c_double*(44 if is_hinf else 26))()
    rows, fault = [], None
    robot.release()
    release_pitch = robot.true_pitch_deg()
    schedule = (2*cfg['leg_duration_s']+cfg['turn_duration_s']+cfg['initial_hold_s']+
                2*cfg['between_hold_s']+cfg['final_hold_s']) if is_path else None
    viewer = None
    viewer_threads = []
    try:
        if args.viewer:
            from mujoco import viewer as mujoco_viewer
            previous_threads = set(threading.enumerate())
            viewer = mujoco_viewer.launch_passive(robot.m, robot.d)
            viewer_threads = [t for t in threading.enumerate() if t not in previous_threads]
            with viewer.lock():
                viewer.cam.lookat[:] = robot.d.xpos[robot.body_id]
                viewer.cam.distance = 1.3 if is_path else .55
                viewer.cam.azimuth = 135
                viewer.cam.elevation = -20
            viewer.sync(state_only=True)
            print('Finestra MuJoCo aperta: simulazione in tempo reale; chiudere la finestra per interrompere.', flush=True)
        wall_start = wall_clock.perf_counter()
        for i in range(round(duration*100)):
            if viewer is not None and not viewer.is_running():
                fault = 'viewer closed by user';break
            time = i*.01
            acc, rates, norm, az = sample_imu(robot, frame_offset)
            gx, gy, gz = rates-bias
            q = robot.d.qpos[joint_q]-origin
            enc = np.rint(q*cfg['counts_per_rev']/(2*math.pi)*[1, -1]).astype(int)+32768
            if np.any(enc <= 0) or np.any(enc >= 65535):
                fault = 'saturated encoder';break
            ok = core.twin_step(handle, int(enc[0]), int(enc[1]), acc-cfg['pitch_reference_deg'],
                                float(gy), float(gx), float(gz), norm, az,
                                0. if i == 0 else .01, time, output)
            if not ok:
                fault = FAULTS.get(round(output[24]), 'unknown core failure');break
            x = np.asarray(output[:6])
            xy = robot.d.qpos[robot.root_q:robot.root_q+2]-origin_xy
            matrix = robot.d.xmat[robot.body_id].reshape(3, 3)
            row = dict(t_s=time, sample=i, t_ns=round(time*1e9),
                       pitch_fused_deg=math.degrees(x[1]), pitch_acc_deg=acc-cfg['pitch_reference_deg'],
                       gyro_deg_s=float(gy), accel_g=norm, enc_sx_raw=int(enc[0]), enc_dx_raw=int(enc[1]),
                       **dict(zip(STATES, map(float, x))),
                       position_reference_m=output[6], yaw_reference_rad=output[8],
                       pitch_reference_offset_rad=output[7], pitch_rate_reference_rad_s=output[10],
                       velocity_reference_m_s=output[9], yaw_rate_reference_rad_s=output[11],
                       phase=int(output[20]), schedule_finished=int(output[21]),
                       world_x_est_m=output[22] if is_path else 0.,
                       world_y_est_m=output[23] if is_path else x[0],
                       true_x_m=float(xy[0]), true_y_m=float(xy[1]),
                       true_pitch_error_deg=robot.true_pitch_deg()-equilibrium,
                       true_yaw_rad=math.atan2(matrix[1, 0], matrix[0, 0]),
                       true_velocity_x_m_s=float(robot.d.qvel[robot.root_v]),
                       true_velocity_y_m_s=float(robot.d.qvel[robot.root_v+1]),
                       supply_voltage_V=args.voltage, accel_correction_used=int(output[25]))
            if is_hinf:
                row.update({f'hinf_feedback_{j}': output[26+j] for j in range(6)})
                row.update({f'hinf_xc_{j}': output[32+j] for j in range(12)})
            for j, wheel in enumerate(('sx', 'dx')):
                row.update({f'pwm_{wheel}_raw_percent': output[12+j],
                            f'pwm_{wheel}_limited_percent': output[14+j],
                            f'pwm_{wheel}_applied_percent': output[16+j],
                            f'feedforward_{wheel}_percent': output[18+j],
                            f'wheel_{wheel}_joint_rad_s': float(robot.d.qvel[joint_v[j]]),
                            f'torque_{wheel}_drive_forward_Nm': robot.torque_per_duty*output[16+j]/100})
            robot.set_motors(output[16], output[17])
            # Interval means refer to [t,t+10ms); state columns refer to t.
            actuator, passive = [], []
            for _ in range(10):
                robot.step()
                actuator.append(robot.d.qfrc_actuator[joint_v].copy())
                passive.append(robot.d.qfrc_passive[joint_v].copy())
            for j, wheel in enumerate(('sx', 'dx')):
                row[f'torque_{wheel}_actuator_joint_mean_Nm'] = float(np.mean(actuator, axis=0)[j])
                row[f'torque_{wheel}_passive_joint_mean_Nm'] = float(np.mean(passive, axis=0)[j])
            rows.append(row)
            if viewer is not None:
                viewer.sync(state_only=True)
                remaining = wall_start+(i+1)*.01-wall_clock.perf_counter()
                if remaining > 0:
                    wall_clock.sleep(remaining)
    finally:
        robot.set_motors(0, 0)
        if viewer is not None:
            viewer.close()
            for thread in viewer_threads:
                thread.join(timeout=5)
        core.twin_destroy(handle)
    if rows:
        write_csv(folder/'simulation.csv', rows)
    sources = [source, ROOT/'scripts/digital_twin.py', ROOT/'scripts/digital_twin_bridge.cpp', ROOT/'scripts/twin_payload.py', ROOT/'scripts/twin_parameters.py',
               ROOT/'models/verifica.xml', ROOT/'src_python/mujoco_robot.py',
               ROOT/'src_python/sensor_models.py', ROOT/'src_python/wheel_dynamics.py', args.sensor_config.resolve()]
    sources += sorted((ROOT/'src_cpp/terasic_reale/controllers').glob('*.h'))
    if args.parameters:
        sources.append(args.parameters.resolve())
    if friction_path:
        sources.extend([Path(friction_path).resolve(), ROOT/'src_python/stribeck_motor.py'])
    sources += [ROOT/'src_cpp/terasic_reale/controllers'/name for name in
                ('balance_lqr_mimo.cpp', 'balance_lqr_path_simple.cpp', 'balance_hinf.cpp', 'balance_hinf_path.cpp')]
    summary = dict(kind=kind, success=fault is None and len(rows)==round(duration*100), fault=fault,
                   samples=len(rows), requested_duration_s=duration, simulated_duration_s=len(rows)*.01,
                   requested_initial_error_deg=args.initial_error, measured_initial_error_deg=initial,
                   release_true_pitch_deg=release_pitch, equilibrium_pitch_deg=equilibrium,
                   sensor_frame_alignment_deg=frame_offset, sensor_frame_alignment_measured=False,
                   imu=effective_imu, sensor_seed=args.seed, voltage_V=args.voltage,
                   external_supply=args.external_supply,
                   offline_low_voltage_exception=args.allow_low_voltage,
                   viewer_enabled=args.viewer,
                   startup_protocol='initial inclined support, fixed reference, gyro-bias calibration, free recovery',
                   support_model='temporary weld released at control start; no physical inclined-block contact',
                   sensor_configuration=sensor_configuration,
                   payload=payload,
                   experimental_model=experiment,
                   wheel_pwm_limit_percent=cap, differential_limit_percent=cfg['max_differential_pwm'] if not is_path else cap,
                   profile_duration_s=schedule, final_motor_ctrl=robot.d.ctrl.tolist(),
                   turn_yaw_reserve_percent=cfg.get('turn_yaw_reserve_percent',0) if kind=='hinf-path' else None,
                   state_order=STATES, K_pwm_percent=[[cfg[f'k_{w}_{s}'] for s in
                       ('position', 'pitch', 'yaw', 'velocity', 'pitch_rate', 'yaw_rate')] for w in ('sx', 'dx')] if not is_hinf else None,
                   controller_type='dynamic Hinf 12 states, one measurement delay' if is_hinf else 'static LQR',
                   controller_export_sha256=digest(ROOT/'models/hinf/simscape_2pwm_20261026.json') if is_hinf else None,
                   torque_per_full_duty_Nm=robot.torque_per_duty,
                   motor_model_identified_on_hardware=False,
                   motor_friction_profile=robot.motor_friction_profile,
                   runtime_motor_override=bool(friction_path),
                   physical_parameters=dict(body_mass_kg=robot.m.body_mass.tolist(),
                       body_inertia_kg_m2=robot.m.body_inertia.tolist(),
                       joint_damping=robot.m.dof_damping.tolist(), joint_armature=robot.m.dof_armature.tolist(),
                       joint_frictionloss=robot.m.dof_frictionloss.tolist()),
                   sha256={str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p): digest(p) for p in sources},
                   limitations=['Same real K/estimator/allocator/reference cores; no gain synthesis.',
                       'No measured real torque/current; PWM-derived torque is a model estimate.',
                       'Sensor-frame alignment assumes real fixed pitch reference corresponds to CAD equilibrium.',
                       '100 Hz without real Linux/I2C jitter; modeled MPU delay/noise is approximate.',
                       'Passive torque excludes dry friction/contact constraint forces; it is not total net wheel torque.',
                       'Real world trajectory needs an independent measurement; encoder odometry alone cannot validate slip.'])
    if rows:
        tail = rows[-min(200, len(rows)):]
        summary.update(peak_pwm_percent=max(abs(r[f'pwm_{w}_applied_percent']) for r in rows for w in ('sx', 'dx')),
                       tail_pitch_rms_deg=float(np.sqrt(np.mean([r['pitch_fused_deg']**2 for r in tail]))),
                       tail_speed_rms_m_s=float(np.sqrt(np.mean([r['velocity_m_s']**2 for r in tail]))),
                       final_xy_true_m=[rows[-1]['true_x_m'], rows[-1]['true_y_m']],
                       final_xy_est_m=[rows[-1]['world_x_est_m'], rows[-1]['world_y_est_m']],
                       schedule_finished=bool(rows[-1]['schedule_finished']))
    write_json(folder/'summary.json', summary)
    print(f'{kind}: success={summary["success"]}, duration={len(rows)*.01:.2f}s, fault={fault}', flush=True)
    if rows:
        plot(folder/'simulation.csv', folder/'simulation.svg')
    return summary


def normalized(rows):
    if not rows:
        raise ValueError('Empty log')
    result = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    result['t_s'] = (result['t_s']-result['t_s'][0] if 't_s' in result else
                     np.array([(int(r['t_ns'])-int(rows[0]['t_ns']))/1e9 for r in rows]))
    for target, candidates in [('yaw_rad', ['yaw_fused_rad', 'yaw_encoder_rad']),
                               ('yaw_rate_rad_s', ['yaw_rate_gyro_rad_s', 'yaw_rate_encoder_rad_s'])]:
        if target not in result:
            result[target] = next(result[k] for k in candidates if k in result)
    return result


def plot(sim_path, output, real_path=None):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-twin-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    traces = [('MuJoCo, real C++ core', normalized(read_csv(sim_path)))]
    if real_path:
        traces.append(('Real robot', normalized(read_csv(real_path))))
    fig, axes = plt.subplots(4, 2, figsize=(13, 13), layout='constrained', sharex=True)
    fields = [('pitch_fused_deg', 'Pitch error [deg]', 1), ('gyro_deg_s', 'Pitch angular velocity [deg/s]', 1),
              ('position_m', 'Odometry position [m]', 1), ('velocity_m_s', 'Odometry velocity [m/s]', 1),
              ('yaw_rad', 'Yaw [deg]', 180/math.pi), ('yaw_rate_rad_s', 'Yaw angular velocity [deg/s]', 180/math.pi),
              ('pwm_sx_applied_percent', 'Applied left PWM [%]', 1), ('pwm_dx_applied_percent', 'Applied right PWM [%]', 1)]
    for axis, (key, label, scale) in zip(axes.flat, fields):
        for name, trace in traces:
            axis.plot(trace['t_s'], trace[key]*scale, label=name, linewidth=1)
        axis.set(ylabel=label, xlabel='Time since first logged sample [s]')
        axis.grid(alpha=.25)
        axis.set_xlim(left=0)
        axis.legend(fontsize=8)
    fig.suptitle('Real robot versus MuJoCo — shared C++ controller cores' if real_path else 'MuJoCo — real C++ controller core and configuration')
    fig.savefig(output)
    fig.savefig(Path(output).with_suffix('.pdf'))
    fig.savefig(Path(output).with_suffix('.png'), dpi=200)
    plt.close(fig)
    sim = traces[0][1]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    for j, wheel in enumerate(('sx', 'dx')):
        axis = axes[0, j]
        axis.plot(sim['t_s'], sim[f'torque_{wheel}_drive_forward_Nm'], label='Modeled drive torque, forward')
        axis.plot(sim['t_s'], -sim[f'torque_{wheel}_passive_joint_mean_Nm'], label='Modeled passive torque, forward', alpha=.7)
        axis.set(xlabel='Time since first logged sample [s]', ylabel=f'{wheel.upper()} modeled torque [N m]')
        axis.legend(fontsize=8)
        axes[1, j].plot(sim['t_s'], -sim[f'wheel_{wheel}_joint_rad_s'])
        axes[1, j].set(xlabel='Time since first logged sample [s]', ylabel=f'{wheel.upper()} relative wheel speed, forward [rad/s]')
    for axis in axes.flat:
        axis.grid(alpha=.25)
    fig.suptitle('MuJoCo only — modeled torque; real torque is unmeasured')
    fig.savefig(Path(output).with_name(Path(output).stem+'_torque.svg'))
    plt.close(fig)
    fig, axis = plt.subplots(figsize=(8, 7), layout='constrained')
    axis.plot(sim['true_x_m'], sim['true_y_m'], label='MuJoCo ground truth')
    axis.plot(sim['world_x_est_m'], sim['world_y_est_m'], '--', label='MuJoCo odometry')
    if real_path and 'world_x_est_m' in traces[1][1]:
        real = traces[1][1]
        axis.plot(real['world_x_est_m'], real['world_y_est_m'], label='Real robot odometry')
    axis.set(xlabel='X [m]', ylabel='Y [m]', aspect='equal',
             title='Trajectory — real ground truth requires an external measurement')
    axis.grid(alpha=.25)
    axis.legend()
    fig.savefig(Path(output).with_name(Path(output).stem+'_xy.svg'))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sim = sub.add_parser('simulate')
    sim.add_argument('--viewer', action='store_true',
                     help='Show the same real-core simulation in the MuJoCo 3D viewer, paced in real time')
    sim.add_argument('--voltage', type=float, default=12.)
    sim.add_argument('--external-supply', action='store_true',
                     help='Offline bench-supply case: permit measured voltage up to 13 V instead of battery limit 12.6 V')
    sim.add_argument('--allow-low-voltage', action='store_true',
                     help='Offline only: reproduce an explicitly authorized measured run down to10.3V; normal minimum remains10.5V')
    sim.add_argument('--stand-config', type=Path, default=ROOT/'config/controllers/lqr_mimo.txt')
    sim.add_argument('--path-config', type=Path, default=ROOT/'config/controllers/lqr_path_40cm_28p5s.txt')
    sim.add_argument('--hinf-path-config', type=Path, default=ROOT/'config/controllers/hinf_path_40cm_28p5s.txt')
    sim.add_argument('--hinf-config', type=Path, default=ROOT/'config/controllers/hinf_stand.txt')
    sim.add_argument('--initial-error', type=float, default=-3.5)
    sim.add_argument('--imu', choices=['mpu6500', 'quantized'], default='mpu6500')
    sim.add_argument('--sensor-config', type=Path, default=ROOT/'config/mujoco_sensors_mpu6500.json',
                     help='Explicit sensor profile; defaults to the unchanged MPU6500 profile')
    sim.add_argument('--seed', type=int, default=0)
    friction = sim.add_mutually_exclusive_group()
    friction.add_argument('--motor-friction-profile', type=Path, default=DEFAULT_MOTOR_FRICTION_PROFILE,
                     help='Robot01 Stribeck profile; default for new trials, controller K unchanged')
    friction.add_argument('--legacy-motor-friction', action='store_true',
                     help='Explicit historical reproduction without Stribeck')
    sim.add_argument('--payload-mass-g', type=float, default=20.)
    sim.add_argument('--payload-height-m', type=float, default=.10)
    sim.add_argument('--case', choices=['both', 'stand', 'path', 'hinf', 'hinf-path'], default='both', help='both means LQR stand and path; hinf is stabilization; hinf-path is experimental full tracking')
    motors = sim.add_mutually_exclusive_group()
    motors.add_argument('--parameters', type=Path,
                        default=ROOT/'config/mujoco_motors_sx_dx_20261030.json',
                        help='Motor profile JSON; default: measured separate SX/DX baseline. K unchanged')
    motors.add_argument('--nominal-motors', action='store_true',
                        help='Use original XML motors for historical comparisons')
    sim.add_argument('--stand-pwm', type=float, default=20.)
    sim.add_argument('--path-pwm', type=float, default=20.)
    sim.add_argument('--stand-duration', type=float, default=10.)
    sim.add_argument('--hinf-pwm', type=float, default=20.)
    sim.add_argument('--hinf-duration', type=float, default=10.)
    sim.add_argument('--path-duration', type=float, default=33.)
    sim.add_argument('--output-dir', type=Path)
    compare = sub.add_parser('compare')
    compare.add_argument('--simulation', type=Path, required=True, help='simulation.csv')
    compare.add_argument('--real', type=Path, required=True, help='New real CSV, physical_sx_dx_v2')
    compare.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'compare':
        snapshot = args.real.with_suffix('.config.txt')
        expected = args.simulation.with_suffix('.config.txt')
        if not expected.exists():
            expected = args.simulation.parent/'controller.config.txt'
        if not snapshot.exists() or not expected.exists() or digest(snapshot) != digest(expected):
            parser.error('Real configuration snapshot missing or differs from simulation; rerun simulation with the real configuration')
        status = args.real.with_suffix('.status.log')
        if not status.exists() or 'wheel_convention=physical_sx_dx_v2' not in status.read_text():
            parser.error('Real log must declare wheel_convention=physical_sx_dx_v2')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        plot(args.simulation, args.output, args.real)
        real, sim = normalized(read_csv(args.real)), normalized(read_csv(args.simulation))
        end = min(real['t_s'][-1], sim['t_s'][-1])
        mask = real['t_s'] <= end
        errors = {}
        for key in ('pitch_fused_deg', 'gyro_deg_s', 'position_m', 'velocity_m_s', 'yaw_rad',
                    'yaw_rate_rad_s', 'pwm_sx_applied_percent', 'pwm_dx_applied_percent'):
            delta = real[key][mask]-np.interp(real['t_s'][mask], sim['t_s'], sim[key])
            errors[key] = dict(rmse=float(np.sqrt(np.mean(delta**2))), max_abs=float(np.max(np.abs(delta))))
        write_json(args.output.with_suffix('.json'), dict(real=str(args.real), simulation=str(args.simulation),
                   config_sha256=digest(snapshot), common_duration_s=float(end), errors=errors,
                   note='Aligned at first control sample, no time warp. Initial pitch, voltage, limits and runtime conditions must also match. Torque is unmeasured on hardware.'))
        print(args.output)
        return 0
    if args.nominal_motors:
        args.parameters = None
        args.motor_friction_profile = None
    if args.legacy_motor_friction:
        args.motor_friction_profile = None
    numbers = [args.voltage, args.initial_error, args.stand_pwm, args.path_pwm, args.stand_duration, args.path_duration, args.hinf_pwm, args.hinf_duration,
               args.payload_mass_g, args.payload_height_m]
    if (not all(math.isfinite(v) for v in numbers) or not (10.3 if args.allow_low_voltage else 10.5)<=args.voltage<=(13. if args.external_supply else 12.6) or
            abs(args.initial_error)>5 or not 0<args.stand_pwm<=100 or not 0<args.path_pwm<=50 or
            not 1<=args.stand_duration<=30 or not 28.5<=args.path_duration<=120 or not 1<=args.hinf_duration<=30 or not 0<args.hinf_pwm<=25 or args.seed<0 or
            not 0<=args.payload_mass_g<=100 or not 0<=args.payload_height_m<=.3):
        parser.error('Invalid voltage/start/PWM/duration/seed')
    folder = args.output_dir or ROOT/'logs/mujoco/digital_twin'/datetime.now(ZoneInfo('Europe/Rome')).strftime('twin_%Y%m%d_%H%M%S')
    folder.mkdir(parents=True, exist_ok=False)
    print('Output: '+str(folder), flush=True)
    with tempfile.TemporaryDirectory(prefix='terasic-twin-') as build:
        cases = ('stand', 'path') if args.case=='both' else (args.case,)
        results = {kind: simulate_case(args, kind, folder/kind, build) for kind in cases}
    write_json(folder/'summary.json', results)
    return 0 if all(r['success'] for r in results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
