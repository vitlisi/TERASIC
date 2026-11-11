#!/usr/bin/env python3
"""Nominal design and offline screening; never exports an armed hardware config."""
import argparse
import ctypes
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src_python'))
from lqr_controller import solve_dlqr
from motion_lqr import design_motion_lqr
from mujoco_robot import MujocoRobot


def complex_pairs(values):
    return [[float(v.real), float(v.imag)] for v in values]


def filtered_active_poles(a, b, k4, dt=.01, tau=.05):
    alpha = math.exp(-dt/tau)
    f = np.zeros((5, 5)); f[:4, :4] = a
    f[4, :4] = (1-alpha)*(a[0]-[1, 0, 0, 0])/dt
    f[4, 4] = alpha
    g = np.r_[b[:, 0], (1-alpha)*b[0, 0]/dt][:, None]
    feedback = np.array([[0., k4[1], 0., k4[3], k4[2]]])
    return np.linalg.eigvals((f-g@feedback)[1:, 1:])


def design():
    # Reuse mechanical linearization, not motion-controller gains/references.
    models = [design_motion_lqr(epsilon=e) for e in (1e-4, 5e-5)]
    for key in ('Ad', 'Bd_duty'):
        np.testing.assert_allclose(models[0][key], models[1][key], rtol=.002, atol=2e-5)
    raw = models[0]
    indices = [0, 1, 3, 4]  # theta, pitch error, relative wheel rate, pitch rate
    a = np.array(raw['Ad'])[np.ix_(indices, indices)]
    b = np.array(raw['Bd_duty'])[indices].sum(axis=1, keepdims=True)
    # Position is cyclic and unregulated for this first balancing candidate.
    assert np.max(np.abs(a[:, 0]-[1, 0, 0, 0])) < 1e-8
    q = np.diag(1/np.array([math.radians(3), 5., math.radians(30)])**2)
    r = np.array([[1/.15**2]])
    k, poles = solve_dlqr(a[1:, 1:], b[1:], q, r)
    k4 = np.r_[0., k[0]]
    dt, tau = .01, .05
    # Include exact sampled-position difference and 50 ms velocity filter.
    # Pitch/gyro here are ideal; actual complementary filter is tested below.
    filtered_poles = filtered_active_poles(a,b,k4,dt,tau)
    if np.max(np.abs(filtered_poles)) >= 1:
        raise ValueError('Candidate unstable with encoder velocity filter')
    return dict(state_order=['theta_relative_rad','psi_error_rad','theta_dot_rad_s','psi_dot_rad_s'],
                Ad=a.tolist(), Bd_common_duty=b.tolist(), Q=q.tolist(), R=r.tolist(),
                K_pwm_percent=(100*k4).tolist(), K_duty=k4.tolist(), dt_s=dt,
                wheel_velocity_filter_tau_s=tau, closed_loop_poles=complex_pairs(poles),
                filtered_poles_excluding_unregulated_position=complex_pairs(filtered_poles),
                unregulated_position_pole=1.,
                equilibrium_pitch_deg=raw['equilibrium_pitch_deg'],
                projection='Symmetric longitudinal subset of MuJoCo no-slip finite-difference model',
                derivative_convergence_checked=True,
                candidate_coordinate_mapping={'wheel_model_sign':-1,'pitch_model_sign':1,'motor_sign':1},
                coordinate_mapping_hardware_verified=False,
                counts_per_rev=1325, real_pitch_reference_candidate_deg=.320,
                hardware_gains_validated=False,
                input='same signed normalized duty on BOTH motors; B already sums contributions')


def load_core(folder):
    library = Path(folder)/'lqr_core_sim_bridge.so'
    subprocess.run(['g++', '-std=c++98', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-shared', '-fPIC', str(ROOT/'scripts/lqr_core_sim_bridge.cpp'),
                    '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    ptr = ctypes.POINTER(ctypes.c_double)
    lib.lqr_sim_create.argtypes = [ctypes.c_double, ptr, ctypes.c_double]
    lib.lqr_sim_create.restype = ctypes.c_void_p
    lib.lqr_sim_step.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint,
                                ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                ctypes.c_double, ptr]
    lib.lqr_sim_step.restype = ctypes.c_int
    lib.lqr_sim_destroy.argtypes = [ctypes.c_void_p]
    return lib


def imu(robot):
    acc, gyro = robot.read_imu()
    # Match real MPU range/ADC quantum; no invented random noise or bias.
    counts = np.clip(np.rint(acc/9.81*16384), -32768, 32767)
    angle = math.degrees(math.atan2(-counts[0], counts[2]))
    rate = float(np.clip(np.rint(math.degrees(gyro[1])*131), -32768, 32767)/131)
    return angle, rate, float(np.linalg.norm(counts)/16384), int(counts[2])


def simulate(candidate, core, output, error_deg, voltage):
    robot = MujocoRobot()  # original XML/contact/friction retained
    robot.set_supply_voltage(voltage)
    equilibrium = candidate['equilibrium_pitch_deg']
    robot.start_support(equilibrium+error_deg)
    for _ in range(2000):
        robot.step()
    # Bias calibration while held, with fixed equilibrium target, as on board.
    angles, gyros = [], []
    for _ in range(200):
        angle, rate, _, _ = imu(robot); angles.append(angle); gyros.append(rate)
        for _ in range(10): robot.step()
    bias = float(np.mean(gyros))
    qpos = [int(robot.m.joint(n).qposadr[0]) for n in ('perno_sx','perno_dx')]
    origin = robot.d.qpos[qpos].copy()
    gains = (ctypes.c_double*4)(*candidate['K_pwm_percent'])
    handle = core.lqr_sim_create(1325., gains, float(np.mean(angles))-equilibrium)
    result = (ctypes.c_double*6)()
    robot.release()
    rows, fault = [], None
    try:
        for i in range(1000):
            angle, rate, norm, az = imu(robot); rate -= bias
            # Forward physical rotation is negative in the MuJoCo wheel joints.
            q = robot.d.qpos[qpos]-origin
            sx = 32768+int(np.rint(q[0]*1325/(2*math.pi)))
            dx = 32768+int(np.rint(-q[1]*1325/(2*math.pi)))
            if not (0 < sx < 65535 and 0 < dx < 65535):
                fault = 'encoder saturation'; break
            valid = core.lqr_sim_step(handle, sx, dx, angle-equilibrium, rate, .01, 18., result)
            if not valid:
                fault = 'invalid C++ core output'; break
            fused, theta, velocity, raw, limited, applied = result
            if (abs(fused)>12 or abs(angle-equilibrium)>15 or abs(rate)>180 or
                    not .6 <= norm <= 1.4 or az<8000):
                fault = 'same attitude/IMU envelope as hardware'; break
            rows.append([i*.01, robot.true_pitch_deg()-equilibrium, fused, rate,
                         theta, velocity, raw, limited, applied, sx, dx, norm])
            robot.set_motors(applied, applied)
            for _ in range(10): robot.step()
    finally:
        robot.set_motors(0, 0)
        core.lqr_sim_destroy(handle)
    name = f'nominal_release_{error_deg:+.1f}deg_{voltage:.1f}V'
    with (output/f'{name}.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t_s','true_pitch_error_deg','fused_pitch_error_deg','gyro_deg_s',
                         'theta_model_rad','wheel_rate_filtered_rad_s','pwm_raw_percent',
                         'pwm_limited_percent','pwm_applied_percent','enc_sx','enc_dx','accel_g'])
        writer.writerows(rows)
    tail = rows[-200:]
    return dict(case=name, release_error_deg=error_deg, voltage_v=voltage,
                samples=len(rows), completed=len(rows)==1000 and fault is None, fault=fault,
                peak_true_pitch_error_deg=max((abs(row[1]) for row in rows), default=None),
                last_2s_max_true_pitch_error_deg=max((abs(row[1]) for row in tail), default=None),
                peak_pwm_percent=max((abs(row[8]) for row in rows), default=None),
                saturated_samples=sum(abs(row[6])>18 for row in rows),
                final_ctrl=robot.d.ctrl.tolist())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'logs/lqr/real_candidate_20261016')
    parser.add_argument('--pitch-rate-scale', type=float, default=1.,
                        help='Manual retuning of pitch-rate gain only; NOT a new optimal LQR solution')
    args = parser.parse_args()
    if not math.isfinite(args.pitch_rate_scale) or not 0 < args.pitch_rate_scale <= 1:
        parser.error('--pitch-rate-scale must be finite and in (0,1]')
    if args.pitch_rate_scale != 1 and args.output == ROOT/'logs/lqr/real_candidate_20261016':
        parser.error('Specify a separate --output for manual retuning')
    args.output.mkdir(parents=True, exist_ok=True)
    candidate = design()
    candidate['pitch_rate_scale'] = args.pitch_rate_scale
    candidate['optimal_for_recorded_Q_R'] = args.pitch_rate_scale == 1
    if args.pitch_rate_scale != 1:
        candidate['K_before_pitch_rate_scale_pwm'] = list(candidate['K_pwm_percent'])
        candidate['K_pwm_percent'][3] *= args.pitch_rate_scale
        candidate['K_duty'][3] *= args.pitch_rate_scale
        a,b = np.array(candidate['Ad']),np.array(candidate['Bd_common_duty'])
        k = np.array(candidate['K_duty'])
        poles = np.linalg.eigvals(a[1:,1:]-b[1:]@k[None,1:])
        filtered = filtered_active_poles(a,b,k)
        if max(np.abs(poles)) >= 1 or max(np.abs(filtered)) >= 1:
            raise ValueError('Retuned candidate fails nominal linear stability screening')
        candidate['closed_loop_poles'] = complex_pairs(poles)
        candidate['filtered_poles_excluding_unregulated_position'] = complex_pairs(filtered)
        candidate['retuning_note'] = 'Only pitch-rate gain manually scaled after hardware chatter; not optimal for recorded Q/R.'
    sources = ['models/verifica.xml','src_python/motion_lqr.py','src_python/lqr_controller.py',
               'src_cpp/terasic_reale/controllers/balance_lqr_core.h',
               'scripts/lqr_core_sim_bridge.cpp','scripts/design_real_lqr_candidate.py']
    candidate['sha256'] = {s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
    with tempfile.TemporaryDirectory() as folder:
        core = load_core(folder)
        cases = [(e, 12.) for e in (-1., -.5, 0., .5, 1.)]+[(-1., 10.8), (1., 10.8)]
        candidate['nonlinear_cpp_core_screening'] = [simulate(candidate,core,args.output,e,v) for e,v in cases]
    candidate['limitations'] = [
        'Nominal mechanics/actuator parameters are not identified on current real robot.',
        'Real pitch/front, gyro and motor input coordinate mapping still require verification.',
        'Common wheel position is unregulated (k_theta=0); yaw is not controlled.',
        'MuJoCo screening includes actual C++ complementary/encoder filters and PWM math, not Linux/I2C timing.',
        'IMU is deterministic and quantized; real bias, vibration and current/supply sag not identified.',
        'No armed config generated; model screening is not real hardware validation.']
    path=args.output/'candidate.json'; path.write_text(json.dumps(candidate,indent=2,allow_nan=False)+'\n')
    gains = candidate['K_pwm_percent']
    config = ['# Nominal MuJoCo candidate: hardware coordinates/model NOT validated.',
              '# SHADOW ONLY. The controller must reject this file in run mode.',
              'shadow_only 1', 'pitch_reference_deg 0.320', 'counts_per_rev 1325',
              'gyro_sign 1', 'motor_sign 1', 'encoder_sx_sign -1', 'encoder_dx_sign 1',
              '# Model joint angle is negative for physical forward rotation.',
              'wheel_model_sign -1', '# Pitch mapping is a hypothesis to verify manually.',
              'pitch_model_sign 1']
    config += [f'{key} {value:.17g}' for key,value in zip(
        ['k_theta','k_psi','k_theta_dot','k_psi_dot'],gains)]
    (args.output/'gains_shadow_only.txt').write_text('\n'.join(config)+'\n')
    print('K PWM% [theta,psi,theta_dot,psi_dot]:', candidate['K_pwm_percent'])
    for case in candidate['nonlinear_cpp_core_screening']: print(json.dumps(case))
    print('Candidate:',path)


if __name__=='__main__':
    main()
