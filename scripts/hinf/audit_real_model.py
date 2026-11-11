#!/usr/bin/env python3
"""Export a MuJoCo local plant and audit the existing Hinf (offline only)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import mujoco
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src_python'))
from motion_lqr import design_motion_lqr
from mujoco_robot import MujocoRobot


def filtered_plant(a, b, dt=.01, tau=.05):
    """Native z6 + filtered relative-wheel speed + filtered yaw speed.

    Filters at k+1 use the sampled position difference from k to k+1.
    Ideal pitch/gyro here: the nonlinear screening includes the MPU model.
    """
    alpha = math.exp(-dt/tau)
    f = np.zeros((8, 8)); f[:6, :6] = a
    g = np.zeros((8, 2)); g[:6] = b
    for row, position in [(6, 0), (7, 2)]:
        f[row, :6] = (1-alpha)*(a[position]-np.eye(6)[position])/dt
        f[row, row] = alpha
        g[row] = (1-alpha)*b[position]/dt
    c = np.zeros((6, 8))
    c[0, :2] = [-1, 1]; c[1, 1] = c[2, 2] = 1
    c[3, 4] = 1; c[3, 6] = -1; c[4, 4] = c[5, 7] = 1
    return f, g, c


def closed_loop_radius(a, b, c, controller):
    # One explicit external measurement delay; positive feedback per export.
    ak, bk, ck, dk = (np.array(controller[k]) for k in ('Ak','Bk','Ck','Dk'))
    n, nk = len(a), len(ak)
    f = np.zeros((n+6+nk, n+6+nk))
    f[:n, :n] = a; f[:n, n:n+6] = b@dk; f[:n, n+6:] = b@ck
    f[n:n+6, :n] = c
    f[n+6:, n:n+6] = bk; f[n+6:, n+6:] = ak
    return float(max(abs(np.linalg.eigvals(f))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    designs = [design_motion_lqr(epsilon=e) for e in (1e-4, 5e-5)]
    for key in ('Ad', 'Bd_duty'):
        np.testing.assert_allclose(designs[0][key], designs[1][key], rtol=.002, atol=2e-5)
    raw = designs[0]; a = np.array(raw['Ad'])
    # MuJoCo is L,R; original Hinf and MATLAB synthesis are R,L.
    b = np.array(raw['Bd_duty'])[:, ::-1]
    ideal = np.eye(6); ideal[0,:2] = [-1,1]; ideal[3,3:5] = [-1,1]
    f, g, c = filtered_plant(a,b)
    cost = np.pad(ideal, ((0,0),(0,2)))
    original_path = ROOT/'models/hinf/simscape_2pwm_20261026.json'
    original = json.loads(original_path.read_text())
    robot = MujocoRobot()
    eq = math.radians(raw['equilibrium_pitch_deg'])
    robot.start_support(math.degrees(eq))
    mass = np.zeros((robot.m.nv,robot.m.nv))
    mujoco.mj_fullM(robot.m,robot.d,mass)
    lift = np.zeros((robot.m.nv,3))
    lift[robot.root_v+1,0] = .032
    lift[robot.root_v+3,1] = -1
    lift[robot.root_v+4,2] = -math.sin(eq)
    lift[robot.root_v+5,2] = math.cos(eq)
    for name,sign in [('perno_sx',1),('perno_dx',-1)]:
        lift[int(robot.m.joint(name).dofadr[0])] = [-1,1,sign*.16444/(2*.032)]
    projected_mass = lift.T@mass@lift
    # Same nominal constants/formula as terasic_hinf_parameters.m, not fitted.
    m,mw,r,track,l,jm,ip,iz = .752,.049,.032,.16444,.03503,.005,.001450,.000970
    jw = .5*mw*r*r
    analytic_mass = np.array([[r*r*(m+2*mw)+2*(jw+jm),m*r*l-2*jm,0],
        [m*r*l-2*jm,ip+m*l*l+2*jm,0],
        [0,0,iz+mw*track*track/2+2*(jw+jm)*(track/(2*r))**2]])
    inertial = ET.parse(ROOT/'models/verifica.xml').find('.//body[@name="struttura_centrale"]/inertial')
    tensor = [float(v) for v in inertial.attrib['fullinertia'].split()]
    joints = {}
    for name in ('perno_sx','perno_dx'):
        dof = robot.m.joint(name).dofadr[0]
        joints[name] = dict(armature=float(robot.m.dof_armature[dof]),
                           damping=float(robot.m.dof_damping[dof]),
                           frictionloss=float(robot.m.dof_frictionloss[dof]))
    payload = dict(schema_version=1, hardware_access=False, dt_s=.01,
        A=f.tolist(), B=g.tolist(), C_measurement=c.tolist(), C_cost=cost.tolist(),
        A_native=a.tolist(), B_native_right_left=b.tolist(),
        measurement_delay_samples=1, encoder_filter_tau_s=.05,
        input_order=['PWM_R','PWM_L'], input_unit='normalized_duty',
        measurement_order=['theta','psi','phi','theta_dot','psi_dot','phi_dot'],
        measurement_definition='[s/r,pitch,yaw,v/r,pitch_rate,yaw_rate]',
        internal_state_order=['wheel_relative','pitch','yaw','wheel_rate','pitch_rate','yaw_rate',
                              'filtered_wheel_rate','filtered_yaw_rate'],
        wheel_radius_m=.032, wheel_track_m=raw['track_width_m'],
        supply_voltage_v=robot.voltage, torque_per_duty_nm=robot.torque_per_duty,
        joints=joints, equilibrium_pitch_deg=raw['equilibrium_pitch_deg'],
        mass_audit=dict(coordinates=['s/r','pitch','yaw'],
            mujoco_projected_mass=projected_mass.tolist(),analytic_mass=analytic_mass.tolist(),
            relative_frobenius_difference=float(np.linalg.norm(projected_mass-analytic_mass)/np.linalg.norm(analytic_mass)),
            analytic_pitch_axis='Y, Ip=chassis_I_diag(2)',mujoco_pitch_axis='-X body, IMU local Y',
            analytic_chassis_pitch_inertia_kg_m2=ip,mujoco_chassis_pitch_inertia_kg_m2=tensor[0],
            note='Axis convention mismatch in current models; physical CAD/Simscape frame still needs checking. Analytic constants are a snapshot of terasic_hinf_parameters.m.'),
        original_controller_sha256=hashlib.sha256(original_path.read_bytes()).hexdigest(),
        old_controller_radius_mujoco_ideal=closed_loop_radius(a,b,ideal,original),
        old_controller_radius_mujoco_encoder_filters=closed_loop_radius(f,g,c,original),
        derivative_convergence_checked=True,
        source_hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in
            ['models/verifica.xml','src_python/motion_lqr.py','src_python/mujoco_robot.py',
             'src_cpp/terasic_reale/controllers/balance_lqr_mimo_core.h','scripts/hinf/terasic_hinf_parameters.m']},
        limitations=['Local no-slip projected plant, not identified real dynamics.',
            'Dry friction removed for differentiation, retained in nonlinear screening.',
            'Linear measurement model assumes ideal pitch and gyro, no acceleration contamination.',
            'Encoder filters included; MPU filter/sample/quantization only in nonlinear screening.',
            'No saturation, deadband, supply sag or startup estimator transient in pole check.'])
    path = args.output_dir/'mujoco_filtered_plant.json'
    path.write_text(json.dumps(payload,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in payload.items() if k.startswith('old_controller_')},indent=2))
    print(path)


if __name__ == '__main__': main()
