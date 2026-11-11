#!/usr/bin/env python3
"""Six-state/two-input LQR-derived hold-zero candidate; shadow-only export."""
import csv
import ctypes
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

import mujoco
import numpy as np
from design_real_lqr_candidate import ROOT, imu, complex_pairs
from motion_lqr import design_motion_lqr
from lqr_controller import solve_dlqr
from mujoco_robot import MujocoRobot

OUTPUT=ROOT/'logs/lqr/real_mimo_candidate_20261019'
RADIUS=.032


def design():
    models=[design_motion_lqr(epsilon=e) for e in (1e-4,5e-5)]
    for key in ('Ad','Bd_duty'):
        np.testing.assert_allclose(models[0][key],models[1][key],rtol=.002,atol=2e-5)
    a,b=np.array(models[0]['Ad']),np.array(models[0]['Bd_duty'])
    transform=np.eye(6)
    transform[0]=[-RADIUS,RADIUS,0,0,0,0]
    transform[3]=[0,0,0,-RADIUS,RADIUS,0]
    inverse=np.linalg.inv(transform)
    ap,bp=transform@a@inverse,transform@b
    limits=np.array([.05,math.radians(3),math.radians(15),.16,
                     math.radians(30),math.radians(60)])
    q=np.diag(1/limits**2); r=np.eye(2)/(2*.15**2)
    optimal,original_poles=solve_dlqr(ap,bp,q,r)
    # Keep longitudinal gains unchanged to isolate adding differential control.
    # Same units as previous hold-zero, not the old wheel-relative state.
    baseline=ROOT/'models/lqr/hardware_position_trial_20261018.txt'
    values=dict(x.split() for x in baseline.read_text().splitlines() if x and not x.startswith('#'))
    common=np.zeros(6)
    for col,key in zip([0,1,3,4],['k_position','k_pitch','k_velocity','k_pitch_rate']):
        common[col]=float(values[key])/100
    differential=np.zeros(6)
    differential[[2,5]]=.5*(optimal[1]-optimal[0])[[2,5]]
    k=np.vstack([common-differential,common+differential])
    # Actual encoder estimates use backward differences and 50 ms filters.
    alpha=math.exp(-.01/.05)
    f=np.zeros((8,8));f[:6,:6]=a;g=np.zeros((8,2));g[:6]=b
    for row,state in [(6,0),(7,2)]:
        unit=np.eye(6)[state]
        f[row,:6]=(1-alpha)*(a[state]-unit)/.01
        f[row,row]=alpha;g[row]=(1-alpha)*b[state]/.01
    kz=k@transform;feedback=np.zeros((2,8));feedback[:,:6]=kz
    feedback[:,6]=feedback[:,3];feedback[:,3]=0
    feedback[:,7]=feedback[:,5];feedback[:,5]=0
    poles=np.linalg.eigvals(f-g@feedback)
    if max(abs(poles))>=1: raise ValueError('Unstable filtered MIMO candidate')
    return dict(state_order=['position_m','pitch_error_rad','yaw_encoder_rad',
                'velocity_m_s','pitch_rate_rad_s','yaw_rate_encoder_rad_s'],
                input_order=['hardware_sx_duty','hardware_dx_duty'],Ad=ap.tolist(),
                Bd_duty=bp.tolist(),Q=q.tolist(),R=r.tolist(),K_pwm_percent=(100*k).tolist(),
                optimal_K_before_retuning_pwm_percent=(100*optimal).tolist(),
                optimal_poles_before_retuning=complex_pairs(original_poles),
                filtered_poles=complex_pairs(poles),optimal_for_recorded_Q_R=False,
                longitudinal_gains_preserved=True,
                baseline_sha256=hashlib.sha256(baseline.read_bytes()).hexdigest(),
                equilibrium_pitch_deg=models[0]['equilibrium_pitch_deg'],
                wheel_radius_m=RADIUS,track_width_m=models[0]['track_width_m'],
                max_wheel_pwm_percent=15,max_differential_pwm_percent=3,
                yaw_source='encoder difference only, 50 ms filtered rate; no gyro yaw fusion',
                yaw_convention='-radius/track * 2*pi/CPR * (delta_hw_sx + delta_hw_dx)',
                hardware_validated=False)


def load_core(folder):
    path=Path(folder)/'mimo.so'
    subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',
                    str(ROOT/'scripts/lqr_mimo_sim_bridge.cpp'),'-o',str(path)],check=True)
    lib=ctypes.CDLL(str(path));ptr=ctypes.POINTER(ctypes.c_double)
    lib.mimo_create.argtypes=[ptr,ctypes.c_double];lib.mimo_create.restype=ctypes.c_void_p
    lib.mimo_step.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_uint,ctypes.c_double,ctypes.c_double,ptr]
    lib.mimo_step.restype=ctypes.c_int;lib.mimo_destroy.argtypes=[ctypes.c_void_p]
    return lib


def simulate(candidate,core,error=0.,voltage=12.,offset=0.,yaw_deg=0.,pitch_torque=0.,yaw_torque=0.,sx_factor=1.):
    robot=MujocoRobot();robot.set_supply_voltage(voltage)
    if sx_factor!=1: robot.m.actuator('motore_sx').gear[0]*=sx_factor
    eq=candidate['equilibrium_pitch_deg'];robot.start_support(eq+error)
    for _ in range(2000):robot.step()
    angles=[];rates=[]
    for _ in range(200):
        angle,rate,_,_=imu(robot);angles.append(angle);rates.append(rate)
        for _ in range(10):robot.step()
    bias=float(np.mean(rates));joints=[int(robot.m.joint(n).qposadr[0]) for n in ('perno_sx','perno_dx')]
    origin=robot.d.qpos[joints].copy();ground_origin=float(robot.d.qpos[robot.root_q+1])
    gains=(ctypes.c_double*12)(*np.array(candidate['K_pwm_percent']).ravel())
    handle=core.mimo_create(gains,float(np.mean(angles))-eq);out=(ctypes.c_double*12)()
    robot.release()
    yaw=math.radians(yaw_deg)
    if offset or yaw:
        robot.d.qpos[robot.root_q+1]+=offset
        robot.d.qpos[joints]+=np.array([-offset/RADIUS+robot.wheel_track_m*yaw/(2*RADIUS),
                                        -offset/RADIUS-robot.wheel_track_m*yaw/(2*RADIUS)])
        pitch=math.radians(eq+error);cp,sp=math.cos(pitch/2),math.sin(pitch/2)
        cy,sy=math.cos(yaw/2),math.sin(yaw/2)
        robot.d.qpos[robot.root_q+3:robot.root_q+7]=[cy*cp,-cy*sp,-sy*sp,sy*cp]
        mujoco.mj_forward(robot.m,robot.d)
    rows=[];fault=None
    try:
        for i in range(2500):
            angle,rate,norm,az=imu(robot);rate-=bias;q=robot.d.qpos[joints]-origin
            sx=32768+int(np.rint(q[0]*1325/(2*math.pi)))
            dx=32768+int(np.rint(-q[1]*1325/(2*math.pi)))
            if not 0<sx<65535 or not 0<dx<65535 or not core.mimo_step(handle,sx,dx,angle-eq,rate,out):
                fault='invalid/saturated encoder or nonfinite command';break
            s,pitch,yaw,v,rate_rad,yaw_rate=out[:6]
            if (abs(pitch)>math.radians(12) or abs(angle-eq)>15 or abs(rate)>180 or
                not .6<=norm<=1.4 or az<8000 or abs(s)>.12 or abs(v)>.30 or
                abs(yaw)>math.radians(20) or abs(yaw_rate)>math.radians(90)):
                fault='hardware attitude/odometry envelope';break
            rotation=robot.d.xmat[robot.body_id].reshape(3,3)
            true_yaw=math.atan2(rotation[1,0],rotation[0,0])
            true_s=float(robot.d.qpos[robot.root_q+1])-ground_origin
            rows.append([i*.01,robot.true_pitch_deg()-eq,true_s,math.degrees(true_yaw),*out])
            active=8<=i*.01<8.08
            robot.set_pitch_disturbance(pitch_torque if active else 0.)
            robot.d.xfrc_applied[robot.body_id,5]=yaw_torque if active else 0.
            robot.set_motors(out[8],out[9])
            for _ in range(10):robot.step()
    finally:
        robot.set_motors(0,0);robot.set_pitch_disturbance(0)
        robot.d.xfrc_applied[robot.body_id,5]=0;core.mimo_destroy(handle)
    name=f'p{error:+.1f}_v{voltage:.1f}_s{offset:+.3f}_yaw{yaw_deg:+.1f}_tp{pitch_torque:+.3f}_ty{yaw_torque:+.3f}_sx{sx_factor:.2f}'
    with (OUTPUT/(name+'.csv')).open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['t_s','true_pitch_deg','true_forward_position_m','true_yaw_deg',
            'position_m','pitch_rad','yaw_rad','velocity_m_s','pitch_rate_rad_s','yaw_rate_rad_s',
            'sx_raw','dx_raw','sx_applied','dx_applied','common_raw','differential_raw'])
        writer.writerows(rows)
    tail=rows[-200:]
    position=max((abs(x[2]) for x in tail),default=float('inf'))
    yaw=max((abs(x[3]) for x in tail),default=float('inf'))
    return dict(case=name,completed=len(rows)==2500 and fault is None,fault=fault,
                last_2s_max_true_forward_position_m=position,last_2s_max_true_yaw_deg=yaw,
                returns_to_zero=position<.01 and yaw<3,
                peak_applied_pwm=max((max(abs(x[12]),abs(x[13])) for x in rows),default=0),
                final_ctrl=robot.d.ctrl.tolist())


def main():
    OUTPUT.mkdir(parents=True,exist_ok=True);c=design()
    cases=[dict(error=e) for e in (-1.,-.5,0.,.5,1.)]
    cases += [dict(error=e,voltage=10.8) for e in (-1.,1.)]
    cases += [dict(offset=s) for s in (-.02,.02)]
    cases += [dict(yaw_deg=y) for y in (-5.,5.)]
    cases += [dict(pitch_torque=t) for t in (-.015,.015)]
    cases += [dict(yaw_torque=t) for t in (-.002,.002)]
    cases += [dict(sx_factor=.90),dict(error=.5,yaw_deg=5.)]
    with tempfile.TemporaryDirectory() as folder:
        core=load_core(folder);c['nonlinear_cpp_core_screening']=[simulate(c,core,**case) for case in cases]
    sources=['scripts/design_real_lqr_mimo.py','scripts/lqr_mimo_sim_bridge.cpp',
             'src_cpp/terasic_reale/controllers/balance_lqr_mimo_core.h']
    c['sha256']={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources}
    c['limitations']=['Nominal model, deterministic IMU; hardware vibration/supply sag not identified.',
        'Encoder yaw cannot detect wheel slip; effective radius and track not precisely calibrated.',
        'Retains previous longitudinal feedback: does not establish removal of observed real 3 cm offset.',
        'Longitudinal path distance and heading hold; no lateral world-position control or trajectory.',
        '10% weaker SX simulation is hypothetical, not measured motor identification.',
        'Export is shadow-only; physical differential/yaw sign still requires manual verification.']
    (OUTPUT/'candidate.json').write_text(json.dumps(c,indent=2,allow_nan=False)+'\n')
    if not all(x['completed'] and x['returns_to_zero'] for x in c['nonlinear_cpp_core_screening']):
        raise ValueError('Nominal screening failed; configuration not exported')
    lines=['# Single 6-state, 2-PWM LQR-derived candidate; SHADOW ONLY.',
           '# State order [position_m,pitch_rad,yaw_rad,velocity_m_s,pitch_rate_rad_s,yaw_rate_rad_s].',
           'counts_per_rev 1325','gyro_sign 1','motor_sign 1','encoder_sx_sign -1','encoder_dx_sign 1',
           'wheel_model_sign -1','pitch_model_sign 1','pitch_reference_deg 0.320','shadow_only 1',
           'wheel_radius_m 0.032',f"wheel_track_m {c['track_width_m']:.17g}",'max_differential_pwm 3']
    for wheel,row in zip(['sx','dx'],c['K_pwm_percent']):
        lines += [f'k_{wheel}_{key} {value:.17g}' for key,value in zip(
            ['position','pitch','yaw','velocity','pitch_rate','yaw_rate'],row)]
    (OUTPUT/'gains_shadow_only.txt').write_text('\n'.join(lines)+'\n')
    print('K PWM%/SI:',c['K_pwm_percent'])
    for case in c['nonlinear_cpp_core_screening']:print(json.dumps(case))


if __name__=='__main__':main()
