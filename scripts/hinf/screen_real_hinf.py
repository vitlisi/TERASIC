#!/usr/bin/env python3
"""Nominal MuJoCo screening of the real C++ Hinf core. Never connects to hardware."""
import argparse,csv,ctypes,hashlib,json,math,subprocess,sys,tempfile
from pathlib import Path
import numpy as np
from collections import deque
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src_python'),str(ROOT/'scripts')]
from mujoco_robot import MujocoRobot
from design_real_lqr_candidate import imu
from hinf_controller import DiscreteHinf

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initial-error-deg',type=float,default=-3.6)
    parser.add_argument('--voltage',type=float,default=11.67)
    parser.add_argument('--max-pwm',type=float,default=20,help='1..50, simulation ONLY; real executable has its own lower cap')
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--imu-model',choices=['ideal','mpu6500'],default='ideal')
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--motor-factor',type=float,default=1.,help='Hypothetical torque scale, not identification')
    parser.add_argument('--actuator-delay-samples',type=int,default=0,help='Additional hypothetical command transport delay')
    parser.add_argument('--controller-json',type=Path,help='Offline candidate of arbitrary order; does not modify the real binary')
    a=parser.parse_args()
    if not (math.isfinite(a.initial_error_deg) and abs(a.initial_error_deg)<=5 and
            math.isfinite(a.voltage) and 10.5<=a.voltage<=12.6 and
            math.isfinite(a.max_pwm) and 1<=a.max_pwm<=50 and
            math.isfinite(a.motor_factor) and .5<=a.motor_factor<=1.5 and
            0<=a.actuator_delay_samples<=3):parser.error('Parametri di screening fuori intervallo')
    a.output_dir.mkdir(parents=True,exist_ok=False)
    candidate=DiscreteHinf(a.controller_json) if a.controller_json else None
    if candidate and (not math.isclose(candidate.export['wheel_radius_m'],.032,abs_tol=1e-12) or
                      not math.isclose(candidate.export['wheel_track_m'],.16444,abs_tol=1e-12)):
        raise ValueError('Geometria del candidato diversa dal modello corrente')
    robot=MujocoRobot();robot.set_supply_voltage(a.voltage)
    for name in ('motore_sx','motore_dx'):robot.m.actuator(name).gear[0]*=a.motor_factor
    if a.imu_model=='mpu6500':
        robot.configure_sensors('mpu6500','ideal',ROOT/'config/mujoco_sensors_mpu6500.json',seed=a.seed)
    body=robot.m.body('struttura_centrale')
    eq=math.degrees(-math.atan2(body.ipos[1],body.ipos[2]))
    robot.start_support(eq+a.initial_error_deg)
    for _ in range(2000):robot.step()
    calibration=[]
    for _ in range(200):
        calibration.append(imu(robot))
        for _ in range(10):robot.step()
    initial=float(np.mean([r[0] for r in calibration]))-eq;bias=float(np.mean([r[1] for r in calibration]))
    joints=[int(robot.m.joint(n).qposadr[0]) for n in ['perno_sx','perno_dx']];origin=robot.d.qpos[joints].copy()
    rows=[];fault=None;fault_detail=[];commands=deque([(0.,0.)]*a.actuator_delay_samples)
    with tempfile.TemporaryDirectory() as d:
        lib=Path(d)/'hinf.so';source=ROOT/'scripts/hinf/hinf_real_sim_bridge.cpp'
        subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',str(source),'-o',str(lib)],check=True)
        core=ctypes.CDLL(str(lib));ptr=ctypes.POINTER(ctypes.c_double)
        core.hinf_create.argtypes=[ctypes.c_double]*3;core.hinf_create.restype=ctypes.c_void_p
        core.hinf_step.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_uint,ctypes.c_double,ctypes.c_double,ptr];core.hinf_step.restype=ctypes.c_int
        core.hinf_measure.argtypes=core.hinf_step.argtypes;core.hinf_measure.restype=ctypes.c_int
        core.hinf_allocate_only.argtypes=[ctypes.c_double]*4+[ptr];core.hinf_allocate_only.restype=ctypes.c_int
        core.hinf_destroy.argtypes=[ctypes.c_void_p]
        order=len(candidate.state) if candidate else 12
        handle=core.hinf_create(initial,a.max_pwm,min(3,a.max_pwm));out=(ctypes.c_double*(18+order))()
        if not handle:raise RuntimeError('Inizializzazione bridge Hinf fallita')
        robot.release()
        try:
            for i in range(1000):
                angle,rate,norm,az=imu(robot);rate-=bias;q=robot.d.qpos[joints]-origin
                sx=32768+int(np.rint(q[0]*1325/(2*math.pi)));dx=32768+int(np.rint(-q[1]*1325/(2*math.pi)))
                step=core.hinf_measure if candidate else core.hinf_step
                if not 0<sx<65535 or not 0<dx<65535 or not step(handle,sx,dx,angle-eq,rate,out):fault='encoder/core';break
                if candidate:
                    s,pitch,yaw,v,pd,yd=out[:6]
                    left,right=100*candidate.step([pitch-s/.032,pitch,yaw,pd-v/.032,pd,yd],.01)
                    allocated=(ctypes.c_double*4)()
                    if not core.hinf_allocate_only(left,right,a.max_pwm,min(3,a.max_pwm),allocated):fault='allocator';break
                    out[6:12]=[left,right,*allocated]
                    out[12:18]=candidate.last_feedback
                    out[18:]=candidate.state
                rows.append([i*.01,robot.true_pitch_deg()-eq,angle-eq,rate,norm,az,*out])
                fault_detail=[name for name,hit in [('fused_pitch_12deg',abs(out[1])>math.radians(12)),
                    ('accel_pitch_15deg',abs(angle-eq)>15),('gyro_180deg_s',abs(rate)>180),
                    ('accel_norm_0.6_1.4g',not .6<=norm<=1.4),('accel_z_8000counts',az<8000)] if hit]
                if fault_detail:fault='attitude/IMU';break
                fault_detail=[name for name,hit in [('position_0.12m',abs(out[0])>.12),
                    ('velocity_0.30m_s',abs(out[3])>.30),('yaw_20deg',abs(out[2])>math.radians(20)),
                    ('yaw_rate_90deg_s',abs(out[5])>math.radians(90))] if hit]
                if fault_detail:fault='odometry';break
                commands.append((out[8],out[9]));robot.set_motors(*commands.popleft())
                for _ in range(10):robot.step()
        finally:robot.set_motors(0,0);core.hinf_destroy(handle)
    with (a.output_dir/'simulation.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['t_s','true_pitch_error_deg','accel_error_deg','gyro_deg_s','accel_g','az',
            'position_m','pitch_rad','yaw_rad','velocity_m_s','pitch_rate_rad_s','yaw_rate_rad_s','sx_raw','dx_raw','sx_applied','dx_applied','common_raw','diff_raw',
            *['feedback_'+str(i) for i in range(6)],*['xc_'+str(i) for i in range(order)]]);w.writerows(rows)
    source=a.controller_json or ROOT/'models/hinf/simscape_2pwm_20261026.json'
    completed=len(rows)==1000 and fault is None
    tail=np.asarray(rows[-200:]) if completed else None
    stabilized=bool(completed and max(abs(tail[:,1]))<1 and max(abs(tail[:,9]))<.03 and max(abs(tail[:,8]))<math.radians(5))
    summary=dict(hardware_access=False,wheel_convention='physical_sx_dx_v2',controller_export_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),controller_order=order,
        voltage=a.voltage,max_pwm=a.max_pwm,initial_error_deg=initial,measurement_delay_samples=1,samples=len(rows),fault=fault,
        imu_model=a.imu_model,seed=a.seed,motor_factor=a.motor_factor,actuator_delay_samples=a.actuator_delay_samples,
        completed_10s=completed,fault_detail=fault_detail,stabilized_last_2s=stabilized,
        stabilization_criterion='Completed10s; last2s true pitch max<1deg, estimated speed max<0.03m/s, encoder yaw max<5deg. Diagnostic criterion, not physical validation.',
        tail_max_true_pitch_error_deg=max(abs(r[1]) for r in rows[-200:]) if completed else None,
        tail_true_pitch_rms_deg=float(np.sqrt(np.mean(tail[:,1]**2))) if completed else None,
        tail_velocity_max_m_s=float(max(abs(tail[:,9]))) if completed else None,
        tail_gyro_rms_deg_s=float(np.sqrt(np.mean(tail[:,3]**2))) if completed else None,
        saturated_fraction=sum(max(abs(r[14]),abs(r[15]))>=a.max_pwm-.001 for r in rows)/len(rows) if rows else None,
        peak_raw_pwm_percent=max(max(abs(r[12]),abs(r[13])) for r in rows) if rows else None,
        limitations='Nominal plant; optional approximate MPU register/filter model. Motor factor and added delay are hypothetical. No identified motor torque map, supply sag, support contact or exact real IMU transfer function. Not physical validation.')
    (a.output_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
    return 0 if summary['completed_10s'] else 1
if __name__=='__main__':raise SystemExit(main())
