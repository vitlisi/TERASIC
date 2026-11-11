"""Raised-wheel PWM pulses with current Stribeck physics and exact C++ compensation.

Pure simulation: never relabel uncompensated real records as compensated data.
Both wheels receive the same nominal profile; each has its own encoder feedback
and friction branch. Chassis fixed, no ground contact, 100 Hz control, 1 kHz physics.
"""
import argparse,ctypes,hashlib,json,os,subprocess
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/robot01_compensated_freewheel')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import mujoco
from compare_freewheel import fixed_model,write_csv
from stateful_motor_model import configure,update
ROOT=Path(__file__).resolve().parents[1]


def load_compensator(library):
    lib=ctypes.CDLL(str(library))
    lib.stribeck_comp_create.argtypes=[];lib.stribeck_comp_create.restype=ctypes.c_void_p
    lib.stribeck_comp_destroy.argtypes=[ctypes.c_void_p];lib.stribeck_comp_destroy.restype=None
    lib.stribeck_comp_step.argtypes=[ctypes.c_void_p,ctypes.c_double,ctypes.c_double,ctypes.c_uint,ctypes.c_uint,ctypes.c_double,ctypes.c_double,ctypes.POINTER(ctypes.c_double)]
    lib.stribeck_comp_step.restype=ctypes.c_int
    return lib


def protocol(t,sign=-1):
    values=[4,0,6,0,8,0,-4,0,-6,0,-8,0]
    segment=int((t-1)/2)
    return float(sign*values[segment]) if 1<=t<25 else 0.


def simulate(rows,profile,library,enabled):
    base=json.loads((ROOT/'config/mujoco_motors_sx_dx_20261030.json').read_text())['wheels']
    model,data,coefficient,details=fixed_model(wheels=base)
    configure(model,profile);mujoco.mj_setConst(model,data);mujoco.mj_forward(model,data)
    body=model.body('struttura_centrale').id;origin=data.xpos[body].copy();rotation=data.xmat[body].copy()
    handle=library.stribeck_comp_create();results=[];previous_t=0.
    max_contacts=0;max_body_motion=0.;max_body_rotation=0.
    try:
        for row in rows:
            target=row['t_s']
            while previous_t<target-1e-12:
                model.opt.timestep=min(.001,target-previous_t)
                update(model,data,data.ctrl*100,profile);mujoco.mj_step(model,data);previous_t=float(data.time)
                max_contacts=max(max_contacts,int(data.ncon))
                max_body_motion=max(max_body_motion,float(np.linalg.norm(data.xpos[body]-origin)))
                max_body_rotation=max(max_body_rotation,float(np.max(abs(data.xmat[body]-rotation))))
            counts=[]
            for w,sign in [('sx',1),('dx',-1)]:
                q=int(model.joint('perno_'+w).qposadr[0])
                counts.append(32768+int(np.rint(float(data.qpos[q])*1325/(2*np.pi)*sign)))
            v=(ctypes.c_double*11)()
            if not library.stribeck_comp_step(handle,row['nominal_sx_percent'],row['nominal_dx_percent'],counts[0],counts[1],target,row['battery_v'],v):
                raise ValueError('Compensator input invalid')
            result=dict(t_s=target,battery_v=row['battery_v'],compensation_enabled=enabled)
            for i,w in enumerate(('sx','dx')):
                nominal=row['nominal_'+w+'_percent']
                # This protocol uses exact PWM quanta and identical nominal
                # commands, so baseline pair allocation is unchanged.
                baseline=0. if abs(nominal)<.5 else np.sign(nominal)*int(500*abs(nominal))/500
                physical=float(v[i]) if enabled else float(baseline)
                model.actuator('motore_'+w).gear[0]=-coefficient*row['battery_v'];data.ctrl[i]=physical/100
                result.update({f'nominal_{w}_percent':nominal,f'applied_{w}_percent':physical,
                    f'enc_{w}_raw':counts[i],f'filtered_speed_{w}_rad_s':float(v[2+i]),
                    f'correction_{w}_percent':float(v[6+i]) if enabled else 0.})
            update(model,data,data.ctrl*100,profile);mujoco.mj_forward(model,data)
            if data.ncon:raise ValueError('Unexpected wheel-floor contact')
            results.append(result)
    finally:library.stribeck_comp_destroy(handle)
    details.update(nv=int(model.nv),max_contacts=max_contacts,max_chassis_translation_m=max_body_motion,
                   max_chassis_rotation_matrix_difference=max_body_rotation,encoder_counts_per_rev=1325,encoder_filter_s=.05,
                   physical_dt_max_s=.001,controller_rate_hz=100,compensation_enabled=enabled)
    if max_contacts or max_body_motion or max_body_rotation:raise ValueError('Fixture not fixed/contact-free')
    return results,details


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--profile',type=Path,default=ROOT/'config/mujoco_robot01_stribeck.json')
    p.add_argument('--voltage',type=float,default=11.5);a=p.parse_args()
    if not np.isfinite(a.voltage) or not 11.2<=a.voltage<=12.1:raise ValueError('Supply must be in 11.20..12.10 V')
    out=a.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    library=out/'compensator.so';source=ROOT/'diagnostics/stribeck_compensation_bridge.cpp'
    subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',str(source),'-o',str(library)],check=True)
    lib=load_compensator(library);profile=json.loads(a.profile.read_text())
    rows=[dict(t_s=i/100,nominal_sx_percent=protocol(i/100),nominal_dx_percent=protocol(i/100),battery_v=a.voltage) for i in range(2601)]
    off,off_details=simulate(rows,profile,lib,False);on,on_details=simulate(rows,profile,lib,True)
    write_csv(out/'requested_profile.csv',rows);write_csv(out/'mujoco_compensation_OFF.csv',off);write_csv(out/'mujoco_compensation_ON.csv',on)
    fig,axes=plt.subplots(3,2,figsize=(15,10),sharex=True)
    for col,w in enumerate(('sx','dx')):
        t=np.array([r['t_s'] for r in rows]);base=np.array([r['filtered_speed_'+w+'_rad_s'] for r in off]);comp=np.array([r['filtered_speed_'+w+'_rad_s'] for r in on])
        axes[0,col].plot(t,[r['nominal_'+w+'_percent'] for r in rows],color='#555555',ls='--',lw=.9,label='Nominal PWM')
        axes[0,col].plot(t,[r['applied_'+w+'_percent'] for r in on],color='#198038',lw=1,label='Compensated physical PWM')
        axes[1,col].plot(t,base,color='#2081c3',lw=1,label='Stribeck, compensation OFF')
        axes[1,col].plot(t,comp,color='#198038',ls='--',lw=1,label='Stribeck, compensation ON')
        axes[2,col].plot(t,comp-base,color='#713d9b',lw=.9);axes[2,col].axhline(0,color='black',lw=.6)
        axes[0,col].set_title('Left wheel (SX)' if w=='sx' else 'Right wheel (DX)')
        axes[0,col].legend(fontsize=8);axes[1,col].legend(fontsize=8)
        for ax in axes[:,col]:ax.set_xlim(0,26);ax.grid(alpha=.2);ax.set_xlabel('Time since test start [s]')
    for ax,label in zip(axes[:,0],['PWM [%]','Wheel speed [rad/s]','Speed change ON - OFF [rad/s]']):ax.set_ylabel(label)
    fig.suptitle('Robot01 — raised-wheel PWM pulses with Stribeck and compensation\nSimulation only: identical nominal commands, compensation OFF versus ON',fontsize=14,y=.995)
    fig.text(.5,.025,'Fixed chassis, no ground contact; same C++ compensator as the real LQR trial, correction bounded to +/-0.5 PWM percentage points.\n'
             f'Both wheels tested together at {a.voltage:g} V; quantized encoders and 50 ms filter. Full 26 s from t=0.\n'
             'No compensated real raised-wheel measurement is included: the existing real pulse records had compensation OFF.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.10,1,.94))
    for ext in ('png','svg'):fig.savefig(out/('raised_wheel_compensation_ON_OFF.'+ext),dpi=150)
    plt.close(fig)
    inputs=[Path(__file__),a.profile.resolve(),source,ROOT/'diagnostics/stribeck_lut_compensation.h',ROOT/'diagnostics/stribeck_compensation_data.h',ROOT/'src_python/stribeck_motor.py',ROOT/'models/verifica.xml']
    summary=dict(real_compensated_measurement_available=False,simulation_only=True,hardware_commands=0,
        requested_pwm_magnitudes=[4,6,8],supply_v=a.voltage,off_fixture=off_details,on_fixture=on_details,
        max_compensation_percent=max(abs(r['correction_'+w+'_percent']) for r in on for w in ('sx','dx')),
        inputs_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(out/'raised_wheel_compensation_ON_OFF.png',flush=True)


if __name__=='__main__':main()
