"""Bench replay of a separate motor hypothesis; no hardware or default mutation."""
import math
import mujoco
import numpy as np
from compare_freewheel import fixed_model,WHEELS,update_wheels
from stateful_motor_model import configure,update


def replay(rows,cpr,wheels,stateful=None,physics_dt=.001):
    if not math.isfinite(physics_dt) or not 0 < physics_dt <= .001:
        raise ValueError('Physics timestep must be finite and in (0, 0.001] s')
    model,data,coefficient,details=fixed_model(wheels=wheels)
    if stateful is not None:
        configure(model,stateful)
        # Armature changes also affect constraint regularization through the
        # reference inverse inertia. Refresh it, as the actual robot runtime does.
        mujoco.mj_setConst(model,data)
        mujoco.mj_forward(model,data)
        details.update(stateful_profile=stateful, reference_constants_refreshed=True,
                       armature_kg_m2=model.dof_armature.tolist(),
                       reference_inverse_inertia=model.dof_invweight0.tolist())
    joints=[int(model.joint('perno_'+w).qposadr[0]) for w in WHEELS]
    dofs=[int(model.joint('perno_'+w).dofadr[0]) for w in WHEELS]
    initial_voltage=next(r['battery_v'] for r in rows if r['battery_v']>0);previous=0.;result=[]
    for row in rows:
        target=row['t_s']
        while previous<target-1e-12:
            model.opt.timestep=min(physics_dt,target-previous)
            if stateful is not None:update(model,data,data.ctrl*100,stateful)
            mujoco.mj_step(model,data);previous=float(data.time)
        voltage=row['battery_v'] or initial_voltage
        for index,w in enumerate(WHEELS):
            model.actuator('motore_'+w).gear[0]=-coefficient*voltage
            data.ctrl[index]=row['pwm_'+w+'_applied_percent']/100
        if stateful is None:update_wheels(model,data.ctrl*100,details['wheel_profiles'])
        else:update(model,data,data.ctrl*100,stateful)
        mujoco.mj_forward(model,data)
        if data.ncon:raise ValueError('Unexpected ground contact in fixed fixture')
        sample=dict(t_s=target,phase=row['stage'],battery_v=voltage)
        for index,w in enumerate(WHEELS):
            angle=float(data.qpos[joints[index]])
            sample.update({f'enc_{w}_counts':int(np.rint(angle*cpr/(2*math.pi)*(1 if w=='sx' else -1))),f'position_{w}_true_rad':-angle,f'velocity_{w}_true_rad_s':-float(data.qvel[dofs[index]])})
        result.append(sample)
    return result,details


def filtered_speed(rows,wheel,simulated=False):
    t=np.array([r['t_s'] for r in rows]);key='enc_'+wheel+('_counts' if simulated else '_raw');counts=np.array([r[key] for r in rows]);q=(counts-counts[0])*(-1 if wheel=='sx' else 1)*2*np.pi/1325;speed=np.zeros(len(t))
    for i in range(1,len(t)):
        dt=t[i]-t[i-1];a=math.exp(-dt/.05);speed[i]=a*speed[i-1]+(1-a)*(q[i]-q[i-1])/dt
    return t,speed
