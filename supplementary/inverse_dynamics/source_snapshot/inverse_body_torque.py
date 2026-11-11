"""Offline planar body inverse dynamics, conditional on no slip and CAD parameters.
No fitting, actuation, or change to model/controller defaults.
Positive pitch leans toward +Y; positive wheel torque drives toward +Y.
"""
import argparse,csv,hashlib,json,os,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src_python'),str(ROOT/'scripts')]
os.environ.setdefault('MPLCONFIGDIR','/tmp/inverse_body_torque')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mujoco_robot import MujocoRobot
from twin_payload import add_centered_payload,body_inertia
from stribeck_motor import friction_parameters

def local_polynomial(t,y,window=11):
    """Cubic local least squares on actual timestamps; derivatives in SI units."""
    if window%2!=1 or window<5 or len(t)<window:raise ValueError('Odd window >=5 required')
    y=np.asarray(y);out=np.empty((len(t),3));half=window//2
    for i in range(len(t)):
        start=min(max(i-half,0),len(t)-window);ids=slice(start,start+window);x=t[ids]-t[i];scale=max(abs(x));z=x/scale
        c=np.linalg.lstsq(np.vander(z,4,increasing=True),y[ids],rcond=None)[0]
        out[i]=[c[0],c[1]/scale,2*c[2]/scale**2]
    return out

def filtered_interval_width(t,width,window):
    """Propagate bounded unknown dry friction through signed cubic filter weights."""
    out=np.empty(len(t));half=window//2
    for i in range(len(t)):
        start=min(max(i-half,0),len(t)-window);ids=slice(start,start+window);x=t[ids]-t[i];z=x/max(abs(x));weights=np.linalg.pinv(np.vander(z,4,increasing=True))[0]
        out[i]=np.dot(abs(weights),width[ids])
    return out

def body_torque(theta,alpha,xdd,mass,cy,cz,ixx,zdd=0):
    ry=cy*np.cos(theta)+cz*np.sin(theta);rz=-cy*np.sin(theta)+cz*np.cos(theta)
    return mass*(9.81+zdd)*ry-mass*rz*xdd-(ixx+mass*(cy*cy+cz*cz))*alpha

def parameters(payload=.02):
    robot=MujocoRobot();reference_equilibrium_rad=float(-np.arctan2(robot.m.body_ipos[robot.body_id,1],robot.m.body_ipos[robot.body_id,2]));add_centered_payload(robot,payload,.1);m=robot.m;b=robot.body_id
    return dict(reference_equilibrium_rad=reference_equilibrium_rad,mass=float(m.body_mass[b]),cy=float(m.body_ipos[b,1]),cz=float(m.body_ipos[b,2]),ixx=float(body_inertia(m,b)[0,0]),payload_mass_kg=payload,payload_height_m=.1)

def read(path):
    with path.open() as f:rows=list(csv.DictReader(f))
    return {k:np.array([float(r[k]) for r in rows]) for k in rows[0]}

def reconstruct(d,profile,p,window=11,offset_deg=0):
    t=(d['t_ns']-d['t_ns'][0])*1e-9
    psi=local_polynomial(t,np.radians(d['pitch_fused_deg']),window)[:,0]
    gyro=local_polynomial(t,np.radians(d['gyro_deg_s']),window)
    theta=psi+p['reference_equilibrium_rad']+np.radians(offset_deg);alpha=gyro[:,1]
    phi=(np.column_stack((d['enc_sx_raw'],d['enc_dx_raw']))-32768)*np.array([-1,1])*2*np.pi/1325
    wheel=[local_polynomial(t,phi[:,j],window) for j in range(2)]
    xdd=.032*(.5*(wheel[0][:,2]+wheel[1][:,2])+alpha)
    tau=body_torque(theta,alpha,xdd,p['mass'],p['cy'],p['cz'],p['ixx'])
    drive=np.zeros(len(t));loss=np.zeros(len(t));rotor=np.zeros(len(t));static_width=np.zeros(len(t));allmoving=np.ones(len(t),bool)
    for j,w in enumerate(('sx','dx')):
        pwm=d['pwm_'+w+'_applied_percent'];omega=wheel[j][:,1];phidd=wheel[j][:,2]
        rawdrive=.441/12*d['battery_v']*pwm/100
        dry=np.array([friction_parameters(profile['wheels'][w],v,u)[1] for v,u in zip(omega,pwm)])
        damping=np.array([friction_parameters(profile['wheels'][w],v,u)[0] for v,u in zip(omega,pwm)])
        stuck=abs(omega)<.1;allmoving &= ~stuck
        # At rest dry friction is set-valued; do not invent its sign from PWM.
        rawloss=damping*omega+np.where(stuck,0,np.sign(omega)*dry)
        drive+=local_polynomial(t,rawdrive,window)[:,0]
        loss+=local_polynomial(t,rawloss,window)[:,0]
        rotor+=profile['wheels'][w]['armature_kg_m2']*phidd
        static_width+=filtered_interval_width(t,np.where(stuck,dry,0),window)
    predicted=drive-loss-rotor
    position_from_counts=.032*(phi.mean(axis=1)+np.radians(d['pitch_fused_deg']-d['pitch_fused_deg'][0]))
    return dict(t_s=t,body_torque_Nm=tau,predicted_body_torque_Nm=predicted,drive_torque_Nm=drive,friction_loss_Nm=loss,armature_inertia_torque_Nm=rotor,static_halfwidth_Nm=static_width,alpha_rad_s2=alpha,axle_acceleration_m_s2=xdd,all_wheels_moving=(static_width<1e-12),position_reconstruction_m=position_from_counts)

def scores(r):
    mask=(r['t_s']>=2)&(r['t_s']<9);e=r['body_torque_Nm']-r['predicted_body_torque_Nm'];dist=np.maximum(abs(e)-r['static_halfwidth_Nm'],0);moving=mask&r['all_wheels_moving']
    rms=lambda x,m:float(np.sqrt(np.mean(x[m]**2))) if m.any() else None
    return dict(window_s=[2,9],body_torque_rms_Nm=rms(r['body_torque_Nm'],mask),predicted_body_torque_rms_Nm=rms(r['predicted_body_torque_Nm'],mask),center_residual_rms_Nm=rms(e,mask),static_interval_distance_rms_Nm=rms(dist,mask),moving_samples=int(moving.sum()),moving_residual_rms_Nm=rms(e,moving),correlation=float(np.corrcoef(r['body_torque_Nm'][mask],r['predicted_body_torque_Nm'][mask])[0,1]))

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    profile_path=ROOT/'config/mujoco_robot01_stribeck.json';profile=json.loads(profile_path.read_text());p=parameters()
    sources={'latest':ROOT/'logs/terasic_reale/robots/robot01/stribeck_loaded_validation_20261109_01/run01/raw/reale.csv','independent_earlier':ROOT/'logs/terasic_reale/robots/robot01/stribeck_compensation_20261109_01/run01/raw/reale.csv','synthetic_sensor_control':ROOT/'experiments/robot01_loaded_validation_20261109/matched_run01/revised_negative_compensated/simulation.csv'}
    report=dict(method='Planar rigid-body pitch moment balance; no-slip axle acceleration from raw encoder counts plus body pitch; total effective axle torque, not separate wheel torques.',parameters=p,fit_performed=False,hardware_commands=0,profile_sha256=hashlib.sha256(profile_path.read_bytes()).hexdigest(),trials={})
    for name,path in sources.items():
        d=read(path)
        if 'battery_v' not in d:d['battery_v']=d['supply_voltage_V']
        runs={w:reconstruct(d,profile,p,w) for w in (7,11,15)};r=runs[11];t=r['t_s'];cases=[]
        for label,pp,offset in [('nominal',p,0),('no_assumed_payload',parameters(0),0),('mass_minus10',dict(p,mass=p['mass']*.9),0),('mass_plus10',dict(p,mass=p['mass']*1.1),0),('com_height_minus10',dict(p,cz=p['cz']*.9),0),('com_height_plus10',dict(p,cz=p['cz']*1.1),0),('inertia_minus20',dict(p,ixx=p['ixx']*.8),0),('inertia_plus20',dict(p,ixx=p['ixx']*1.2),0),('alignment_minus0p5',p,-.5),('alignment_plus0p5',p,.5)]:
            cases.append(dict(case=label,metrics=scores(reconstruct(d,profile,pp,11,offset))))
        with (a.output/(name+'_torques.csv')).open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(r));writer.writeheader();writer.writerows({k:v[i] for k,v in r.items()} for i in range(len(t)))
        metrics=scores(r);report['trials'][name]=dict(source=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),metrics=metrics,filter_sensitivity={str(w):scores(rr) for w,rr in runs.items()},parameter_sensitivity=cases,logged_position_reconstruction_max_error_m=float(max(abs(r['position_reconstruction_m']-(d['position_m']-d['position_m'][0])))))
        fig,axes=plt.subplots(3,1,figsize=(13,10),sharex=True)
        axes[0].plot(t,r['body_torque_Nm'],color='#c62828',lw=.9,label='Inferred from body motion')
        axes[0].plot(t,r['predicted_body_torque_Nm'],color='#198038',lw=.9,label='Frozen motor model: drive - losses - armature inertia')
        axes[0].fill_between(t,r['predicted_body_torque_Nm']-r['static_halfwidth_Nm'],r['predicted_body_torque_Nm']+r['static_halfwidth_Nm'],color='#198038',alpha=.15,label='Static-friction feasible range near zero speed')
        for key,label,color in [('drive_torque_Nm','Electrical drive','#2081c3'),('friction_loss_Nm','Velocity/dry losses','#d47716'),('armature_inertia_torque_Nm','Equivalent armature inertia','#713d9b')]:axes[1].plot(t,r[key],label=label,color=color,lw=.8)
        for w,rr in runs.items():axes[2].plot(t,rr['body_torque_Nm'],lw=.8,label=f'Body estimate: {w}-sample local cubic')
        for ax in axes:ax.set_xlim(0,10);ax.grid(alpha=.2);ax.set_ylabel('Total torque [Nm]');ax.legend(fontsize=8,loc='upper right');ax.axvspan(0,.2,color='gray',alpha=.12);ax.axvspan(9.8,10,color='gray',alpha=.12)
        axes[-1].set_xlabel('Time since active control start [s]');fig.suptitle('Robot01 — inverse dynamics from body motion: '+name.replace('_',' ')+'\nConditional planar estimate; full acquisition from t=0, no parameter fit')
        fig.text(.5,.015,'CAD body + assumed 20 g payload at 0.1 m; no slip, fixed axle height, negligible yaw/roll, assumed IMU alignment.\n11-sample local cubic reconstruction at ~100 Hz; shaded edges have one-sided derivatives. Static friction is not uniquely determined at rest.',ha='center',fontsize=9);fig.tight_layout(rect=(0,.06,1,.94))
        for ext in ('png','svg'):fig.savefig(a.output/(name+'_inverse_torque.'+ext),dpi=150)
        plt.close(fig)
    report['input_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in [ROOT/'models/verifica.xml',ROOT/'config/controllers/lqr_mimo.txt',ROOT/'scripts/twin_payload.py',ROOT/'src_python/stribeck_motor.py',profile_path]}
    report['code_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();(a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v['metrics'] for k,v in report['trials'].items()},indent=2))
if __name__=='__main__':main()
