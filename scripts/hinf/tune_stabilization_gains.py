#!/usr/bin/env python3
"""Offline structured gain tuning of the existing Hinf; no synthesis or hardware."""
import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/hinf'))
from audit_real_model import closed_loop_radius


def candidate(parent,common_gain,pitch_rate_gain):
    """u_new=T u_old(S y); common output scaled, differential unchanged."""
    if not (0<common_gain<=1 and 0<pitch_rate_gain<=2):raise ValueError('Invalid tuning gains')
    e=copy.deepcopy(parent)
    t=np.eye(2)+(common_gain-1)*np.ones((2,2))/2
    s=np.diag([1,1,1,1,pitch_rate_gain,1])
    e['Bk']=(np.asarray(parent['Bk'])@s).tolist()
    e['Ck']=(t@np.asarray(parent['Ck'])).tolist()
    e['Dk']=(t@np.asarray(parent['Dk'])@s).tolist()
    e['controller']='hinf_structured_gain_tuning_offline'
    e['parent_gamma_not_valid_for_retuning']=e.pop('gamma',None)
    e.pop('closed_loop_spectral_radius',None)
    e.pop('lqr_with_delay_spectral_radius',None)
    e['parent_wheel_track_m']=parent['wheel_track_m']
    e['wheel_track_m']=.16444
    e['output_limit']=.20
    e['tuning_common_gain']=common_gain
    e['tuning_pitch_rate_input_gain']=pitch_rate_gain
    e['validation_status']='OFFLINE_ONLY_no_new_Hinf_norm_guarantee_not_real_validated'
    e['tuning_definition']='u_new=T_common*u_parent(S_pitch_rate*y); delay retained, no new hinfsyn'
    return e


def evaluate(config,folder,error,seed,cap):
    run=subprocess.run([sys.executable,str(ROOT/'scripts/hinf/screen_real_hinf.py'),
        '--controller-json',str(config.resolve()),'--output-dir',str(folder.resolve()),
        '--initial-error-deg',str(error),'--max-pwm',str(cap),'--voltage','11.05',
        '--imu-model','mpu6500','--seed',str(seed)],cwd=ROOT,capture_output=True,text=True)
    if not folder.exists() or not (folder/'summary.json').is_file():raise RuntimeError(run.stdout+run.stderr)
    (folder/'console.log').write_text(run.stdout+run.stderr)
    result=json.loads((folder/'summary.json').read_text())
    result.update(requested_error_deg=error,case_seed=seed)
    data=np.genfromtxt(folder/'simulation.csv',delimiter=',',names=True)
    if data.ndim==0:data=data.reshape(1)
    pitch=np.asarray(data['true_pitch_error_deg']);pwm=np.column_stack([data['sx_applied'],data['dx_applied']])
    # Fixed before search; penalties on command magnitude and sample-to-sample changes.
    result['cost']=float(np.mean((pitch/3.)**2)+.1*np.mean((pwm/cap)**2)+
        .05*np.mean((np.diff(pwm,axis=0)/cap)**2)) if len(pitch)>1 else 1e6
    result['pitch_rms_deg']=float(np.sqrt(np.mean(pitch**2)))
    result['pwm_rms_percent']=float(np.sqrt(np.mean(pwm**2)))
    result['pwm_total_variation_mean']=float(np.mean(np.sum(abs(np.diff(pwm,axis=0)),axis=0)))
    result['pwm_saturated_fraction']=float(np.mean(np.any(abs(pwm)>=cap-.001,axis=1)))
    return result


def rank(results):
    return (sum(not r['completed_10s'] for r in results),
            sum(not r['stabilized_last_2s'] for r in results),
            max(r['cost'] for r in results),float(np.mean([r['cost'] for r in results])))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--max-pwm',type=float,choices=[20.,25.],default=20.)
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=False)
    source=ROOT/'models/hinf/simscape_2pwm_20261026.json'
    parent=json.loads(source.read_text())
    plant_path=ROOT/'logs/terasic_reale/runs/diagnostics/hinf_offline_20261028_model_complete/mujoco_filtered_plant.json'
    plant=json.loads(plant_path.read_text())
    names=['models/hinf/simscape_2pwm_20261026.json','models/verifica.xml',
           'config/mujoco_sensors_mpu6500.json','scripts/hinf/tune_stabilization_gains.py',
           'scripts/hinf/screen_real_hinf.py','scripts/hinf/hinf_real_sim_bridge.cpp']
    manifest=dict(hardware_access=False,plant_parameters_changed=False,max_pwm=a.max_pwm,voltage=11.05,
        training_errors=[-3.6,3.],training_seed=0,validation_errors=[-5.,-3.,3.,5.,-3.6],validation_seeds=[1,2],
        ranking='Failures first, unsuccessful terminal stabilization second, worst training cost third, mean cost for ties.',
        cost='mean((true_pitch_deg/3)^2)+0.1*mean((PWM/cap)^2)+0.05*mean((delta_PWM/cap)^2)',
        selection='Training only; baseline and selected candidate validated on identical held-out seeds.',
        source_hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names},
        plant_sha256=hashlib.sha256(plant_path.read_bytes()).hexdigest())
    (a.output_dir/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    rows=[]
    for common in (.4,.6,.8,1.):
        for rate in (.75,1.,1.25):
            name=f'common{common:.2f}_rate{rate:.2f}';folder=a.output_dir/name;folder.mkdir()
            e=candidate(parent,common,rate)
            e['output_limit']=a.max_pwm/100.
            e['parent_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
            rho=closed_loop_radius(np.array(plant['A']),np.array(plant['B']),np.array(plant['C_measurement']),e)
            e['tuning_linear_radius_on_filtered_mujoco']=rho
            config=folder/'candidate.json';config.write_text(json.dumps(e,indent=2)+'\n')
            row=dict(name=name,common_gain=common,pitch_rate_gain=rate,rho=rho,config=str(config))
            if rho>=1:row.update(status='linear_reject',training=[],rank=[99,99,1e6])
            else:
                cases=[evaluate(config,folder/f'train_{error:+.1f}',error,0,a.max_pwm) for error in (-3.6,3.)]
                row.update(status='trained',training=cases,rank=list(rank(cases)))
            rows.append(row)
            (a.output_dir/'training.json').write_text(json.dumps(rows,indent=2)+'\n')
            print(name,'rank=',row['rank'],flush=True)
    best=min(rows,key=lambda r:r['rank'])
    baseline=next(r for r in rows if r['common_gain']==1 and r['pitch_rate_gain']==1)
    validation={}
    for label,row in [('baseline',baseline),('selected',best)]:
        cases=[]
        for error in (-5.,-3.,3.,5.,-3.6):
            for seed in (1,2):
                folder=a.output_dir/f'validation_{label}_{error:+.1f}_seed{seed}'
                cases.append(evaluate(Path(row['config']),folder,error,seed,a.max_pwm))
                validation[label]=cases
                (a.output_dir/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
                print('validation',label,error,seed,cases[-1]['completed_10s'],flush=True)
    result=dict(selected=best['name'],selected_config=best['config'],baseline_training_rank=baseline['rank'],
        selected_training_rank=best['rank'],baseline_validation_rank=list(rank(validation['baseline'])),
        selected_validation_rank=list(rank(validation['selected'])),
        note='Structured retuning only. No new Hinf norm guarantee; no real controller replaced.')
    (a.output_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()
