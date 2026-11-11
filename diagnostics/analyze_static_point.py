"""Classify one static motor point; never constructs a LUT or hides rejected data."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np


def analyze(folder, *, rows=None, write_result=False):
    meta=dict(line.split('=',1) for line in (folder/'point.txt').read_text().splitlines() if '=' in line)
    wheel=meta['wheel'];direction=1 if meta['direction']=='forward' else -1
    cpr=None
    for line in (folder/'reale.config.txt').read_text().splitlines():
        fields=line.split('#')[0].split()
        if fields and fields[0]=='counts_per_rev':cpr=float(fields[1])
    if not cpr or cpr<=0:raise ValueError('Missing CPR')
    if rows is None:
        with (folder/'reale.csv').open() as stream:
            rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(stream)]
    reasons=[];status=(folder/'reale.status.log').read_text()
    if 'result=COMPLETED' not in status or 'final_control_sx=6 final_control_dx=6' not in status:reasons.append('acquisition_incomplete_or_not_stopped')
    if len(rows)<3:raise ValueError('Insufficient samples')
    t=np.array([r['t_s'] for r in rows]);dt=np.diff(t)
    if np.any(dt<=0) or np.max(dt)>.025:raise ValueError('Invalid timestamps')
    counts=np.array([r[f'enc_{wheel}_raw'] for r in rows]);position=(counts-counts[0])*(-1 if wheel=='sx' else 1)*2*math.pi/cpr
    speed=np.zeros(len(t))
    for k in range(1,len(t)):
        alpha=math.exp(-dt[k-1]/.05);speed[k]=alpha*speed[k-1]+(1-alpha)*(position[k]-position[k-1])/dt[k-1]
    start=1+(2 if meta['mode']=='moving' else 0);end=start+float(meta['hold_s'])
    tail=np.flatnonzero((t>=end-.52)&(t<end-.02));previous=np.flatnonzero((t>=end-1.02)&(t<end-.52))
    if len(tail)<45 or len(previous)<45:raise ValueError('Missing steady-state windows')
    measured=np.array([r['stage']==2 for r in rows]);other='dx' if wheel=='sx' else 'sx'
    for r in rows:
        if abs(r[f'pwm_{other}_applied_percent'])>1e-6 or int(r[f'control_{other}'])&1:reasons.append('other_wheel_commanded');break
    for r in rows:
        if abs(r[f'pwm_{wheel}_applied_percent']-r['pwm_requested_percent'])>.0021:reasons.append('pwm_readback_mismatch');break
    if any(not 10.5<=rows[i]['battery_v']<=13 or not 0<=rows[i]['battery_age_s']<=.15 for i in np.flatnonzero(measured)):
        reasons.append('invalid_or_stale_voltage')
    voltage=float(np.mean([rows[i]['battery_v'] for i in tail]));reference=float(meta['voltage_reference_v']);tolerance=float(meta['voltage_plateau_tolerance_v'])
    if abs(voltage-reference)>tolerance:reasons.append('voltage_outside_frozen_comparability_band')
    mean=float(np.mean(speed[tail]));std=float(np.std(speed[tail],ddof=1));excursion=float(np.ptp(counts[tail]))
    drift=abs(mean-float(np.mean(speed[previous])));internal=abs(float(np.mean(speed[tail[len(tail)//2:]]))-float(np.mean(speed[tail[:len(tail)//2]])))
    magnitude=float(meta['pwm_magnitude_percent']);zero=magnitude==0
    if zero:
        if excursion>1 or abs(mean)>.08:reasons.append('nonzero_motion_at_zero_command')
    else:
        if mean*direction<=0 or excursion<6:reasons.append('no_resolved_regular_motion')
        if abs(mean)>1e-12 and std/abs(mean)>.15:reasons.append('relative_velocity_std_above_15_percent')
        if drift>max(.08,.1*abs(mean)) or internal>max(.08,.1*abs(mean)):reasons.append('not_settled')
        directed=np.diff(counts[tail])*(-1 if wheel=='sx' else 1)*direction
        if np.any(directed<-1):reasons.append('wrong_direction_steps')
        pause=maximum=0.
        for step,step_dt in zip(np.diff(counts[tail]),np.diff(t[tail])):
            pause=pause+float(step_dt) if step==0 else 0.;maximum=max(maximum,pause)
        if maximum>.10:reasons.append('encoder_motion_intermittent')
    result=dict(wheel=wheel,direction=meta['direction'],mode=meta['mode'],pwm_magnitude_percent=magnitude,
        accepted_static_point=not reasons,rejection_reasons=reasons,zero_anchor=zero,
        velocity_sign_convention='forward_positive_physical_sx_dx_v2',cpr_used=cpr,mean_velocity_rad_s=mean,std_velocity_rad_s=std,
        mean_velocity_counts_s=mean*cpr/(2*math.pi),tail_excursion_counts=excursion,
        previous_window_drift_rad_s=drift,internal_drift_rad_s=internal,plateau_voltage_mean_v=voltage,
        window_start_s=end-.52,window_end_s=end-.02)
    if write_result:
        (folder/'point_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path);args=parser.parse_args()
    print(json.dumps(analyze(args.folder),indent=2))
