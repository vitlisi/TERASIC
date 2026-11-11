"""Check guarded baseline logs; autonomy always requires operator observation."""
import csv,json,math,re
from pathlib import Path
import numpy as np


def analyze(folder,mode):
    raw=folder/'raw';status=(raw/'reale.status.log').read_text()
    with (raw/'reale.csv').open() as stream:rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(stream)]
    expected=1000 if mode=='run' else 300;flags=[]
    if len(rows)!=expected:flags.append('incomplete_sample_count')
    if 'LQR terminato; RUN SX=0 DX=0.' not in status:flags.append('final_stop_not_confirmed')
    if not rows:raise ValueError('No control samples; inspect startup log')
    t=(np.array([r['t_ns'] for r in rows])-rows[0]['t_ns'])*1e-9
    if len(t)>1 and (np.any(np.diff(t)<=0) or max(np.diff(t))>.025):flags.append('bad_timestamps')
    voltage=[r['battery_v'] for r in rows]
    if any(not 11.2<=r['battery_v']<=12.1 or not 0<=r['battery_age_s']<=.15 for r in rows):flags.append('battery_invalid_or_stale')
    for r in rows:
        if bool(r['armed'])!=(mode=='run'):flags.append('armed_state_mismatch');break
        for wheel in ['sx','dx']:
            limited=r['pwm_'+wheel+'_limited_percent'];actual=r['pwm_'+wheel+'_applied_percent']
            expected_pwm=0 if abs(limited)<.5 else math.copysign(int(50000*abs(limited)/100)/500,limited)
            if abs(actual)>20 or (mode=='shadow' and actual) or (mode=='run' and abs(actual-expected_pwm)>.0021):flags.append('physical_pwm_mismatch');break
    mask=(t>=2)&(t<9) if mode=='run' else np.ones(len(t),dtype=bool)
    if not mask.any():mask=np.ones(len(t),dtype=bool)
    pitch=np.array([r['pitch_fused_deg'] for r in rows]);gyro=np.array([r['gyro_deg_s'] for r in rows]);pos=np.array([r['position_m'] for r in rows])
    initial=re.search(r'errore iniziale=([-+0-9.]+) deg',status)
    result=dict(mode=mode,logging_complete=not flags,flags=sorted(set(flags)),samples=len(rows),initial_error_deg=float(initial[1]) if initial else None,battery_min_v=min(voltage),battery_max_v=max(voltage),pitch_rms_deg=float(np.sqrt(np.mean(pitch[mask]**2))),gyro_rms_deg_s=float(np.sqrt(np.mean(gyro[mask]**2))),position_span_mm=float(1000*np.ptp(pos[mask])),peak_physical_pwm=max(abs(r['pwm_'+wheel+'_applied_percent']) for r in rows for wheel in ['sx','dx']),window_s=[2,9] if mode=='run' else [0,3],operator_autonomy_confirmation=None,speed_lut=False,K_changed=False)
    (folder/'analysis.json').write_text(json.dumps(result,indent=2)+'\n');return result
