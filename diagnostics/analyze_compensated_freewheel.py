"""Verify complete logs and replay the exact native compensator."""
import csv,ctypes,json,subprocess
from pathlib import Path
import numpy as np
from simulate_compensated_freewheel import ROOT,load_compensator,protocol

def analyze(folder,mode):
    folder=Path(folder);raw=folder/'raw';rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader((raw/'reale.csv').open())]
    flags=[]
    def check(ok,label):
        if not ok:flags.append(label)
    expected=2601 if mode=='run' else 301
    check(len(rows)==expected,'incomplete_sample_count')
    status=(raw/'reale.status.log').read_text()
    check((raw/'returncode.txt').read_text().strip()=='0' and 'result=COMPLETED' in status,'native_not_completed')
    check('final_control_sx=6 final_control_dx=6' in status,'native_final_OFF_missing')
    import re
    registers=(raw/'final_registers.txt').read_text()
    for wheel in ('SX','DX'):
        m=re.search(r'Motore '+wheel+r':\s+periodo=(\d+) alto=(\d+) controllo=0x([0-9a-fA-F]+)',registers)
        check(bool(m) and int(m[2])==0 and int(m[3],16)&1==0,'final_OFF_'+wheel)
    if not rows:return dict(accepted=False,flags=flags+['empty_log'],samples=0)
    check(all(np.isfinite(list(r.values())).all() for r in rows),'nonfinite_values')
    t=np.array([r['t_s'] for r in rows]);dt=np.diff(t)
    check(0<=t[0]<.025 and expected/100-.011<=t[-1]<expected/100+.025 and (dt>0).all() and (dt<=.025).all(),'timing')
    library=folder/'replay_compensator.so'
    subprocess.run(['g++','-std=c++98','-O2','-shared','-fPIC',str(ROOT/'diagnostics/stribeck_compensation_bridge.cpp'),'-o',str(library)],check=True)
    lib=load_compensator(library.resolve());h=lib.stribeck_comp_create();errors=dict(pwm=0.,speed=0.,target=0.,correction=0.)
    try:
        for i,r in enumerate(rows):
            check(r['sample']==i and r['armed']==(mode=='run'),'sample_or_armed_mismatch')
            check(11.2<=r['battery_v']<=12.1 and 0<=r['battery_age_s']<=.15,'battery')
            v=(ctypes.c_double*11)()
            check(bool(lib.stribeck_comp_step(h,r['nominal_sx_percent'],r['nominal_dx_percent'],int(r['enc_sx_raw']),int(r['enc_dx_raw']),r['t_s'],r['battery_v'],v)),'invalid_replay')
            for j,w in enumerate(('sx','dx')):
                check(r['nominal_'+w+'_percent']==protocol(i/100),'nominal_protocol')
                actual=r['pwm_'+w+'_applied_percent'];check(abs(actual)<=8.000001,'PWM_cap')
                check(abs(r['correction_'+w+'_percent'])<=.500001,'correction_cap')
                errors['pwm']=max(errors['pwm'],abs(actual-(v[j] if mode=='run' else 0)))
                for key,index,field in [('speed',2,'filtered_speed_{}_rad_s'),('correction',6,'correction_{}_percent'),('target',8,'target_{}_percent')]:errors[key]=max(errors[key],abs(r[field.format(w)]-v[index+j]))
    finally:lib.stribeck_comp_destroy(h)
    check(errors['pwm']<=.002001,'PWM_replay_mismatch')
    for key in ('speed','target','correction'):check(errors[key]<=3e-6,key+'_replay_mismatch')
    return dict(accepted=not flags,flags=sorted(set(flags)),samples=len(rows),duration_s=float(t[-1]),replay_max_errors=errors,battery_min_v=min(r['battery_v'] for r in rows),battery_max_v=max(r['battery_v'] for r in rows))
