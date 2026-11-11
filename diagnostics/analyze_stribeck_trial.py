"""Analyze a qualified bounded-compensation trial; exact C++ replay, no actuation."""
import csv
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from analyze_guarded_balance import analyze as analyze_baseline
ROOT=Path(__file__).resolve().parents[1]
QUALIFICATION=ROOT/'config/qualification/lqr_stribeck_arm.json'


def analyze(folder,mode):
    qualification=json.loads(QUALIFICATION.read_text())
    for name in ('diagnostics/stribeck_lut_compensation.h','diagnostics/stribeck_compensation_data.h'):
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=qualification['source_sha256'][name]:
            raise ValueError('Local compensation source differs from qualified ARM source')
    result=analyze_baseline(folder,mode)
    flags=[x for x in result['flags'] if x!='physical_pwm_mismatch']
    with (folder/'raw/reale.csv').open() as stream:rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(stream)]
    source=ROOT/'diagnostics/stribeck_compensation_bridge.cpp'
    checks=[]
    with tempfile.TemporaryDirectory(prefix='stribeck_replay_') as tmp:
        libpath=Path(tmp)/'comp.so'
        subprocess.run(['g++','-std=c++98','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',str(source),'-o',str(libpath)],check=True)
        lib=ctypes.CDLL(str(libpath));lib.stribeck_comp_create.argtypes=[];lib.stribeck_comp_create.restype=ctypes.c_void_p
        lib.stribeck_comp_destroy.argtypes=[ctypes.c_void_p]
        lib.stribeck_comp_step.argtypes=[ctypes.c_void_p,ctypes.c_double,ctypes.c_double,ctypes.c_uint,ctypes.c_uint,ctypes.c_double,ctypes.c_double,ctypes.POINTER(ctypes.c_double)]
        lib.stribeck_comp_step.restype=ctypes.c_int
        handle=lib.stribeck_comp_create()
        try:
            first=rows[0]['t_ns']
            for r in rows:
                t=(r['t_ns']-first)*1e-9;out=(ctypes.c_double*11)()
                valid=lib.stribeck_comp_step(handle,r['pwm_sx_limited_percent'],r['pwm_dx_limited_percent'],
                    int(r['enc_sx_raw']),int(r['enc_dx_raw']),t,r['battery_v'],out)
                if not valid:flags.append('compensator_input_invalid');break
                check=dict(t_s=t)
                for w,i in [('sx',0),('dx',1)]:
                    actual=r['pwm_'+w+'_applied_percent'];expected=out[i] if mode=='run' else 0.
                    check[w+'_physical_difference']=abs(actual-expected)
                    check[w+'_target_difference']=abs(r['pwm_'+w+'_compensation_target_percent']-out[8+i])
                    check[w+'_omega_difference']=abs(r['motion_'+w+'_rad_s']-out[2+i])
                    check[w+'_correction_percent']=out[6+i]
                    if abs(actual-expected)>.002001:flags.append('compensated_physical_pwm_mismatch')
                    if check[w+'_target_difference']>2e-6 or check[w+'_omega_difference']>3e-6:flags.append('compensator_shadow_replay_mismatch')
                    if abs(actual)>20.000001 or abs(out[6+i])>.500001:flags.append('compensation_bound_exceeded')
                if abs(r['pwm_sx_applied_percent']-r['pwm_dx_applied_percent'])>6.000001:flags.append('differential_cap_exceeded')
                checks.append(check)
        finally:lib.stribeck_comp_destroy(handle)
    result.update(flags=sorted(set(flags)),logging_complete=not flags,
        compensation='STRIBECK_MODEL_MATCHING_BOUNDED_0.5',candidate_sha256=qualification['candidate_sha256'],
        replay_samples=len(checks),exact_compensator_source_replayed=True,
        maximum_physical_replay_difference_percent=max((r[w+'_physical_difference'] for r in checks for w in ('sx','dx')),default=None),
        maximum_omega_replay_difference_rad_s=max((r[w+'_omega_difference'] for r in checks for w in ('sx','dx')),default=None),
        maximum_compensation_percent=max((abs(r[w+'_correction_percent']) for r in checks for w in ('sx','dx')),default=None))
    if checks:
        with (folder/'compensator_replay.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(checks[0]));writer.writeheader();writer.writerows(checks)
    (folder/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
