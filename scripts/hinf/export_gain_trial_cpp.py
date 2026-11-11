#!/usr/bin/env python3
"""Export only the reviewed12-state gain-trial to a SEPARATE C++ header."""
import hashlib
import json
import math
from pathlib import Path
import argparse
ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'models/hinf/stabilization_gain_trial_20261028.json'
HEADER=ROOT/'src_cpp/terasic_reale/controllers/balance_hinf_gain_trial_matrices.h'


def render():
    e=json.loads(SOURCE.read_text())
    assert e['controller']=='hinf_structured_gain_tuning_offline'
    assert e['tuning_common_gain']==1 and e['tuning_pitch_rate_input_gain']==.75
    assert e['schema_version']==1 and e['dt_s']==.01 and e['measurement_delay_samples']==1
    assert e['wheel_radius_m']==.032 and e['wheel_track_m']==.16444
    assert e['measurement_order']==['theta','psi','phi','theta_dot','psi_dot','phi_dot']
    assert e['measurement_units']==['rad']*3+['rad/s']*3
    assert e['output_order']==['PWM_R','PWM_L'] and e['output_unit']=='normalized_duty'
    assert e['input_convention']=='estimated_state_minus_reference_at_equilibrium'
    assert e['controller_initial_state']==[0]*12 and e['measurement_delay_initial_state']==[0]*6
    # Preinclude this header for the experimental binary only. Same guard skips
    # the immutable original header when balance_hinf_core.h includes it later.
    h='#ifndef TERASIC_HINF_MATRICES_H\n#define TERASIC_HINF_MATRICES_H\n'
    h+='// Experimental gain trial ONLY: never overwrite balance_hinf_matrices.h.\n'
    h+='#define HINF_GAIN_TRIAL_LABEL "gain_trial_20261028 common=1 pitch_rate_input=0.75; gamma not revalidated"\n'
    h+='static const char HINF_EXPORT_SHA256[]="'+hashlib.sha256(SOURCE.read_bytes()).hexdigest()+'";\n'
    for key,rows,cols in [('Ak',12,12),('Bk',12,6),('Ck',2,12),('Dk',2,6)]:
        a=e[key];assert len(a)==rows and all(len(row)==cols for row in a)
        assert all(math.isfinite(v) for row in a for v in row)
        h+=f'static const double HINF_{key}[{rows}][{cols}]={{\n'
        h+=',\n'.join('    {'+','.join(format(v,'.17g') for v in row)+'}' for row in a)+'\n};\n'
    return h+'#endif\n'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');args=p.parse_args()
    content=render()
    if args.check:assert HEADER.read_text()==content
    else:HEADER.write_text(content)
    print(HEADER)
