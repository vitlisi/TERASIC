#!/usr/bin/env python3
"""Generate ARM C++98 constants from the same immutable export used by MuJoCo."""
import argparse
import hashlib
import json
import math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]

def render(source):
    e=json.loads(source.read_text())
    assert e['schema_version']==1 and e['dt_s']==.01
    assert e['measurement_delay_samples']==1
    assert e['measurement_order']==['theta','psi','phi','theta_dot','psi_dot','phi_dot']
    assert e['measurement_units']==['rad']*3+['rad/s']*3
    assert e['input_convention']=='estimated_state_minus_reference_at_equilibrium'
    assert e['output_order']==['PWM_R','PWM_L'] and e['output_unit']=='normalized_duty'
    assert e['wheel_radius_m']==.032 and e['wheel_track_m']==.1644
    assert e['controller_initial_state']==[0]*12
    assert e['measurement_delay_initial_state']==[0]*6
    h='#ifndef TERASIC_HINF_MATRICES_H\n#define TERASIC_HINF_MATRICES_H\n// Generated from models/hinf/simscape_2pwm_20261026.json; do not tune here.\n'
    h+='static const char HINF_EXPORT_SHA256[]="'+hashlib.sha256(source.read_bytes()).hexdigest()+'";\n'
    for key,rows,cols in [('Ak',12,12),('Bk',12,6),('Ck',2,12),('Dk',2,6)]:
        a=e[key]
        assert len(a)==rows and all(len(row)==cols for row in a)
        assert all(math.isfinite(v) for row in a for v in row)
        h+=f'static const double HINF_{key}[{rows}][{cols}]={{\n'
        h+=',\n'.join('    {'+','.join(format(v,'.17g') for v in row)+'}' for row in a)+'\n};\n'
    return h+'#endif\n'

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    header=ROOT/'src_cpp/terasic_reale/controllers/balance_hinf_matrices.h'
    content=render(ROOT/'models/hinf/simscape_2pwm_20261026.json')
    if args.check:
        if header.read_text()!=content:parser.exit(1,'Matrici C++ diverse dall’export: rigenerare e verificare.\n')
        print('Matrici C++ identiche all’export.')
    else:
        header.write_text(content)
        print(header)
