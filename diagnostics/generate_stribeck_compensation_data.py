"""Reproduce the C++ loss LUT from its frozen, source-checked candidate JSON."""
import argparse,hashlib,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--profile',type=Path,default=ROOT/'config/motors/robot01_compensation_source_profile.json');a=p.parse_args()
    if a.output.exists():raise ValueError('Choose a new output file')
    record=json.loads(a.candidate.read_text())
    if record['robot']!='robot01' or record['kind']!='bounded_friction_model_matching_lut':raise ValueError('Wrong candidate')
    for name,value in [('correction_cap_percent',.5),('correction_rate_percent_s',10),('encoder_filter_s',.05),
                       ('stationary_window_s',.1),('request_persistence_s',.03),('identity_from_percent',8)]:
        if record[name]!=value:raise ValueError('Candidate settings differ from the qualified C++ implementation')
    profile_path=a.profile;reference_path=ROOT/'config/mujoco_motors_sx_dx_20261030.json'
    if sha(profile_path)!=record['profile_sha256'] or sha(reference_path)!=record['reference_sha256']:raise ValueError('Input source changed')
    profile=json.loads(profile_path.read_text());reference=json.loads(reference_path.read_text())
    size=len(record['branches'][0]['nodes'])
    lines=['// Generated from the frozen friction-model matching candidate.','#ifndef STRIBECK_COMPENSATION_DATA_H',
           '#define STRIBECK_COMPENSATION_DATA_H',f'static const char *stribeck_lut_sha256="{sha(a.candidate)}";',
           'static const unsigned stribeck_lut_size='+str(size)+';', 'struct StribeckLossNode { double speed,delta_torque; };']
    for i,(wheel,direction) in enumerate((w,d) for w in ('sx','dx') for d in ('forward','reverse')):
        branch=record['branches'][i];nodes=branch['nodes']
        if (branch['wheel'],branch['direction'])!=(wheel,direction) or len(nodes)!=size:raise ValueError('Branch ordering or grid mismatch')
        params=profile['wheels'][wheel]['branches'][direction];ref=reference['wheels'][wheel];previous=-1.
        for node in nodes:
            v=node['speed_rad_s'];delta=node['loss_difference_Nm']
            if not math.isfinite(v) or v<0 or v<=previous or not math.isfinite(delta):raise ValueError('Invalid LUT node')
            predicted=params['coulomb_Nm']+(params['breakaway_Nm']-params['coulomb_Nm'])*math.exp(-(v/params['stribeck_velocity_rad_s'])**2)-ref['frictionloss_Nm']+(params['damping_Nm_s_rad']-ref['damping_Nm_s_rad'])*v
            if abs(delta-predicted)>1e-14:raise ValueError('Node does not match its frozen source law')
            previous=v
        lines.append('static const StribeckLossNode stribeck_loss_'+str(i)+'[]={'+','.join('{'+format(n['speed_rad_s'],'.17g')+','+format(n['loss_difference_Nm'],'.17g')+'}' for n in nodes)+'};')
    lines+=['static const StribeckLossNode *const stribeck_loss[]={stribeck_loss_0,stribeck_loss_1,stribeck_loss_2,stribeck_loss_3};','#endif']
    a.output.write_text('\n'.join(lines)+'\n');print(sha(a.output))


if __name__=='__main__':main()
