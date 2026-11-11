"""Execute ONE qualified guarded baseline floor trial; no automatic motor retry or plotting."""
import argparse
import datetime
import hashlib
import io
import json
from pathlib import Path
import re
import shlex
import subprocess
import tarfile
from analyze_guarded_balance import analyze
import math
import uuid
from trial_remote_session import session_setup

WORK='/home/root/terasic_robot/experiments/robot01_lut_prof/protocol/v014_guarded_balance_20261106_01'
QUALIFICATION=Path(__file__).resolve().parents[1]/'config/qualification/lqr_guarded_arm.json'


def valid_trial_name(name):return bool(re.fullmatch(r'[a-zA-Z0-9_.-]+',name)) and name not in ['.','..']


def utc(): return datetime.datetime.now(datetime.timezone.utc).isoformat()


def verify_session(raw, expected):
    """A failed launch must never make old remote data look like a new trial."""
    marker=raw/'session_id.txt'
    if not marker.is_file() or marker.read_text().strip()!=expected:
        raise RuntimeError('Remote session identity mismatch; preserve files, do not analyze or retry motors')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('host');p.add_argument('folder',type=Path)
    p.add_argument('mode',choices=['shadow','run'])
    p.add_argument('--floor-ready',action='store_true',required=True)
    p.add_argument('--experiment-qualification',type=Path,
                   help='Explicit isolated Stribeck experiment qualification; default remains baseline')
    a=p.parse_args()
    if not re.fullmatch(r'[a-zA-Z0-9.-]+',a.host):p.error('Invalid host')
    qualification_path=a.experiment_qualification or QUALIFICATION
    if not qualification_path.exists():p.error('ARM qualification missing')
    qualification=json.loads(qualification_path.read_text())
    work=WORK;binary='balance_lqr_guarded'
    if a.experiment_qualification:
        expected=Path(__file__).resolve().parents[1]/'config/qualification/lqr_stribeck_arm.json'
        if qualification_path.resolve()!=expected.resolve() or qualification.get('kind')!='stribeck_compensation_v1':p.error('Unsupported experimental qualification')
        work=qualification['remote_work'];binary='balance_lqr_stribeck'
        if qualification.get('binary_name')!=binary:p.error('Experimental binary mismatch')
    if not qualification['tests_passed'] or (not a.experiment_qualification and qualification['remote_work']!=WORK):p.error('Qualification mismatch')
    binary_sha=qualification['binary_sha256']
    if a.folder.exists():p.error('New local trial folder required; investigate existing trials before retry')
    a.folder.mkdir(parents=True)
    ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=30','-o','ServerAliveInterval=15',
         '-o','ServerAliveCountMax=3','-o','ControlMaster=auto','-o','ControlPersist=300',
         '-o','ControlPath=/tmp/robot01-hyst-%C','-o','HostKeyAlias=192.168.50.2','root@'+a.host]
    state=dict(started_utc=utc(),host=a.host,
               binary_sha256=binary_sha,remote_work=work,cap_percent=20,live_min_v=11.20,mode=a.mode,duration_s=10 if a.mode=="run" else 3,config_sha256=qualification["config_sha256"],compensation=qualification.get('compensation','OFF'),
               status='preflight',hardware_command_count=0)
    def save(): (a.folder/'state.json').write_text(json.dumps(state,indent=2)+'\n')
    save()
    pre=subprocess.run(ssh+['hostname; cd /home/root/terasic_robot && build/robot/fpga_readonly && build/robot/battery_status; sha256sum '+work+'/build/robot/'+binary+' '+work+'/config/controllers/lqr_mimo.txt'],capture_output=True,text=True,timeout=45)
    output=pre.stdout+'\n'+pre.stderr;(a.folder/'preflight.txt').write_text(output)
    if pre.returncode or not re.search(r'^socfpga$',output,re.M) or binary_sha not in output or qualification['config_sha256'] not in output:
        raise RuntimeError('Identity, connection or qualified binary check failed')
    for label in ['SX','DX']:
        m=re.search(r'Motore '+label+r':\s+periodo=(\d+) alto=(\d+) controllo=0x([0-9a-fA-F]+)',output)
        if not m or int(m[2]) or int(m[3],16)&1:raise RuntimeError('Motor OFF not verified')
    v=re.search(r'Tensione pacco .*?: ([0-9.]+) V',output)
    if not v or not 11.20<=float(v[1])<=12.10:raise RuntimeError('OFF voltage outside 11.20..12.10 V')
    state['off_voltage_v']=float(v[1])
    name=a.folder.resolve().name
    if not valid_trial_name(name):raise ValueError('Invalid trial name')
    session_id=uuid.uuid4().hex
    remote=work+'/floor/'+name+'_'+session_id
    state.update(remote_folder=remote,session_id=session_id)
    state.update(status='started_or_unknown',hardware_command_count=1);save()
    q=shlex.quote
    command=('set -e; '+session_setup(remote,session_id)+'cd '+q(work)+'; '
        'test "$(sha256sum build/robot/'+binary+' | cut -d " " -f 1)" = '+q(binary_sha)+'; '
        'set +e; ./build/robot/'+binary+' '+a.mode+' config/controllers/lqr_mimo.txt '+str(10 if a.mode=='run' else 3)+' 20 > '+q(remote+'/reale.csv')+' 2> '+q(remote+'/reale.status.log')+' & '
        'pid=$!; echo "$pid" > '+q(remote+'/active.pid')+'; wait "$pid"; rc=$?; echo "$rc" > '+q(remote+'/returncode.txt')+'; '
        'rm -f '+q(remote+'/active.pid')+'; /home/root/terasic_robot/build/robot/fpga_readonly > '+q(remote+'/final_registers.txt')+' 2>&1; '
        'cat '+q(remote+'/reale.status.log')+'; cat '+q(remote+'/final_registers.txt')+'; exit "$rc"')
    try:
        result=subprocess.run(ssh+[command],capture_output=True,text=True,timeout=160)
        (a.folder/'transport.txt').write_text(result.stdout+'\n'+result.stderr)
        state['transport_returncode']=result.returncode
    except subprocess.TimeoutExpired as e:
        state['status']='transport_timeout_unknown_do_not_retry';save();raise RuntimeError('Unknown trial state; inspect remote files, never rerun blindly') from e
    save()
    for attempt in range(3):
        fetched=subprocess.run(ssh+['tar -C '+q(remote)+' -cf - .'],capture_output=True,timeout=45)
        if not fetched.returncode:break
    if fetched.returncode:raise RuntimeError('Read-only fetch failed; no motor retry')
    archive_path=a.folder/'remote.tar';archive_path.write_bytes(fetched.stdout)
    state['remote_tar_sha256']=hashlib.sha256(fetched.stdout).hexdigest()
    raw=a.folder/'raw';raw.mkdir()
    with tarfile.open(fileobj=io.BytesIO(fetched.stdout)) as archive:
        for item in archive.getmembers():
            if Path(item.name).is_absolute() or '..' in Path(item.name).parts or not (item.isfile() or item.isdir()):
                raise ValueError('Unsafe remote archive')
            dest=raw/item.name
            if item.isdir():dest.mkdir(exist_ok=True,parents=True)
            else:
                dest.parent.mkdir(exist_ok=True,parents=True)
                with archive.extractfile(item) as stream:dest.write_bytes(stream.read())
    try:verify_session(raw,session_id)
    except RuntimeError:
        state.update(status='invalid_remote_session_preserved',finished_utc=utc());save();raise
    status=(raw/'reale.status.log').read_text()
    if (raw/'returncode.txt').read_text().strip()!='0' or 'LQR terminato; RUN SX=0 DX=0.' not in status:
        state.update(status='aborted_preserved',finished_utc=utc());save()
        print(status,flush=True);raise SystemExit(1)
    if a.experiment_qualification:
        from analyze_stribeck_trial import analyze as analyze_experiment
        report=analyze_experiment(a.folder,a.mode)
    else:
        report=analyze(a.folder,a.mode)
    (a.folder/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    state['status']='measured' if report['logging_complete'] else 'measured_flagged'
    state['finished_utc']=utc();save()
    print(json.dumps(dict(status=state['status'],off_voltage_v=state['off_voltage_v'],
        accepted=report['logging_complete'],samples=report['samples'],flags=report['flags'],output=str(a.folder)),indent=2),flush=True)
    # Quality flags remain data. Supervisor decides whether an extended hold is justified.


if __name__=='__main__':main()
