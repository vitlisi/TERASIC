"""One qualified raised-wheel acquisition. Never retries motor commands."""
import argparse,datetime,hashlib,io,json,re,shlex,subprocess,tarfile,uuid
from pathlib import Path
from trial_remote_session import session_setup
ROOT=Path(__file__).resolve().parents[1]
QUAL=ROOT/'config/qualification/freewheel_compensated_arm.json'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('host');p.add_argument('folder',type=Path);p.add_argument('mode',choices=['shadow','run'])
    p.add_argument('--wheels-lifted',action='store_true',required=True);p.add_argument('--chassis-fixed',action='store_true',required=True)
    a=p.parse_args();q=shlex.quote
    if not re.fullmatch(r'[a-zA-Z0-9.-]+',a.host):p.error('Invalid host')
    qual=json.loads(QUAL.read_text())
    assert qual['tests_passed'] and qual['kind']=='robot01_compensated_freewheel_v1'
    for path,digest in qual['source_sha256'].items():
        if sha(ROOT/path)!=digest:raise RuntimeError('Qualified source changed: '+path)
    a.folder.mkdir(parents=True,exist_ok=False)
    ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=15','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=3','-o','HostKeyAlias=192.168.50.2','root@'+a.host]
    binary=qual['remote_work']+'/build/robot/'+qual['binary_name']
    state=dict(mode=a.mode,host=a.host,status='preflight',qualification_sha256=sha(QUAL),hardware_command_count=0,started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    def save(): (a.folder/'state.json').write_text(json.dumps(state,indent=2)+'\n')
    save()
    pre=subprocess.run(ssh+['hostname; uname -m; /home/root/terasic_robot/build/robot/fpga_readonly; /home/root/terasic_robot/build/robot/battery_status; sha256sum '+q(binary)],capture_output=True,text=True,timeout=45)
    output=pre.stdout+'\n'+pre.stderr;(a.folder/'preflight.txt').write_text(output)
    if pre.returncode or not re.search(r'^socfpga$',output,re.M) or not re.search(r'^armv7l$',output,re.M) or qual['binary_sha256'] not in output:raise RuntimeError('Identity/binary preflight failed')
    for w in ('SX','DX'):
        m=re.search(r'Motore '+w+r':\s+periodo=(\d+) alto=(\d+) controllo=0x([0-9a-fA-F]+)',output)
        if not m or int(m[2]) or int(m[3],16)&1:raise RuntimeError('Motor OFF not verified')
    v=re.search(r'Tensione pacco .*?: ([0-9.]+) V',output)
    if not v or not 11.2<=float(v[1])<=12.1:raise RuntimeError('Voltage outside qualified band')
    sid=uuid.uuid4().hex;remote=qual['remote_work']+'/raised/'+sid
    state.update(status='started_or_unknown_do_not_retry',session_id=sid,remote_folder=remote,off_voltage_v=float(v[1]),hardware_command_count=1);save()
    cmd=('set -e; '+session_setup(remote,sid)+'test "$(sha256sum '+q(binary)+' | cut -d " " -f 1)" = '+q(qual['binary_sha256'])+'; set +e; '+q(binary)+' '+a.mode+' --wheels-lifted --chassis-fixed '+q(remote+'/reale.csv')+' > '+q(remote+'/stdout.txt')+' 2> '+q(remote+'/reale.status.log')+'; rc=$?; echo "$rc" > '+q(remote+'/returncode.txt')+'; /home/root/terasic_robot/build/robot/fpga_readonly > '+q(remote+'/final_registers.txt')+'; cat '+q(remote+'/reale.status.log')+'; cat '+q(remote+'/final_registers.txt')+'; exit "$rc"')
    try:result=subprocess.run(ssh+[cmd],capture_output=True,text=True,timeout=90)
    except subprocess.TimeoutExpired:save();raise RuntimeError('Unknown remote state; inspect, never retry motors')
    (a.folder/'transport.txt').write_text(result.stdout+'\n'+result.stderr)
    fetched=subprocess.run(ssh+['tar -C '+q(remote)+' -cf - .'],capture_output=True,timeout=45)
    if fetched.returncode:raise RuntimeError('Read-only fetch failed; do not retry motors')
    (a.folder/'remote.tar').write_bytes(fetched.stdout);raw=a.folder/'raw';raw.mkdir()
    with tarfile.open(fileobj=io.BytesIO(fetched.stdout)) as archive:
        for item in archive.getmembers():
            if Path(item.name).is_absolute() or '..' in Path(item.name).parts or not(item.isfile() or item.isdir()):raise RuntimeError('Unsafe archive')
            dest=raw/item.name
            if item.isdir():dest.mkdir(parents=True,exist_ok=True)
            else:
                dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(archive.extractfile(item).read())
    if (raw/'session_id.txt').read_text().strip()!=sid:raise RuntimeError('Session mismatch')
    state['status']='captured_pending_analysis' if result.returncode==0 else 'aborted_preserved';save()
    print(result.stdout,flush=True)
    from analyze_compensated_freewheel import analyze
    report=analyze(a.folder,a.mode);(a.folder/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    state['status']='verified' if report['accepted'] else 'flagged_preserved';save();print(json.dumps(report,indent=2))
    if not report['accepted']:raise SystemExit(1)
if __name__=='__main__':main()
