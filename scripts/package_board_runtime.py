#!/usr/bin/env python3
"""Package current board sources/configs without PC data, environments or binaries."""
import argparse
import hashlib
import io
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'deliverables/terasic_board_runtime.tar.gz')
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error('Output exists; choose a new filename')
    files = list((ROOT/'src_cpp').rglob('*.cpp'))+list((ROOT/'src_cpp').rglob('*.h'))
    files += [p for p in (ROOT/'diagnostics').glob('*.cpp') if p.name not in ('digital_twin_ga_bridge.cpp', 'hinf_runtime_tuning_bridge.cpp', 'stribeck_compensation_bridge.cpp')]
    files += list((ROOT/'diagnostics').glob('*.h'))
    files += [p for p in (ROOT/'config/controllers').iterdir() if p.suffix in ('.txt', '.json')]
    files += [ROOT/'scripts'/name for name in (
        'run_pd_logged.sh', 'build_robot.sh', 'inspect_robot_setup.sh', 'check_robot_setup.sh',
        'run_lqr_mimo_logged.sh', 'run_lqr_path_simple_logged.sh',
        'run_hinf_logged.sh', 'run_hinf_path_logged.sh',
        'run_lqr_ga_logged.sh', 'run_lqr_ga_path_logged.sh')]
    records = []
    total = 0
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream, tarfile.open(fileobj=stream, mode='w:gz') as archive:
        for path in sorted(set(files)):
            rel = path.relative_to(ROOT).as_posix()
            data = path.read_bytes()
            records.append(hashlib.sha256(data).hexdigest()+'  '+rel)
            total += len(data)
            info = archive.gettarinfo(str(path), 'terasic_robot_final/'+rel)
            info.uid = info.gid = 0
            info.uname = info.gname = ''
            archive.addfile(info, io.BytesIO(data))
        data = ('\n'.join(records)+'\n').encode()
        info = tarfile.TarInfo('terasic_robot_final/SHA256SUMS')
        info.size = len(data)
        archive.addfile(info, io.BytesIO(data))
    checksum = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_name(output.name+'.sha256').write_text(checksum+'  '+output.name+'\n')
    print(output)
    print(str(len(records))+' files; uncompressed source bytes: '+str(total))
    print('Archive bytes: '+str(output.stat().st_size))
    print('Extract into a new directory; build on ARM. No hardware is started by packaging or compilation.')


if __name__ == '__main__':
    main()
