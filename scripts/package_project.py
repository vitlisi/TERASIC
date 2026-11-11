#!/usr/bin/env python3
"""Build a downloadable source/results archive, excluding local environments and credentials."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {'.git', '.aws', '.codex', '.agents', '.ssh', '.vscode', 'env', 'venv',
            '.venv', '__pycache__', '.pytest_cache', '.mypy_cache', '.ruff_cache',
            'build', 'node_modules', '.cache'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'deliverables/terasic_robot_project.tar.gz')
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error('Archive exists; choose a new --output or remove the old archive explicitly')
    output.parent.mkdir(parents=True, exist_ok=True)
    records = []
    files = []
    for folder in ('config', 'models', 'src_python', 'src_cpp', 'scripts', 'diagnostics',
                   'docs', 'tests', 'supplementary', 'maintenance', 'logs', 'deliverables'):
        for p in sorted((ROOT/folder).rglob('*')):
            rel = p.relative_to(ROOT)
            if any(part in EXCLUDED for part in rel.parts) or p.is_symlink() or not p.is_file():
                continue
            if p == output or p.suffix in ('.pyc', '.pyo', '.pem', '.key') or p.name.startswith('.env'):
                continue
            if rel.parts[0] == 'deliverables' and p.name.endswith(('.tar.gz', '.zip', '.tar.gz.sha256', '.zip.sha256')):
                continue
            files.append(p)
    files += [ROOT/'README.md', ROOT/'requirements.txt', ROOT/'.gitignore']
    # Exclusive output creation also prevents accidental overwrite during a race.
    with output.open('xb') as stream, tarfile.open(fileobj=stream, mode='w:gz') as archive:
        for p in files:
            rel = p.relative_to(ROOT).as_posix()
            payload = p.read_bytes()
            records.append(dict(path=rel, bytes=len(payload), sha256=hashlib.sha256(payload).hexdigest()))
            info = archive.gettarinfo(str(p), arcname='terasic_robot/'+rel)
            info.uid = info.gid = 0
            info.uname = info.gname = ''
            archive.addfile(info, io.BytesIO(payload))
        manifest = json.dumps(dict(files=records, excluded=sorted(EXCLUDED),
            note='Create a fresh PC Python environment. No private host keys or per-board credentials are supplied.'),
            indent=2).encode()+b'\n'
        info = tarfile.TarInfo('terasic_robot/PROJECT_MANIFEST.json')
        info.size = len(manifest)
        archive.addfile(info, io.BytesIO(manifest))
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_name(output.name+'.sha256').write_text(digest+'  '+output.name+'\n')
    print(str(output))
    print(str(len(records))+' files; '+str(output.stat().st_size)+' bytes; SHA256 '+digest)


if __name__ == '__main__':
    main()
