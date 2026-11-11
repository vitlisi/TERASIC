#!/usr/bin/env python3
"""Named offline experiments and paired plots. Never connects to real hardware."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]


def name_path(value):
    if not re.fullmatch(r'[A-Za-z0-9_-]+(?:/[A-Za-z0-9_-]+)*', value):
        raise argparse.ArgumentTypeError('Use letters, numbers, _ or -, optionally separated by /')
    return value


def run_id():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+uuid.uuid4().hex[:8]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sim = sub.add_parser('simulate')
    sim.add_argument('--controller', choices=['lqr', 'hinf', 'ga', 'pd', 'pd-outer'], required=True)
    sim.add_argument('--case', choices=['stand', 'path'], default='stand')
    sim.add_argument('--name', type=name_path, help='Optional label included in filenames')
    sim.add_argument('--duration', type=float)
    sim.add_argument('--initial-error', type=float, default=-3.5)
    sim.add_argument('--voltage', type=float, default=12.)
    sim.add_argument('--max-pwm', type=float, default=20.)
    sim.add_argument('--seed', type=int, default=0)
    sim.add_argument('--viewer', action='store_true')
    sim.add_argument('--config', type=Path, help='Controller TXT for real-core LQR/Hinf/GA')
    sim.add_argument('--data-root', type=Path, default=ROOT/'logs')
    sim.add_argument('--figure-root', type=Path, help='Optional alternate plot root; default: data-root')
    compare = sub.add_parser('compare')
    compare.add_argument('--name', type=name_path, required=True)
    compare.add_argument('--simulation', type=Path, required=True)
    compare.add_argument('--real', type=Path, required=True)
    compare.add_argument('--figure-root', type=Path, default=ROOT/'logs')
    args = parser.parse_args()
    identifier = run_id()
    figures = (args.figure_root.resolve()/'real'/args.name/'plot' if args.action == 'compare'
               else (args.figure_root or args.data_root).resolve()/'mujoco'/args.controller/'plot')
    if args.action == 'compare':
        command = [sys.executable, '-B', str(ROOT/'scripts/digital_twin.py'), 'compare',
                   '--simulation', str(args.simulation.resolve()), '--real', str(args.real.resolve()),
                   '--output', str(figures/(identifier+'_real_vs_mujoco.svg'))]
        result = subprocess.run(command, cwd=ROOT)
        print('Comparison figures and metrics: '+str(figures), flush=True)
        return result.returncode
    if args.controller.startswith('pd') and args.case != 'stand':
        parser.error('PD routes currently support stand only')
    if args.controller.startswith('pd') and args.config:
        parser.error('--config applies only to real-core LQR/Hinf/GA')
    duration = args.duration if args.duration is not None else (33. if args.case == 'path' else 10.)
    # Check values before any result folder is created.
    import math
    if not all(math.isfinite(v) for v in (duration, args.initial_error, args.voltage, args.max_pwm)):
        parser.error('Finite parameters required')
    if not 10.5 <= args.voltage <= 12.6 or abs(args.initial_error)>5 or args.seed<0:
        parser.error('Voltage 10.5..12.6, initial error within +/-5 deg and nonnegative seed required')
    cap = 25 if args.controller == 'hinf' else 50 if args.case == 'path' else 100
    if not 0 < args.max_pwm <= cap or not (28.5 <= duration <= 120 if args.case == 'path' else 1 <= duration <= 30):
        parser.error('Invalid duration/PWM limit for the selected route')
    if args.config and not args.config.resolve().is_file():
        parser.error('Controller configuration does not exist')
    folder = args.data_root.resolve()/'.staging'/identifier
    destination = args.data_root.resolve()/'mujoco'/args.controller
    label = (args.name.replace('/', '-')+'_' if args.name else '')
    file_prefix = identifier+'_'+args.case+'_'+label
    folder.parent.mkdir(parents=True, exist_ok=True)
    print('Run ID: '+identifier, flush=True)
    print('Data: '+str(destination), flush=True)
    print('Figures: '+str(figures), flush=True)
    if args.controller.startswith('pd'):
        sys.path.insert(0, str(ROOT/'src_python'))
        from mujoco_sim import parse_args, run
        argv = ['--mode', 'pd', '--duration', str(duration), '--release-pitch-deg', str(args.initial_error),
                '--battery-voltage-v', str(args.voltage), '--max-pwm', str(args.max_pwm),
                '--imu-model', 'mpu6500', '--encoder-model', 'motor', '--sensor-seed', str(args.seed),
                '--tag', args.controller, '--exit-on-stop']
        if not args.viewer:
            argv.append('--headless')
        if args.controller == 'pd':
            argv += ['--kx', '0', '--kv', '0']
        result = run(parse_args(argv), folder)
        records = list(folder.glob('*.csv'))
        for record in records:
            subprocess.run([sys.executable, '-B', str(ROOT/'scripts/plot_mujoco.py'),
                            str(record), '--format', 'both'], cwd=ROOT, check=True)
    else:
        kind = args.case if args.controller != 'hinf' else ('hinf' if args.case == 'stand' else 'hinf-path')
        backend = 'simulate_ga_twin.py' if args.controller == 'ga' else 'digital_twin.py'
        prefix = 'hinf' if args.controller == 'hinf' and args.case == 'stand' else args.case
        command = [sys.executable, '-B', str(ROOT/'scripts'/backend), 'simulate', '--case', kind,
                   '--output-dir', str(folder), '--'+prefix+'-duration', str(duration),
                   '--'+('hinf' if args.controller == 'hinf' else prefix)+'-pwm', str(args.max_pwm), '--initial-error', str(args.initial_error),
                   '--voltage', str(args.voltage), '--seed', str(args.seed)]
        if args.config:
            config_flag = 'hinf-path' if kind == 'hinf-path' else prefix
            command += ['--'+config_flag+'-config', str(args.config.resolve())]
        if args.viewer:
            command.append('--viewer')
        result = subprocess.run(command, cwd=ROOT).returncode
    # Backends retain their private layout while running; publish one flat folder.
    # On an exception staging is retained for diagnosis, never silently removed.
    destination.mkdir(parents=True, exist_ok=True)
    figures.mkdir(parents=True, exist_ok=True)
    exported = []
    mapping = {}
    sources = sorted(p for p in folder.rglob('*') if p.is_file())
    for source in sources:
        relative = source.relative_to(folder)
        suffix = source.name if len(relative.parts) <= 2 else '__'.join(relative.parts)
        if len(relative.parts) == 1 and source.name == 'summary.json':
            suffix = 'backend_summary.json'
        if source.name == 'controller.config.txt':
            suffix = 'simulation.config.txt'
        target = destination/(file_prefix+suffix)
        if target.exists():
            raise FileExistsError(target)
        published = figures/target.name if target.suffix in ('.svg', '.pdf', '.png') else target
        if published.exists() or str(published) in mapping.values():
            raise FileExistsError(published)
        mapping[str(source)] = str(published)
    for source in sources:
        target = Path(mapping[str(source)])
        if source.suffix == '.json':
            content = source.read_text()
            for old, new in sorted(mapping.items(), key=lambda pair: -len(pair[0])):
                content = content.replace(old, new)
            target.write_text(content)
        else:
            shutil.copy2(source, target)
        if target.suffix in ('.svg', '.pdf', '.png'):
            exported.append(str(target))
    manifest = dict(run_id=identifier, controller=args.controller, case=args.case,
        label=args.name, exit_code=result, stribeck_enabled=True,
        friction_compensation='OFF', real_core=not args.controller.startswith('pd'),
        note='PD routes are independent Python controllers, not firmware parity tests.',
        files=[str(Path(v)) for v in mapping.values() if Path(v).suffix not in ('.svg', '.pdf', '.png')],
        figures=exported)
    (destination/(file_prefix+'experiment.json')).write_text(json.dumps(manifest, indent=2)+'\n')
    if folder.exists():
        shutil.rmtree(folder)
        try:
            folder.parent.rmdir()
        except OSError:
            pass  # Another run or an interrupted backend may still use staging.
    print('Published logs: '+str(destination), flush=True)
    print('Exit code: '+str(result)+'; figures exported: '+str(len(exported)), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
