#!/usr/bin/env python3
"""Summarize all held-out GA validation scenarios without selecting examples."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-ga-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ga-run', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    summary = json.loads((args.ga_run / 'summary.json').read_text())
    if summary['status'] != 'complete':
        parser.error('The GA search and validation must be complete')
    data = json.loads((args.ga_run / 'validation.json').read_text())
    config = json.loads((args.ga_run / 'config.json').read_text())
    names = [t['scenario'] for t in data['baseline']['trials']]
    if names != [t['scenario'] for t in data['ga']['trials']]:
        parser.error('Validation scenario order differs')
    x = np.arange(len(names))
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), layout='constrained')
    labels = {'baseline': 'Baseline LQR', 'ga': 'GA-LQR'}
    colors = {'baseline': 'tab:orange', 'ga': 'tab:blue'}
    fields = [('cost', 'Scenario cost', 1), ('pitch_rms_deg', 'Full-trace pitch RMS [deg]', 1),
              ('position_rms_m', 'Full-trace planar position RMS [mm]', 1000),
              ('pwm_rms_percent', 'Applied PWM RMS [%]', 1),
              ('saturation_fraction', 'Samples at PWM cap [%]', 100)]
    aggregate = {}
    for offset, name in ((-.18, 'baseline'), (.18, 'ga')):
        trials = data[name]['trials']
        complete = sum(bool(t.get('survived')) for t in trials)
        settled = sum(bool(t.get('stabilized')) for t in trials)
        bars = axes[0, 0].bar(np.arange(2) + offset, [complete, settled], width=.34,
                              label=labels[name], color=colors[name])
        axes[0, 0].bar_label(bars, padding=3)
        aggregate[name] = dict(completed=complete, settled=settled,
                               rank=data[name]['rank'], means={}, metric_counts={})
        for ax, (field, title, scale) in zip(list(axes.flat)[1:], fields):
            values = np.array([t.get(field, np.nan) for t in trials], dtype=float)
            ax.plot(x, values*scale, 'o-', color=colors[name], label=labels[name], lw=1)
            ax.set(ylabel=title, xticks=x, xticklabels=[n.rsplit('_', 1)[-1] for n in names],
                   xlabel='Held-out scenario index')
            valid = np.isfinite(values)
            aggregate[name]['means'][field] = float(np.mean(values[valid])) if valid.any() else None
            aggregate[name]['metric_counts'][field] = int(valid.sum())
    axes[0, 0].set(xticks=[0, 1], xticklabels=['Completed duration', 'Met all settling criteria'],
                   ylabel='Scenario count', ylim=(0, len(names)+1))
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.legend(loc='best', fontsize=8)
    protocol = config['protocol']
    fig.suptitle(f'MuJoCo — LQR versus GA-LQR, all {len(names)} held-out validation scenarios\n'
                 f'Current Stribeck model; {protocol["duration_s"]:g} s trials; '
                 f'{protocol["max_pwm"]:g}% PWM cap; same scenario and sensor seed per pair')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for ext in ('svg', 'pdf', 'png'):
        path = args.output_dir / ('all_validation_scenarios.' + ext)
        fig.savefig(path, dpi=180)
        print(path)
    plt.close(fig)
    (args.output_dir / 'all_validation_metrics.json').write_text(json.dumps(dict(
        source_run=str(args.ga_run.resolve()), scenario_count=len(names),
        validation_improved=summary['validation_improved'], aggregate=aggregate), indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
