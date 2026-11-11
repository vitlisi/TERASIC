#!/usr/bin/env python3
"""Plot GA training progress; final validation is reported separately."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-ga-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ga-run', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads((args.ga_run / 'config.json').read_text())
    history = json.loads((args.ga_run / 'history.json').read_text())
    if not history:
        parser.error('No evaluated individuals')
    generations = sorted({r['generation'] for r in history})
    best = [min((r for r in history if r['generation'] <= g),
                key=lambda r: tuple(r['rank'])) for g in generations]
    baseline = next(r for r in history if not any(r['genes']))['rank']
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout='constrained')
    x = [g + 1 for g in generations]
    for ax, component, label in ((axes[0], 0, 'Scenarios failing the settling criteria'),
                                  (axes[1], 2, 'Training cost (mean + 0.5 × worst)')):
        ax.plot(x, [r['rank'][component] for r in best], 'o-', label='Best selected so far')
        ax.axhline(baseline[component], color='tab:orange', linestyle='--', label='Baseline LQR')
        ax.set(xlabel='Completed/evaluated generation', ylabel=label, xticks=x)
        ax.grid(alpha=.25)
        ax.legend()
    axes[0].set_ylim(-.2, config['train_scenarios'] + .2)
    fig.suptitle('GA training progress — fixed training bank\n'
                 'Selection prioritizes failed scenarios, then missing time, then cost; validation is held out')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for ext in ('svg', 'pdf', 'png'):
        path = args.output_dir / ('ga_training_progress.' + ext)
        fig.savefig(path, dpi=180)
        print(path)
    plt.close(fig)


if __name__ == '__main__':
    main()
