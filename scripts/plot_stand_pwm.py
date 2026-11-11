#!/usr/bin/env python3
"""Dedicated requested/limited/applied PWM plots from existing stand logs."""
import argparse
import os
from pathlib import Path

from digital_twin import normalized, read_csv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--case', choices=['stand', 'path'], default='stand')
    args = parser.parse_args()
    folder = args.run
    real = list(folder.glob('lqr_mimo_*.csv' if args.case == 'stand' else 'lqr_path_simple_*.csv'))
    if len(real) != 1:
        raise ValueError('Expected exactly one real acquisition')
    traces = [('Robot reale', 'black', normalized(read_csv(real[0]))),
              ('MuJoCo SX/DX', 'tab:blue', normalized(read_csv(folder/'mujoco_sx_dx'/args.case/'simulation.csv')))]
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/terasic-twin-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    windows = ([('pwm_intera_prova', 0, 10), ('pwm_mantenimento', 2, 10)] if args.case == 'stand'
               else [('pwm_percorso', 0, 28.5), ('pwm_svolta', 10.5, 17.5)])
    cap = 20 if args.case == 'stand' else 50
    for filename, start, stop in windows:
        fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True, layout='constrained')
        for row, (stage, label) in enumerate([
                ('raw', 'Richiesta dal controllo'), ('limited', 'Dopo i limiti'),
                ('applied', 'Effettivamente applicata')]):
            for col, wheel in enumerate(['sx', 'dx']):
                ax = axes[row, col]
                for name, color, data in traces:
                    key = f'pwm_{wheel}_{stage}_percent'
                    mask = (data['t_s'] >= start) & (data['t_s'] <= stop)
                    ax.plot(data['t_s'][mask], data[key][mask], color=color, label=name, lw=1)
                ax.axhline(0, color='gray', lw=.5)
                ax.axhline(cap, color='tab:red', lw=.7, ls='--', label=f'Limiti ±{cap}%')
                ax.axhline(-cap, color='tab:red', lw=.7, ls='--')
                ax.set(title=label+' — '+wheel.upper(), ylabel='PWM [%]', xlim=(start, stop))
                if row:
                    ax.set_ylim(-cap-2, cap+2)
                ax.grid(alpha=.25); ax.legend(fontsize=8)
                if row == 2:
                    ax.set_xlabel('Tempo dall’avvio controllo [s]')
        title = 'Fermo LQR' if args.case == 'stand' else 'LQR 40 cm / 180° / 40 cm'
        fig.suptitle(title+' — PWM SX/DX reale e MuJoCo\nK e profilo congelati; grafici dai log acquisiti')
        fig.savefig(folder/(filename+'.svg'))
        fig.savefig(folder/(filename+'.png'), dpi=140)
        plt.close(fig)
    for filename, _, _ in windows:
        print(folder/(filename+'.svg'))


if __name__ == '__main__':
    main()
