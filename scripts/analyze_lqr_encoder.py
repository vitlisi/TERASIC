#!/usr/bin/env python3
"""Check a manually annotated, motor-off wheel revolution; never writes gains."""
import argparse
import csv
import json
import math
from pathlib import Path


def analyze(path, wheel, turns, direction, legacy_fpga_names=False):
    if not math.isfinite(turns) or turns <= 0:
        raise ValueError('Numero di giri positivo e finito richiesto')
    with Path(path).open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 100:
        raise ValueError('Servono almeno 100 campioni e una prova terminata normalmente')
    for row in rows:
        if int(row['armed']) != 0 or float(row['pwm_applied_percent']) != 0:
            raise ValueError('La calibrazione deve essere svolta con motori spenti')
    counts = {side: [int(row[f'enc_{side}_raw']) for row in rows] for side in ('sx', 'dx')}
    if any(value <= 0 or value >= 65535 for series in counts.values() for value in series):
        raise ValueError('Contatore saturo: ripetere con meno giri')
    # Check stationary windows at both ends; physical rotation count is supplied
    # by the operator, never inferred from these encoder values.
    stable = all(max(window)-min(window) <= 2 for series in counts.values()
                 for window in (series[:30], series[-30:]))
    delta = {side: series[-1]-series[0] for side, series in counts.items()}
    other = 'dx' if wheel == 'sx' else 'sx'
    errors = []
    if not stable:
        errors.append('Ruote non ferme nei primi/ultimi 0.3 s: attendere prima e dopo il giro')
    if abs(delta[wheel]) < 20:
        errors.append('Movimento troppo piccolo per una misura affidabile')
    if abs(delta[other]) > max(4, .02*abs(delta[wheel])):
        errors.append('Anche l altra ruota si e mossa: ripetere muovendo una sola ruota')
    expected = ((1 if wheel == 'sx' else -1) if legacy_fpga_names else (-1 if wheel == 'sx' else 1)) * (1 if direction == 'forward' else -1)
    if delta[wheel]*expected <= 0:
        errors.append('Segno diverso dal montaggio documentato: verificare ruota e direzione')
    return dict(csv=str(path), wheel=wheel, turns_declared=turns, direction=direction,
                samples=len(rows), delta_counts=delta, checks_passed=not errors,
                counts_per_rev_candidate=abs(delta[wheel])/turns if not errors else None,
                errors=errors,
                note='Giri dichiarati manualmente. Confrontare SX/DX e i due versi prima di configurare LQR.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--wheel', choices=['sx', 'dx'], required=True)
    parser.add_argument('--turns', type=float, required=True)
    parser.add_argument('--direction', choices=['forward', 'backward'], required=True)
    parser.add_argument('--legacy-fpga-names', action='store_true', help='Vecchi log prima della mappa fisica v2')
    args = parser.parse_args()
    try:
        result = analyze(args.csv, args.wheel, args.turns, args.direction, args.legacy_fpga_names)
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f'Calibrazione non valida: {error}\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['checks_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
