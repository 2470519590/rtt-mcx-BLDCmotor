"""Cross-correlation between the angle-periodic voltage residual and phase.

For each speed, find the lag that maximises the fit. A lag that grows in
magnitude and flips sign with speed is the signature of a time delay
(causality), not of a calibration-zero rotation, which would be one constant
offset for every speed.
"""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from angle_phase_model import HARMONICS  # noqa: E402

source = Path(__file__).resolve().parents[1] / 'data/angle_phase_refit_20261021.csv'
rows = [r for r in csv.DictReader(source.open(encoding='utf-8')) if r['config'] == 'pi_kp150_ki100']
SPEEDS = (15, -15, 30, -30, 20, -20)


def steady_rows(repeat, target):
    selected = [r for r in rows if int(r['repeat']) == repeat and int(r['target_rpm']) == target]
    first = int(selected[0]['tick_ms'])
    return [r for r in selected if int(r['tick_ms']) - first >= 2000]


def residual(samples, target):
    position = np.array([int(r['position_mrad']) for r in samples]) / 1000
    total = np.array([int(r['uq_mv']) for r in samples]) / 1000
    model = target * 2 * np.pi / 60 / 44.18
    label = total - model
    label = label - label.mean()
    return position, label


def phase_from_harmonic(position, label):
    design = np.column_stack([f(h * position) for h in HARMONICS for f in (np.sin, np.cos)])
    coefficients, _, _, _ = np.linalg.lstsq(design, label, rcond=None)
    explained = 1 - np.mean((label - design @ coefficients) ** 2) / max(np.mean(label ** 2), 1e-12)
    return float(np.arctan2(coefficients[1], coefficients[0])), float(explained)


print(f"{'RPM':>5} {'repeat':>7} {'phase_rad':>10} {'phase_deg':>10} {'harmonics':>10} "
      f"{'lag_deg':>9} {'lag_ms':>8}")
summary = {}
for target in SPEEDS:
    lags = []
    for repeat in (0, 1, 2):
        position, label = residual(steady_rows(repeat, target), target)
        phase, explained = phase_from_harmonic(position, label)
        period_ms = 60 / abs(target) * 1000  # one mechanical revolution
        lag_ms = phase / (2 * np.pi) * period_ms
        print(f'{target:>5} {repeat:>7} {phase:>10.3f} {np.degrees(phase):>10.1f} '
              f'{explained:>10.3f} {np.degrees(phase):>9.1f} {lag_ms:>8.1f}')
        lags.append(phase)
    summary[target] = float(np.mean(lags))

print('\nmean fundamental phase per speed (rad):')
for target, value in summary.items():
    print(f'  {target:+4d} RPM: {value:+.3f} rad ({np.degrees(value):+.1f} deg), '
          f'lag {value / (2 * np.pi) * 60 / abs(target) * 1000:+.1f} ms')
# A fixed delay tau predicts phase = -sign(rpm)*2*pi*|rpm|/60*tau ... in degrees
# of mechanical angle per revolution, so the implied delay is lag_ms above.
delays = {t: v / (2 * np.pi) * 60 / abs(t) * 1000 for t, v in summary.items()}
print('\nimplied delay per speed (ms):', {t: round(d, 2) for t, d in delays.items()})
