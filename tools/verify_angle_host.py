"""Run actual host-compiled C, not a board test, using conda py310 as driver."""
import argparse
import json
from pathlib import Path
import subprocess

import numpy as np

from angle_shadow_model import load_angle_model, predict_float32


def wsl_path(path):
    path = Path(path).resolve()
    return '/mnt/'+path.drive[0].lower()+str(path)[2:].replace('\\', '/')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='data/angle_residual_candidate_v2.npz')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    binary = root/'debug'/'angle_shadow_host'
    subprocess.run(['wsl.exe', '--exec', 'gcc', '-std=c99', '-O2', '-ffp-contract=off',
                    '-I'+wsl_path(root/'app'), wsl_path(root/'app/angle_shadow.c'),
                    wsl_path(root/'app/nn_controller.c'), wsl_path(root/'tools/angle_shadow_harness.c'),
                    '-lm', '-o', wsl_path(binary)], check=True, capture_output=True)
    rng = np.random.default_rng(47)
    position = np.r_[rng.uniform(-100, 100, 3000), [0, 2*np.pi, -2*np.pi]].astype(np.float32)
    speed = np.r_[rng.choice([-30, -15, 0, 15, 30], 3000)*2*np.pi/60, [0, 15*2*np.pi/60, -15*2*np.pi/60]].astype(np.float32)
    baseline = rng.uniform(-1, 1, len(position)).astype(np.float32)
    lines = [f'5 {p:.9g} {s:.9g} {b:.9g} 1000' for p, s, b in zip(position, speed, baseline)]
    completed = subprocess.run(['wsl.exe', '--exec', wsl_path(binary)],
                               input='\n'.join(lines)+'\n', capture_output=True, text=True, check=True)
    actual = np.array([list(map(float, line.split())) for line in completed.stdout.splitlines()])
    if actual.shape != (len(position), 2):
        raise ValueError('unexpected host harness output')
    expected = predict_float32(position, speed, load_angle_model(args.model))
    prediction_error = float(np.max(np.abs(actual[:, 0]-expected)))
    output_error = float(np.max(np.abs(actual[:, 1]-baseline)))
    blend_lines = [f'6 {p:.9g} {s:.9g} {b:.9g} 250' for p, s, b in zip(position, speed, baseline)]
    blended = subprocess.run(['wsl.exe', '--exec', wsl_path(binary)],
                             input='\n'.join(blend_lines)+'\n', capture_output=True, text=True, check=True)
    blend_values = np.array([list(map(float, line.split())) for line in blended.stdout.splitlines()])
    wanted = baseline + .25*np.clip(expected, -.08, .08)
    blend_error = float(np.max(np.abs(blend_values[:, 1]-wanted)))
    max_applied = float(np.max(np.abs(blend_values[:, 1]-baseline)))
    passed = prediction_error < .00002 and output_error < 1e-7 and blend_error < .00002 and max_applied < .020001
    report = dict(samples=len(position), max_c_python_prediction_error_v=prediction_error,
                  max_shadow_output_change_v=output_error, passed=passed,
                  max_blended_application_error_v=blend_error, max_applied_angle_voltage_v=max_applied,
                  limitation='Actual C compiled and executed on host via WSL GCC; not Cortex-M execution, '
                             'not MCU timing, not hardware closed-loop validation.')
    (root/'data'/'angle_shadow_host_parity.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit('angle host parity failed')


if __name__ == '__main__':
    main()
