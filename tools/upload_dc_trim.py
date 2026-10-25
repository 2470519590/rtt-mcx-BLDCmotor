"""Upload only a per-speed DC voltage table (no angle dependence).

The DC/AC split on the NN-off baseline shows the constant part is worth about
60% of the residual while the angle-periodic part barely helps. This tool tests
exactly that hypothesis with the smallest possible intervention: upload a
network whose 12 input weights are zero, so the network output is a constant
bias, and steer that bias through the firmware's existing per-speed phase
points (which select the output bias).

Nothing here claims the DC is a physical disturbance: it is the recorded
voltage offset of the closed loop, and the file is written with that caveat.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from angle_phase_model import PhaseTable, load_model, upload_sequence
from collect_nn_dataset import read_state
from upload_angle_model import AngleUploader, _mark_settled, collect_shadow, shadow_report


def dc_table(split_path, speed_map, alpha_milli):
    """One phase point per speed that encodes the DC voltage to apply."""
    split = json.loads(Path(split_path).read_text(encoding='utf-8'))
    dc = split['dc_by_speed']
    alpha = alpha_milli / 1000.0
    points = []
    for key, rpm in speed_map.items():
        if key not in dc:
            raise ValueError(f'no DC entry for speed {key}')
        voltage = dc[key]
        # The firmware clamps the applied prediction to NN_ANGLE_DELTA_MAX
        # (0.20 V) and the alpha to NN_ANGLE_ALPHA_MAX, so a DC that would be
        # silently truncated is rejected here instead.
        needed_alpha = abs(voltage) / .20
        if needed_alpha > alpha:
            raise ValueError(f'DC {voltage:+.3f} V at {key} RPM needs alpha >= {needed_alpha:.3f}; '
                             f'raise --alpha or the clamp will silently truncate it')
        points.append((rpm, voltage / alpha))
    return PhaseTable(points, interp=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--split', required=True, help='split_angle_dc_ac JSON')
    parser.add_argument('--output-table', required=True)
    parser.add_argument('--alpha', type=int, default=250)
    parser.add_argument('--shadow', nargs='*', type=int, help='speeds for a G5 check')
    args = parser.parse_args()
    table = dc_table(args.split, {'-30': -30.0, '-20': -20.0, '-15': -15.0,
                                  '15': 15.0, '20': 20.0, '30': 30.0}, args.alpha)
    model = load_model(args.model)
    flat = {k: (v.copy() if hasattr(v, 'copy') else v) for k, v in model.items()}
    flat['hidden_weights'] = np.zeros_like(model['hidden_weights'])
    Path(args.output_table).write_text(
        json.dumps(dict(source=args.split, alpha_milli=args.alpha,
                        schema='angle_dc_upload_v1',
                        table=table.as_dict(),
                        limitation='Uploaded network has zero input weights: output is a per-speed '
                                   'constant voltage. This is a DC trim of the recorded closed-loop '
                                   'voltage, not a measured disturbance torque.'),
                   indent=2), encoding='utf-8')
    with AngleUploader() as uploader:
        print('health', ','.join(uploader.health()), flush=True)
        for op, value in upload_sequence(flat, table):
            uploader.command(op, value)
        print('uploaded', uploader.query(), flush=True)
        if args.shadow:
            rows = collect_shadow(uploader, args.shadow)
            _mark_settled(rows)
            print('shadow', shadow_report(rows, flat, table), flush=True)
    print('table', args.output_table)


if __name__ == '__main__':
    main()
