"""Validated float32 contract for the non-actuating angle candidate."""
import hashlib
from pathlib import Path
import numpy as np

HARMONICS = np.array([1, 2, 7, 14, 21], dtype=np.float32)


def load_angle_model(path):
    with np.load(path, allow_pickle=False) as archive:
        if str(archive['schema']) != 'angle_voltage_candidate_v1':
            raise ValueError('incompatible angle model schema')
        model = {k: np.array(archive[k], dtype=np.float32) for k in
                 ('input_mean', 'input_std', 'hidden_weights', 'hidden_bias',
                  'output_weights', 'output_bias')}
        shapes = dict(input_mean=(12,), input_std=(12,), hidden_weights=(8, 12),
                      hidden_bias=(8,), output_weights=(8,), output_bias=())
        if any(v.shape != shapes[k] or not np.all(np.isfinite(v)) for k, v in model.items()):
            raise ValueError('invalid angle model dimensions or nonfinite coefficients')
        if np.any(model['input_std'] <= 0) or not np.array_equal(archive['harmonics'], HARMONICS):
            raise ValueError('invalid normalization or harmonics')
    return model


def predict_float32(position_rad, target_rad_s, model):
    angle = np.asarray(position_rad, dtype=np.float32)
    speed = np.broadcast_to(np.asarray(target_rad_s, dtype=np.float32), angle.shape)
    rpm = speed * np.float32(60) / np.float32(6.28318530718)
    x = np.column_stack([rpm/np.float32(40), np.sign(rpm)] +
                        [f(h*angle) for h in HARMONICS for f in (np.sin, np.cos)])
    x = (x-model['input_mean'])/model['input_std']
    # Match scalar MCU accumulation order, not a different BLAS reduction.
    output = np.full(len(x), model['output_bias'], dtype=np.float32)
    for i in range(8):
        hidden = np.full(len(x), model['hidden_bias'][i], dtype=np.float32)
        for j in range(12):
            hidden += model['hidden_weights'][i, j]*x[:, j]
        output += model['output_weights'][i]*np.maximum(hidden, np.float32(0))
    return output


def render_header(path):
    from export_nn_weights import array_1d, array_2d, c_float
    model = load_angle_model(path)
    digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    return f'''#ifndef ANGLE_SHADOW_MODEL_H
#define ANGLE_SHADOW_MODEL_H
/* Generated shadow-only model; source SHA256 {digest}.
   Calibration-relative phase; this file is NOT permission to apply output. */
#define ANGLE_MODEL_INPUTS 12U
#define ANGLE_MODEL_HIDDEN 8U
static const float angle_input_mean[12] = {{{array_1d(model["input_mean"])}}};
static const float angle_input_std[12] = {{{array_1d(model["input_std"])}}};
static const float angle_hidden_weights[8][12] = {{
{array_2d(model["hidden_weights"])}
}};
static const float angle_hidden_bias[8] = {{{array_1d(model["hidden_bias"])}}};
static const float angle_output_weights[8] = {{{array_1d(model["output_weights"])}}};
static const float angle_output_bias = {c_float(model["output_bias"])};
#endif
'''


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model')
    parser.add_argument('--output', default='app/angle_shadow_model.h')
    args = parser.parse_args()
    target = Path(args.output)
    if target.exists():
        raise FileExistsError('model header exists; select an explicit new export path')
    target.write_text(render_header(args.model), encoding='utf-8')
    print(f'Exported shadow-only model: {target.resolve()}')
