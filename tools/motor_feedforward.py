"""Shared firmware ff1 contract: fixed knots, 0.1 mV coefficients, +/-0.3 V."""
import json
import hashlib
from pathlib import Path

import numpy as np

KNOTS = np.array([-1, -2/3, -1/3, -.05, .05, 1/3, 2/3, 1.])


def model_identity(path):
    source = Path(path).resolve()
    return dict(ff_model=str(source), ff_model_sha256=hashlib.sha256(source.read_bytes()).hexdigest())


def basis(rpm):
    x = np.clip(np.asarray(rpm, dtype=float)/180, -1, 1)
    return np.column_stack([np.maximum(x[:, None]-KNOTS, 0), np.ones(len(x))])


def load_weights(path):
    model = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(model, dict):
        raise ValueError('feedforward model must be a JSON object')
    weights = model.get('upload_weights')
    knots = model.get('knots')
    if (model.get('schema') != 'relu_speed_ff1' or
            model.get('weight_units_v') != .0001 or
            not isinstance(weights, list) or len(weights) != 9 or
            any(type(w) is not int or abs(w) > 100000 for w in weights) or
            not isinstance(knots, list) or len(knots) != 8):
        raise ValueError('incompatible feedforward model')
    try:
        matches = np.allclose(np.array(knots, dtype=float), KNOTS, rtol=0, atol=1e-7)
    except (TypeError, ValueError):
        matches = False
    if not matches:
        raise ValueError('feedforward knots differ from firmware')
    return weights


def predict_float32(rpm, weights):
    """Mirror firmware float operations, including accumulation and clamping.

    Offline reference only; not execution of the compiled C firmware.
    """
    speeds = np.asarray(rpm, dtype=np.float32)
    target_rad_s = speeds * np.float32(6.28318530718) / np.float32(60)
    x = np.clip(target_rad_s/np.float32(18.84955592), -1, 1)
    coefficients = np.asarray(weights, dtype=np.float32)/np.float32(10000)
    value = np.full_like(x, coefficients[8])
    for knot, coefficient in zip(KNOTS.astype(np.float32), coefficients[:8]):
        value += coefficient*np.maximum(x-knot, np.float32(0))
    return np.clip(value, -.3, .3)
