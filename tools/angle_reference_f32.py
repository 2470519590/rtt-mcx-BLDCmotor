"""Float32 reference for the angle candidate, matching the MCU evaluation order.

The original reference computed in float64 while the firmware accumulates
float32, which is where the apparent 0.13 V shadow "mismatch" came from once
the double-scaling bug was fixed. This keeps the feature construction and the
accumulation in float32, one neuron at a time, in the same order the C code
uses, so parity comparisons mean what they claim.

Consequence for earlier documentation: the previously reported parity
(1.34e-7 V host, quantized intervals on board) could not have detected a
double-scaling bug, because std multiplies and divides cancel in that
comparison if the comparison itself is in float64.
"""
import numpy as np

HARMONICS_F32 = (np.float32(1), np.float32(2), np.float32(7), np.float32(14), np.float32(21))
TWO_PI_F32 = np.float32(6.28318530718)


def predict_f32(position_rad, rpm, phase_rad, model, exact_tensor=False):
    """Mirror of angle_shadow_predict(): milli-truncated inputs, float32.

    The firmware truncates position and target speed to integer milli-units,
    the same units the telemetry carries, so a recording can reproduce the
    inference exactly rather than within a ramp-duration window.

    ``exact_tensor=True`` skips the truncation: the firmware also records the
    untruncated tensor of the last inference, and that is the only way to
    compare the prediction below the 1 mm / 1 mrad/s quantization.
    """
    angle32 = np.asarray(position_rad, dtype=np.float32)
    target32 = np.asarray(rpm, dtype=np.float32) * TWO_PI_F32 / np.float32(60)
    if not exact_tensor:
        angle32 = (np.trunc(angle32 * np.float32(1000)) / np.float32(1000)).astype(np.float32)
        target32 = (np.trunc(target32 * np.float32(1000)) / np.float32(1000)).astype(np.float32)
    angle32 = angle32.astype(np.float32)
    phase32 = np.broadcast_to(np.asarray(phase_rad, dtype=np.float32), angle32.shape)
    rpm32 = (target32 * np.float32(60) / TWO_PI_F32).astype(np.float32)
    columns = [rpm32 / np.float32(40),
               (rpm32 > 0).astype(np.float32) - (rpm32 < 0).astype(np.float32)]
    for h in HARMONICS_F32:
        shifted = h * angle32 + h * phase32
        columns.append(np.sin(shifted).astype(np.float32))
        columns.append(np.cos(shifted).astype(np.float32))
    x = np.column_stack(columns).astype(np.float32)
    mean = model['input_mean'].astype(np.float32)
    std = model['input_std'].astype(np.float32)
    z = ((x - mean) / std).astype(np.float32)
    weights = model['hidden_weights'].astype(np.float32)
    bias = model['hidden_bias'].astype(np.float32)
    output_weights = model['output_weights'].astype(np.float32)
    output = np.full(len(z), np.float32(model['output_bias']), dtype=np.float32)
    for i in range(weights.shape[0]):
        hidden = np.full(len(z), bias[i], dtype=np.float32)
        for j in range(weights.shape[1]):
            hidden = (hidden + weights[i, j] * z[:, j]).astype(np.float32)
        hidden = np.maximum(hidden, np.float32(0))
        output = (output + output_weights[i] * hidden).astype(np.float32)
    return output


def expected_float32_tolerance(model):
    """Per-sample float32 rounding allowance for a compact parity test."""
    return float(np.float32(4e-4)) + 1.5e-3 * float(np.max(np.abs(model['output_weights'])))
