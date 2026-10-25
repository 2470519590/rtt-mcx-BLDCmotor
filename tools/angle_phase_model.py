"""Shared RAM-upload contract for the angle model (dynamic phase).

Python is the reference for the firmware upload protocol. Keep this file and
app/angle_shadow.c in sync: the host C tests compare C output against
``predict`` here, including speed-dependent phase.

Model layout (matches data/angle_residual_candidate_v2.npz):
  input[0]       rpm/40
  input[1]       sign(rpm)
  input[2+2k]    sin(h_k * position + h_k * phase(rpm))  for h_k = 1,2,7,14,21
  input[3+2k]    cos(h_k * position + h_k * phase(rpm))
The firmware adds the phase per harmonic: harmonic k lands on k*phase, which is
exactly what these features do. ``phase=0`` reproduces the compiled candidate.

Upload protocol, one opcode+integer per line, milli units, RAM only.
Every integer argument is limited to +/-100000 by the command parser:
  M<i>       select point/neuron i (0..7); resets the staged neuron row
  u<milli>   phase for the selected normalization point
  p<milli>   commit the point: normalization speed in milli-RPM
  m<mode>    0 hold / 1 interpolate, +2 marks compact 2.6 weight fields
  g<field>   20000 + index*4000 + code, index 0..11; code is 0..1999 for
             positive or 2000..3999 for negative, keeping fields disjoint
             and the maximum argument below 68000
  i<milli>   output weight for the selected neuron (milli)
  n<0>       commit that neuron (needs all 12 g values first)
  q<milli>   output bias in milli-volt
  x<0>       read-only: X,result,point,count,valid,points,interp,g_stage
  z<0>       read-only build stamp (3001 = dynamic phase upload)
"""
import hashlib
from pathlib import Path

import numpy as np

HARMONICS = (1, 2, 7, 14, 21)
NORM_POINTS = 8
WEIGHT_SCALE = 10000  # firmware integer weights are 1e-4
PHASE_LIMIT_MILLI = 3142


class PhaseTable:
    """Sorted normalization points with one phase each."""

    def __init__(self, points, interp=1):
        if len(points) > NORM_POINTS:
            raise ValueError(f'at most {NORM_POINTS} normalization points')
        pairs = sorted((float(p), float(q)) for p, q in points)
        # Duplicates must be rejected explicitly: sorting would hide them and
        # the firmware requires strictly ascending normalization speeds.
        supplied = [float(p) for p, _ in points]
        if len(set(supplied)) != len(supplied):
            raise ValueError('normalization speeds must be unique')
        for _, phase in pairs:
            if not np.isfinite(phase) or abs(phase) > np.pi:
                raise ValueError('phase must be finite within +/-pi')
        self.points = pairs
        self.interp = int(interp)
        if self.interp not in (0, 1):
            raise ValueError('interp must be 0 (hold) or 1 (linear)')

    @classmethod
    def constant(cls, phase_rad):
        """One point: the old single-phase configuration (hold mode)."""
        return cls([(0.0, float(phase_rad))], interp=0)

    def phase(self, rpm):
        rpm = np.asarray(rpm, dtype=float)
        if not self.points:
            return np.zeros_like(rpm)
        speeds = np.array([p for p, _ in self.points], dtype=float)
        phases = np.array([q for _, q in self.points], dtype=float)
        if len(self.points) == 1:
            return np.full_like(rpm, phases[0])
        if self.interp == 0:
            nearest = np.argmin(np.abs(rpm[..., None] - speeds), axis=-1)
            return phases[nearest]
        return np.interp(rpm, speeds, phases)

    def as_dict(self):
        return {'points': [{'rpm': p, 'phase_rad': q} for p, q in self.points],
                'interp': self.interp}


def load_model(path):
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
        model['sha256'] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    return model


def features(position_rad, rpm, phase_rad):
    """Harmonic features at an already-resolved phase (no delay applied)."""
    angle = np.asarray(position_rad, dtype=np.float64)
    speed = np.broadcast_to(np.asarray(rpm, dtype=np.float64), angle.shape)
    phase = np.broadcast_to(np.asarray(phase_rad, dtype=np.float64), angle.shape)
    return np.column_stack([speed / 40, np.sign(speed)] +
                           [f(h * angle + h * phase) for h in HARMONICS for f in (np.sin, np.cos)])


def predict(position_rad, rpm, model, phase_rad):
    """Float32 reference used for every parity claim (see angle_reference_f32).

    Kept as one entry point so callers cannot silently regress to the float64
    version that previously masked a firmware double-scaling bug.
    """
    from angle_reference_f32 import predict_f32
    return predict_f32(position_rad, rpm, phase_rad, model)


def delay_rad(rpm, delay_s):
    """Angle the model has to lead by for a constant time delay.

    The motor state at time t reaches the label only after the MCU samples,
    filters and applies it. The signature is a lag in *time*, so the equivalent
    mechanical angle grows with speed: lag_rad(s) = 2*pi*f*s, f = rpm/60, and
    the sign follows the direction of rotation.
    """
    rpm = np.asarray(rpm, dtype=float)
    return 2 * np.pi * (rpm / 60.0) * float(delay_s)


def predict_f64(position_rad, rpm, model, phase_rad):
    """Analytic reference only; never use it to claim hardware parity."""
    x = (features(position_rad, rpm, phase_rad) - model['input_mean']) / model['input_std']
    return np.maximum(x @ model['hidden_weights'].T + model['hidden_bias'], 0) @ \
        model['output_weights'] + model['output_bias']


def weight_milli(value, index):
    milli = int(round(float(value) * WEIGHT_SCALE))
    if not 0 <= index <= 11:
        raise ValueError('weight index must be 0..11')
    if abs(milli) > 99999:
        raise ValueError('weight does not fit the upload field')
    return index * 100000 + milli


def output_milli(value):
    """Output weight in 1e-4 units, bounded by the command argument limit.

    This used to be capped at 50 (0.005), which silently truncated real
    weights near 0.1 and made an upload look accepted while applying a
    different network. Reject out-of-range values instead of truncating.
    """
    milli = int(round(float(value) * WEIGHT_SCALE))
    if abs(milli) > 999999:
        raise ValueError(f'output weight {value} exceeds the command argument limit (+/-99.9999)')
    return milli


def phase_milli(value):
    milli = int(round(float(value) * 1000))
    if abs(milli) > PHASE_LIMIT_MILLI:
        raise ValueError('phase out of range')
    return milli


# Firmware m0/m1 select phase interpolation; m2/m3 additionally mark the g
# fields as the compact 2.6 representation. Bit 0 is interpolation in both.
UPLOAD_MODE_COMPACT = 2


def upload_mode(interp):
    return UPLOAD_MODE_COMPACT + int(interp)


def row_commands(model, neuron, mode_interp):
    """Twelve g entries (2e4 + index*4000 + signed payload), i<output>.

    The 4000 stride leaves disjoint 2000-wide positive/negative payloads.
    All 12 fields fit below the parser's 100000 argument limit. Mode 2/3
    marks the compact representation.
    """
    row = np.r_[model['hidden_bias'][neuron], model['hidden_weights'][neuron]]
    commands = []
    for index, value in enumerate(row[1:]):
        milli = int(round(float(value) * 1000))
        if abs(milli) > 1999:
            raise ValueError(f'weight {index} does not fit the compact field')
        # Keep coefficient sign separate from the encoded index.
        digit = abs(milli) + (2000 if milli < 0 else 0)
        commands.append(('g', 20000 + index * 4000 + digit))
    commands.append(('i', output_milli(model['output_weights'][neuron])))
    bias = int(round(float(model['hidden_bias'][neuron]) * 10000))
    if abs(bias) > 99000:
        raise ValueError('hidden bias exceeds the h command range')
    commands.append(('h', bias))
    commands.append(('n', 0))
    commands.append(('m', upload_mode(mode_interp)))
    return commands


def roundtrip_weight(model, neuron):
    """Host values exactly as the firmware will hold them after upload."""
    weights = model['hidden_weights'][neuron]
    return np.round(np.abs(weights) * 1000) / 1000


def upload_sequence(model, table):
    """Ordered (op, value) list: all neurons, then the phase points.

    An earlier version interleaved M<neuron> with M<point>, and both opcodes use
    the same staging slot. The firmware now rejects a neuron index beyond the
    compiled neuron count, but the ordering here also keeps the two uses apart:
    every neuron is committed before the first point is selected.
    """
    table.as_dict()  # validation
    commands = []
    interp = table.interp
    for neuron in range(int(model['output_weights'].shape[0])):
        commands.append(('M', neuron))
        commands += row_commands(model, neuron, interp)
    commands.append(('q', int(round(float(model['output_bias']) * 1000))))
    for index, (rpm, phase) in enumerate(table.points):
        commands.append(('M', index))
        commands.append(('u', phase_milli(phase)))
        commands.append(('p', int(round(rpm * 1000))))
    commands.append(('m', upload_mode(interp)))
    return commands


def verify_upload(model, table, read_row):
    """Compare uploaded coefficients with the source model.

    ``read_row(neuron)`` returns (bias, 12 signed weights, output_weight) as
    the MCU holds them; signs are part of the contract because dropping them
    turned the trained network into its magnitude version.
    """
    for neuron in range(int(model['output_weights'].shape[0])):
        have = np.r_[read_row(neuron)]
        expected = np.r_[model['hidden_bias'][neuron],
                         model['hidden_weights'][neuron],
                         model['output_weights'][neuron]]
        difference = np.abs(np.asarray(have, dtype=float) - expected)
        if float(difference.max()) > 1.1e-3:
            worst = int(np.argmax(difference))
            raise RuntimeError(f'neuron {neuron} coefficient {worst} differs by '
                               f'{difference[worst]:.4f} (expected {expected[worst]:.4f}, '
                               f'MCU {have[worst]:.4f})')
    return dict(neurons=int(model['output_weights'].shape[0]),
                points=len(table.points), interp=table.interp,
                note='signed weights, bias and output weight verified')
