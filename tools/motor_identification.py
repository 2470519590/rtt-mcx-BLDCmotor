"""Sinestream trace parsing, CSV recording and offline Uq->speed identification."""
import argparse
import csv
import math
import time
from pathlib import Path

TRACE_FIELDS = [
    'sequence', 'tick_ms', 'point', 'stage', 'frequency_mhz', 'phase_mrad',
    'position_mrad', 'speed_mrad_s', 'target_mrad_s', 'excitation_mv', 'uq_mv',
    'sensor_ok', 'calibrated', 'mode', 'fault', 'clipped', 'raw_angle',
    'kp_milli', 'voltage_limit_mv',
]
CSV_FIELDS = ['host_monotonic_s'] + TRACE_FIELDS


def parse_trace(line):
    parts = line.split(',')
    if parts[0] != 'I' or len(parts) != len(TRACE_FIELDS) + 1:
        raise ValueError('Invalid identification frame')
    row = dict(zip(TRACE_FIELDS, (int(value) for value in parts[1:])))
    if row['mode'] != 7 or row['stage'] not in (0, 1, 2) or row['point'] < 1:
        raise ValueError('Invalid identification state')
    row['host_monotonic_s'] = time.monotonic()
    return row


def telemetry_from_trace(row):
    """Preserve CLI state display; the control-loop trace remains authoritative."""
    return (f"T,{row['sequence']},{row['tick_ms']},{row['raw_angle']},"
            f"{row['speed_mrad_s']},{row['sensor_ok']},{row['mode']},"
            f"{row['calibrated']},{row['target_mrad_s']},{row['uq_mv']},"
            f"{row['fault']},0")


def validate_settings(rpm, amplitude, first_hz, last_hz, points):
    if not 0 < abs(rpm) <= 1000:
        raise ValueError('Baseline RPM must be nonzero within +/-1000')
    if not all(math.isfinite(x) for x in (amplitude, first_hz, last_hz)):
        raise ValueError('Identification settings must be finite')
    if not .005 <= amplitude <= .300:
        raise ValueError('Excitation amplitude must be 0.005..0.300 V')
    if not .1 <= first_hz <= last_hz <= 10:
        raise ValueError('Frequency range must satisfy 0.1 <= first <= last <= 10 Hz')
    if not 2 <= points <= 24:
        raise ValueError('Use 2..24 frequency points')
    return [('J', rpm), ('A', round(amplitude * 1000)),
            ('L', round(first_hz * 1000)), ('U', round(last_hz * 1000)),
            ('N', points)]


class TraceRecorder:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.file = self.path.open('x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.started = False
        self.rows = 0
        self.point = None

    def write(self, row):
        self.started = True
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % 100 == 0:
            self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()


def fit_phasor(time_s, phase, signal):
    import numpy as np
    # Linear trend is essential for unwrapped position at nonzero baseline speed.
    design = np.column_stack((np.ones(len(time_s)), time_s - time_s.mean(),
                              np.cos(phase), np.sin(phase)))
    coefficients, _, rank, _ = np.linalg.lstsq(design, signal, rcond=None)
    if rank != 4:
        raise ValueError('Insufficient independent samples')
    fitted = design @ coefficients
    signal_without_trend = signal - design[:, :2] @ coefficients[:2]
    power = float(np.mean(signal_without_trend ** 2))
    fit_quality = 1 - float(np.mean((signal - fitted) ** 2)) / max(power, 1e-20)
    return complex(coefficients[2], -coefficients[3]), fit_quality


def analyze_rows(rows):
    import numpy as np
    results = []
    for point in sorted({int(row['point']) for row in rows}):
        all_point = [r for r in rows if int(r['point']) == point and int(r['stage']) > 0]
        measured = [r for r in all_point if int(r['stage']) == 2]
        result = {'point': point, 'valid': False, 'reason': '', 'samples': len(measured)}
        reasons = []
        if not measured:
            result['reason'] = 'no measurement stage'
            results.append(result)
            continue
        hz = np.median([float(r['frequency_mhz']) for r in measured]) / 1000
        result['frequency_hz'] = hz
        ticks = np.array([int(r['tick_ms']) for r in measured], dtype=np.int64)
        # uint32 MCU timestamp wraps every ~49 days; use modular differences.
        dt = np.diff(ticks) % (1 << 32) / 1000
        t = np.concatenate(([0.], np.cumsum(dt)))
        if len(measured) < 30 or hz <= 0 or t[-1] * hz < 4.5:
            reasons.append('incomplete frequency point (<4.5 cycles or <30 samples)')
        if any(int(r['clipped']) for r in all_point):
            reasons.append('voltage clipping')
        if any(not int(r['sensor_ok']) or not int(r['calibrated']) or int(r['fault'])
               for r in all_point):
            reasons.append('encoder/calibration/fault invalid')
        if dt.size and (np.any(dt <= 0) or np.max(dt) * hz > .2):
            reasons.append('duplicate timestamps or acquisition gap >0.2 cycle')
        if reasons:
            result['reason'] = '; '.join(reasons)
            results.append(result)
            continue
        phase = np.array([float(r['phase_mrad']) for r in measured]) / 1000
        uq = np.array([float(r['uq_mv']) for r in measured]) / 1000
        position = np.array([float(r['position_mrad']) for r in measured]) / 1000
        speed = np.array([float(r['speed_mrad_s']) for r in measured]) / 1000
        excitation = np.array([float(r['excitation_mv']) for r in measured]) / 1000
        try:
            u, u_quality = fit_phasor(t, phase, uq)
            theta, position_quality = fit_phasor(t, phase, position)
            omega_filtered, speed_quality = fit_phasor(t, phase, speed)
            d, _ = fit_phasor(t, phase, excitation)
        except ValueError as exc:
            result['reason'] = str(exc)
            results.append(result)
            continue
        if abs(u) < .001 or abs(d) < .001:
            result['reason'] = 'insufficient actual input/excitation amplitude'
            results.append(result)
            continue
        # Projection onto independent excitation phase is the closed-loop
        # instrument. H = (omega/d)/(u/d), not omega/target RPM.
        plant = (1j * 2 * math.pi * hz * theta / d) / (u / d)
        filtered = (omega_filtered / d) / (u / d)
        gain = abs(plant)
        if gain < 1e-12 or not math.isfinite(gain):
            result['reason'] = 'no measurable mechanical response'
            results.append(result)
            continue
        # Uq also contains the velocity-loop baseline term, so its raw
        # sinusoidal fit is not a validity gate for the plant response.
        # The excitation projection and position fit are the relevant checks.
        result.update(valid=position_quality >= .5,
                      reason='' if position_quality >= .5
                      else 'poor fundamental fit; repeat with a suitable excitation',
                      gain_rad_s_per_v=gain, gain_db=20 * math.log10(gain),
                      plant_real=plant.real, plant_imag=plant.imag,
                      phase_deg=math.degrees(math.atan2(plant.imag, plant.real)),
                      filtered_gain_rad_s_per_v=abs(filtered),
                      filtered_phase_deg=math.degrees(math.atan2(filtered.imag, filtered.real)),
                      input_amplitude_v=abs(u), excitation_amplitude_v=abs(d),
                      input_fit=u_quality, position_fit=position_quality,
                      filtered_speed_fit=speed_quality,
                      effective_sample_hz=1 / float(np.median(dt)),
                      max_gap_ms=1000 * float(np.max(dt)))
        results.append(result)
    return results


def fit_first_order(results):
    """Optional first-order candidate, explicitly rejected if the fit is poor."""
    import numpy as np
    valid = [r for r in results if r['valid']]
    if len(valid) < 3:
        return None
    w = np.array([2 * math.pi * r['frequency_hz'] for r in valid])
    h = np.array([complex(r['plant_real'], r['plant_imag']) for r in valid])
    inv = 1 / h
    a = float(np.mean(inv.real))
    b = float(np.dot(w, inv.imag) / np.dot(w, w))
    if a <= 0 or b <= 0:
        return None
    gain, tau = 1 / a, b / a
    prediction = gain / (1 + 1j * w * tau)
    error = float(np.linalg.norm(prediction - h) / np.linalg.norm(h))
    return {'gain': gain, 'tau_s': tau, 'relative_error': error,
            'accepted': error <= .2}


def plot_frequency_response(results, output):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    valid = [r for r in results if r['valid']]
    if not valid:
        return False
    output = Path(output)
    if output.exists():
        raise FileExistsError(f'Plot already exists: {output}')
    hz = [r['frequency_hz'] for r in valid]
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axes[0].semilogx(hz, [r['gain_db'] for r in valid], 'o-', label='Uq -> speed (from position)')
    axes[0].semilogx(hz, [20 * math.log10(max(r['filtered_gain_rad_s_per_v'], 1e-12))
                         for r in valid], 'x--', label='Uq -> filtered speed')
    axes[1].semilogx(hz, np.degrees(np.unwrap(np.radians(
        [r['phase_deg'] for r in valid]))), 'o-')
    axes[1].semilogx(hz, np.degrees(np.unwrap(np.radians(
        [r['filtered_phase_deg'] for r in valid]))), 'x--')
    axes[0].set_ylabel('Gain [dB re (rad/s)/V]')
    axes[1].set_ylabel('Phase [deg]')
    axes[1].set_xlabel('Excitation frequency [Hz]')
    for axis in axes:
        axis.grid(True, which='both', alpha=.3)
    axes[0].legend()
    fig.tight_layout()
    try:
        with output.open('x', encoding='utf-8') as file:
            fig.savefig(file, format='svg')
    finally:
        plt.close(fig)
    return True


def analyze_file(source, output=None):
    source = Path(source).resolve()
    with source.open(newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        if not set(TRACE_FIELDS).issubset(reader.fieldnames or []):
            raise ValueError('Not an identification CSV')
        rows = list(reader)
    if not rows:
        raise ValueError('CSV has no identification samples')
    results = analyze_rows(rows)
    output = Path(output).resolve() if output else source.with_name(source.stem + '_frf.csv')
    fields = ['point', 'frequency_hz', 'valid', 'reason', 'samples',
              'gain_rad_s_per_v', 'gain_db', 'phase_deg', 'plant_real', 'plant_imag',
              'filtered_gain_rad_s_per_v', 'filtered_phase_deg',
              'input_amplitude_v', 'excitation_amplitude_v',
              'input_fit', 'position_fit', 'filtered_speed_fit',
              'effective_sample_hz', 'max_gap_ms']
    with output.open('x', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
    for row in results:
        if row['valid']:
            print(f"{row['frequency_hz']:7.3f} Hz  {row['gain_db']:8.2f} dB  "
                  f"{row['phase_deg']:8.2f} deg  fit={row['position_fit']:.3f}")
        else:
            print(f"Point {row['point']}: INVALID: {row['reason']}")
    print(f'Frequency response saved: {output}')
    print('Gain unit: (rad/s)/V. Fundamental fit is NOT spectral coherence.')
    candidate = fit_first_order(results)
    if candidate:
        status = 'candidate' if candidate['accepted'] else 'REJECTED'
        print(f"First-order {status}: G(s)={candidate['gain']:.6g}/"
              f"({candidate['tau_s']:.6g}*s+1), complex relative error="
              f"{candidate['relative_error']:.1%}. Validate on another experiment.")
    else:
        print('No credible first-order candidate from the available valid points.')
    plot_path = output.with_suffix('.svg')
    if plot_frequency_response(results, plot_path):
        print(f'Bode plot saved: {plot_path}')
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--output')
    args = parser.parse_args()
    try:
        analyze_file(args.csv, args.output)
    except (ValueError, OSError, ImportError) as exc:
        parser.exit(1, f'Analysis failed: {exc}\n')


if __name__ == '__main__':
    main()
