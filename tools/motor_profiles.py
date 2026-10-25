"""Measured demo speed profiles. Gains use firmware milli-units."""


def speed_gains(rpm, profile):
    if profile == 'lowfast':
        # Measured low-speed range only; NN selection remains independent.
        if not 15 <= abs(rpm) <= 40:
            raise ValueError('lowfast profile is limited to 15..40 RPM magnitude')
        return 150, 100
    if profile == 'smooth':
        # Opt-in low-speed demo; do not extrapolate this high-Kp setting.
        if abs(rpm) > 40:
            raise ValueError('smooth profile is limited to +/-40 RPM; use tuned above that')
        return 50, 100
    if profile == 'baseline':
        return 8, 284
    if profile != 'tuned':
        raise ValueError('profile must be baseline, tuned, smooth or lowfast')
    magnitude = abs(rpm)
    if magnitude <= 40:
        return 20, 284
    if magnitude <= 80:
        return 8, 650
    return 8, 284
