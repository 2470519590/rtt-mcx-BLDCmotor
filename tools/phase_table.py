"""Parse a fit_angle_phase report into a firmware PhaseTable (radians)."""
import argparse
import json
from pathlib import Path

from angle_phase_model import PhaseTable


def load_table(report_path, name):
    report = json.loads(Path(report_path).read_text(encoding='utf-8'))
    entry = report['phase_tables'][name]
    return PhaseTable([(p['rpm'], p['phase_rad']) for p in entry['points']],
                      interp=entry['interp'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report')
    parser.add_argument('table', nargs='?', default='global')
    args = parser.parse_args()
    table = load_table(args.report, args.table)
    for rpm, phase in table.points:
        print(f'{rpm:+8.1f} RPM  phase {phase:+.5f} rad  ({phase * 57.2958:+.2f} deg)')
    print('interp', table.interp)


if __name__ == '__main__':
    main()
