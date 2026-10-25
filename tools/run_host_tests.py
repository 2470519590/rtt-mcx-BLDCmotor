"""Run the host test suite from the py310 script directory.

Two environment limitations of the current sandbox are handled here without
touching the tests themselves:

* Deleting directories under the system temp area is denied, which makes
  ``tempfile.TemporaryDirectory`` raise in its cleanup even when the test body
  passed. TMP/TEMP/TMPDIR are redirected to a workspace folder.
* The C parity tests need WSL GCC or QEMU; when neither is available they are
  reported as skipped with the reason instead of silently passing.

Usage: python tools/run_host_tests.py
"""
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'tools'


# Environment errors that are not test failures: which "error" is which.
def classify(trace):
    if 'CalledProcessError' in trace and 'wsl' in trace.lower():
        return 'wsl_unavailable'
    if 'PermissionError' in trace and 'tmp' in trace.lower():
        return 'sandbox_temp_delete'
    return 'real_failure'


def split_environmental(result):
    """Records sandbox-only errors separately; does not change the verdict."""
    env = []
    for test, trace in result.errors:
        if isinstance(test, str) and 'setUpClass' in test:
            env.append((test, trace))
        elif classify(trace) == 'sandbox_temp_delete':
            env.append((test, trace))
    return env


def main():
    temp = ROOT / 'debug' / 'host_test_tmp'
    temp.mkdir(parents=True, exist_ok=True)
    for name in ('TMP', 'TEMP', 'TMPDIR'):
        os.environ[name] = str(temp)
    tempfile.tempdir = None
    sys.path.insert(0, str(TOOLS))
    suite = unittest.defaultTestLoader.discover(str(TOOLS), pattern='test_*.py',
                                                top_level_dir=str(TOOLS))
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    environmental = split_environmental(result)
    print(f'\nran={result.testsRun} real_failures={len(result.failures)} '
          f'real_errors={len(result.errors)} skipped={len(result.skipped)} '
          f'sandbox_environment_errors={len(environmental)}')
    for test, _ in environmental:
        name = test if isinstance(test, str) else test.id()
        print('  environment-only: ' + name)
    if result.skipped:
        print('skipped: ' + str(result.skipped[0][1]).splitlines()[0])
    return 0 if result.wasSuccessful() and not environmental else 1


if __name__ == '__main__':
    raise SystemExit(main())

