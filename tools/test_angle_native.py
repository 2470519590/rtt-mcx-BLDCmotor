"""Compile and run real C under host sanitizers, never open hardware."""
import shutil
import subprocess
import unittest
from pathlib import Path


class NativeAngleTests(unittest.TestCase):
    def test_native_sanitizers(self):
        if not shutil.which('wsl.exe'):
            self.skipTest('WSL GCC unavailable')
        root = Path(__file__).resolve().parents[1]
        posix = '/mnt/' + root.drive[0].lower() + root.as_posix()[2:]
        probe = subprocess.run(['wsl.exe', '--exec', 'sh', '-c',
                                'command -v gcc'], capture_output=True, timeout=15)
        if probe.returncode:
            self.skipTest('WSL GCC unavailable')
        output = root / 'debug' / 'host_test_tmp'
        output.mkdir(parents=True, exist_ok=True)
        compiler = ['wsl.exe', '--exec', 'gcc', '-std=c11', '-Wall', '-Wextra',
                    '-Werror', '-g', '-O1', '-fno-omit-frame-pointer',
                    '-fsanitize=address,undefined,float-cast-overflow',
                    '-I' + posix + '/app', '-o', posix + '/debug/host_test_tmp/angle_native_test',
                    posix + '/tools/test_angle_native.c', posix + '/app/nn_controller.c',
                    posix + '/app/angle_shadow.c', '-lm']
        subprocess.run(compiler, check=True, capture_output=True, timeout=45)
        result = subprocess.run(['wsl.exe', '--exec',
                                 posix + '/debug/host_test_tmp/angle_native_test'],
                                check=True, capture_output=True, timeout=15)
        self.assertIn(b'native angle checks passed', result.stdout)
