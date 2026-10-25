import subprocess
import unittest
from pathlib import Path
from test_uart_rx_queue import wsl_path


class SpeedTransitionTests(unittest.TestCase):
    def test_actual_c_reset_keep_and_mode_changes(self):
        root = Path(__file__).resolve().parents[1]
        binary = root / 'debug' / 'speed_transition_host'
        subprocess.run(['wsl.exe', '--exec', 'gcc', '-std=c99', '-Wall',
                        '-Wextra', '-Werror', '-fsanitize=address,undefined',
                        '-I' + wsl_path(root / 'app'),
                        wsl_path(root / 'tools/speed_transition_harness.c'),
                        '-o', wsl_path(binary)], check=True, capture_output=True)
        subprocess.run(['wsl.exe', '--exec', wsl_path(binary)],
                       check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
