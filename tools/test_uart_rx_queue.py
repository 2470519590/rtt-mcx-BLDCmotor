"""Actual host C ring test; no hardware access."""
import subprocess
import unittest
from pathlib import Path


def wsl_path(path):
    return '/mnt/' + path.drive[0].lower() + str(path)[2:].replace('\\', '/')


class UartRxQueueTests(unittest.TestCase):
    def test_actual_c_queue_wrap_and_error_resynchronization(self):
        root = Path(__file__).resolve().parents[1]
        binary = root / 'debug' / 'uart_rx_queue_host'
        subprocess.run(['wsl.exe', '--exec', 'gcc', '-std=c99', '-Wall', '-Wextra',
                        '-Werror', '-fsanitize=address,undefined',
                        '-I' + wsl_path(root / 'app'),
                        wsl_path(root / 'app/uart_rx_queue.c'),
                        wsl_path(root / 'tools/uart_rx_queue_harness.c'),
                        '-o', wsl_path(binary)], capture_output=True, check=True)
        subprocess.run(['wsl.exe', '--exec', wsl_path(binary)], capture_output=True, check=True)


if __name__ == '__main__':
    unittest.main()
