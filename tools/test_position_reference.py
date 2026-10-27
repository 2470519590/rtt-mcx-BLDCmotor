"""Run the actual optional C position planner under host sanitizers."""
import subprocess
import unittest
from pathlib import Path

from test_uart_rx_queue import wsl_path


class PositionReferenceTests(unittest.TestCase):
    def test_actual_position_planner(self):
        root = Path(__file__).resolve().parents[1]
        output = root / "debug" / "host_test_tmp" / "position_reference_host"
        output.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["wsl.exe", "--exec", "gcc", "-std=c99", "-Wall", "-Wextra",
                        "-Werror", "-fsanitize=address,undefined",
                        "-I" + wsl_path(root / "app"),
                        wsl_path(root / "tools" / "position_reference_harness.c"),
                        "-lm", "-o", wsl_path(output)],
                       check=True, capture_output=True, timeout=30)
        result = subprocess.run(["wsl.exe", "--exec", wsl_path(output)],
                                check=True, capture_output=True, timeout=15)
        self.assertIn(b"position reference checks passed", result.stdout)


if __name__ == "__main__":
    unittest.main()
