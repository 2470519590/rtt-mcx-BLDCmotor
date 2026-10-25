"""Program the just-built debug/test1.elf with the configured J-Link.

Replaces the PowerShell wrapper only because this sandboxed shell cannot stream
a child program's console output to a pipe and drops piped J-Link output. The
flags, probe serial and command list are identical to tools/flash_jlink.ps1:
J-Link, MCXA346, SWD 100 kHz, loadfile/reset/go, output to a file, then the
file is printed. A reset clears the encoder calibration and any RAM angle
model; recalibrate and re-upload afterwards.
"""
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
JLINK = Path(r'C:\Program Files\JLink_V960\JLink.exe')
PROBE = '602710971'


def flash(log_path=None):
    elf = ROOT / 'debug' / 'test1.elf'
    if not elf.is_file():
        raise SystemExit(f'ELF missing: {elf}')
    if not JLINK.is_file():
        raise SystemExit(f'J-Link not found: {JLINK}')
    log_path = Path(log_path or (ROOT / 'debug' / 'jlink_flash.log'))
    script = log_path.with_suffix('.jlink')
    script.write_text(f'loadfile "{elf}"\nr\ng\nq\n', encoding='ascii')
    started = time.time()
    done = subprocess.run([str(JLINK), '-device', 'MCXA346', '-if', 'SWD', '-speed', '100',
                           '-SelectEmuBySN', PROBE, '-autoconnect', '1', '-ExitOnError', '1',
                           '-CommandFile', str(script)],
                          capture_output=True, text=True, timeout=180, cwd=str(ROOT))
    output = (done.stdout or '') + (done.stderr or '')
    log_path.write_text(f'elapsed {time.time() - started:.1f}s\n{output}')
    print(output)
    print(f'exit={done.returncode} log={log_path}')
    ok = done.returncode == 0 and 'O.K.' in output
    if not ok:
        raise SystemExit('J-Link reported an error; inspect the log above')
    return log_path


if __name__ == '__main__':
    flash(sys.argv[1] if len(sys.argv) > 1 else None)
