"""Automate a small speed/position learning-telemetry collection run."""
import argparse
import csv
import threading
import time
from pathlib import Path

from nn_dataset import LEARNING_FIELDS, parse_learning
from motor_port import resolve_port


def tx(port, op, value=0, write_lock=None):
    data = f"{op}{value}\n".encode("ascii")
    if write_lock is None:
        write_lock = threading.Lock()
    with write_lock:
        for byte in data:
            port.write(bytes([byte]))
            time.sleep(0.005)


def send_checked(port, op, value, write_lock):
    tx(port, op, value, write_lock)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        if not line.startswith("ACK,"):
            continue
        parts = line.split(",")
        if len(parts) >= 3 and parts[1] == op:
            if parts[2] != "0":
                raise RuntimeError(f"firmware rejected {op}{value}: {line}")
            return
    raise RuntimeError(f"no ACK for {op}{value}; collection aborted")


def read_state(port, timeout=3.0, require_calibrated=True):
    """Wait for a fresh healthy T frame and return its fields."""
    deadline = time.monotonic() + timeout
    latest = None
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        if not line.startswith("T,"):
            continue
        latest = line
        parts = line.split(",")
        if (len(parts) >= 12 and parts[5] == "1" and
                (not require_calibrated or parts[7] == "1") and parts[10] == "0"):
            return parts
    raise RuntimeError(
        "firmware is not ready for collection; check stop/calibrate first "
        f"and check state (last frame: {latest or 'none'})")


def wait_for_state(port, timeout=3.0):
    """Require a fresh healthy/calibrated T frame before collecting."""
    return read_state(port, timeout, require_calibrated=True)


def collect(port_name, output, seconds_per_step=4.0, include_position=True,
            auto_calibrate=False):
    import serial
    port_name = resolve_port(port_name)
    if Path(output).exists():
        raise FileExistsError(output)
    steps = [("R", 30), ("R", 60), ("R", 120), ("R", 180),
             ("R", -30), ("R", -60), ("R", -120)]
    if include_position:
        # X uses milliradians, not millidegrees.
        steps += [("X", 87), ("X", 175), ("X", -175)]
    rows = []
    with serial.Serial(port_name, 115200, timeout=.05, write_timeout=.5) as port:
        port.dtr = False; port.rts = False; port.reset_input_buffer()
        write_lock = threading.Lock()
        heartbeat_stop = threading.Event()
        def heartbeat():
            while not heartbeat_stop.wait(.25):
                try:
                    tx(port, "H", 0, write_lock)
                except (OSError, serial.SerialException):
                    heartbeat_stop.set()
        heartbeat_thread = threading.Thread(target=heartbeat, daemon=True)
        heartbeat_thread.start()
        try:
            send_checked(port, "S", 0, write_lock); time.sleep(.25)
            send_checked(port, "T", 0, write_lock)
            state = read_state(port, require_calibrated=False)
            if state[7] != "1":
                if not auto_calibrate:
                    raise RuntimeError(
                        "encoder is healthy but not calibrated; run calibrate "
                        "or pass --auto-calibrate")
                send_checked(port, "C", 0, write_lock)
                print("Calibration started; waiting for calibrated=1...")
                read_state(port, timeout=8.0, require_calibrated=True)
                time.sleep(1.0)
            wait_for_state(port)
            send_checked(port, "T", 1, write_lock)
            send_checked(port, "G", 1, write_lock)
            send_checked(port, "K", 0, write_lock)
            for op, value in steps:
                # Start each segment from the same stopped baseline. This also
                # avoids commanding a large direct reversal under load.
                send_checked(port, "S", 0, write_lock)
                send_checked(port, op, value, write_lock)
                deadline = time.monotonic() + seconds_per_step
                while time.monotonic() < deadline:
                    line = port.readline().decode("ascii", errors="replace").strip()
                    if not line.startswith("L,"):
                        continue
                    try:
                        row = parse_learning(line)
                    except ValueError:
                        continue
                    row["host_monotonic_s"] = time.monotonic()
                    rows.append(row)
        finally:
            try:
                send_checked(port, "S", 0, write_lock)
                send_checked(port, "T", 0, write_lock)
                send_checked(port, "G", 0, write_lock)
            except Exception:
                tx(port, "S", 0, write_lock)
            heartbeat_stop.set()
            heartbeat_thread.join(timeout=1.0)
    path = Path(output)
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", newline="", encoding="utf-8") as stream:
        fields = ["host_monotonic_s"] + LEARNING_FIELDS
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    print(f"Saved {len(rows)} learning frames to {path.resolve()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("output")
    parser.add_argument("--seconds-per-step", type=float, default=4.0)
    parser.add_argument("--no-position", action="store_true")
    parser.add_argument("--auto-calibrate", action="store_true",
                        help="Run C0 automatically when the encoder is not calibrated")
    args = parser.parse_args()
    if args.seconds_per_step <= 0:
        parser.error("--seconds-per-step must be positive")
    collect(args.port, args.output, args.seconds_per_step, not args.no_position,
            args.auto_calibrate)


if __name__ == "__main__":
    main()
