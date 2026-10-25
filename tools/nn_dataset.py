"""Parse and record the firmware learning telemetry stream."""
import argparse
import csv
import time
from pathlib import Path

LEARNING_FIELDS = [
    "sequence", "tick_ms", "target_position_mrad", "position_mrad",
    "target_speed_mrad_s", "speed_mrad_s", "base_uq_mv", "nn_delta_uq_mv",
    "uq_mv", "mode", "sensor_ok", "calibrated", "fault", "clipped",
    "speed_kp_milli", "voltage_limit_mv", "nn_alpha_milli", "nn_mode",
]


def parse_learning(line):
    parts = line.strip().split(",")
    if parts[0] != "L" or len(parts) != len(LEARNING_FIELDS) + 1:
        raise ValueError("Invalid learning frame")
    try:
        values = [int(value) for value in parts[1:]]
    except ValueError as exc:
        raise ValueError("Learning frame contains non-integer data") from exc
    return dict(zip(LEARNING_FIELDS, values))


def write_csv(path, rows):
    path = Path(path)
    with path.open("x", newline="", encoding="utf-8") as file:
        fields = ["host_monotonic_s"] + LEARNING_FIELDS
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def collect(port_name, output, seconds):
    import serial
    rows = []
    with serial.Serial(port_name, 115200, timeout=.05) as port:
        port.dtr = False
        port.rts = False
        port.reset_input_buffer()
        deadline = time.monotonic() + seconds
        next_heartbeat = time.monotonic()
        while time.monotonic() < deadline:
            if time.monotonic() >= next_heartbeat:
                for byte in b"H0\n":
                    port.write(bytes([byte]))
                    time.sleep(.005)
                next_heartbeat = time.monotonic() + .25
            line = port.readline().decode("ascii", errors="replace").strip()
            if not line.startswith("L,"):
                continue
            try:
                row = parse_learning(line)
            except ValueError:
                continue
            row["host_monotonic_s"] = time.monotonic()
            rows.append(row)
    write_csv(output, rows)
    print(f"Saved {len(rows)} learning frames to {Path(output).resolve()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("output")
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    collect(args.port, args.output, args.seconds)


if __name__ == "__main__":
    main()
