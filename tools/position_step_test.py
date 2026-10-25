"""Automated position-loop test using raw AS5600 angle deltas from the calibrated zero."""
import argparse
import csv
import math
import time
from pathlib import Path

import serial


def parse_t(line):
    if not line.startswith("T,"):
        return None
    p = line.split(",")
    if len(p) != 12:
        return None
    try:
        v = [int(x) for x in p[1:]]
    except ValueError:
        return None
    return {"seq": v[0], "tick": v[1], "raw": v[2], "speed": v[3],
            "sensor": v[4], "mode": v[5], "cal": v[6], "target_speed": v[7],
            "uq": v[8], "fault": v[9]}


def send(port, op, value=0):
    for b in f"{op}{value}\n".encode("ascii"):
        port.write(bytes([b]))
        time.sleep(.005)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        if line.startswith("ACK,"):
            p = line.split(",")
            if len(p) >= 3 and p[1] == op:
                if int(p[2]) != 0:
                    raise RuntimeError(f"Rejected {op}{value}: {line}")
                return
    raise TimeoutError(f"No ACK for {op}{value}")


def collect(port, seconds, rows, phase, zero_raw=None):
    end = time.monotonic() + seconds
    next_hb = time.monotonic()
    while time.monotonic() < end:
        if time.monotonic() >= next_hb:
            port.write(b"H0\n")
            next_hb = time.monotonic() + .25
        row = parse_t(port.readline().decode("ascii", errors="replace").strip())
        if row is None:
            continue
        row["host_time"] = time.monotonic()
        row["phase"] = phase
        if zero_raw is not None:
            d = (row["raw"] - zero_raw + 2048) % 4096 - 2048
            row["relative_deg"] = d * 360.0 / 4096.0
        else:
            row["relative_deg"] = 0.0
        rows.append(row)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port", nargs="?", default="COM32")
    ap.add_argument("--output", default="position_step_test.csv")
    ap.add_argument("--angle", type=float, default=10.0)
    ap.add_argument("--kp", type=float, default=4.0)
    ap.add_argument("--max-rpm", type=int, default=60)
    args = ap.parse_args()
    out = Path(args.output).resolve()
    if out.exists():
        raise SystemExit(f"Output exists: {out}")
    rows = []
    with serial.Serial(args.port, 115200, timeout=.05, write_timeout=.5) as port:
        port.dtr = False
        port.rts = False
        port.reset_input_buffer()
        send(port, "S")
        # Wait for a fresh valid stopped telemetry frame before calibration.
        deadline = time.monotonic() + 5
        next_hb = time.monotonic()
        while time.monotonic() < deadline:
            if time.monotonic() >= next_hb:
                port.write(b"H0\n")
                next_hb = time.monotonic() + .25
            row = parse_t(port.readline().decode("ascii", errors="replace").strip())
            if row and row["sensor"]:
                break
        else:
            raise TimeoutError("No AS5600 telemetry before calibration")
        send(port, "C")
        deadline = time.monotonic() + 10
        zero_raw = None
        calibrated_at = None
        while time.monotonic() < deadline:
            if time.monotonic() >= locals().get("next_hb", 0):
                port.write(b"H0\n")
                next_hb = time.monotonic() + .25
            row = parse_t(port.readline().decode("ascii", errors="replace").strip())
            if row is None:
                continue
            row["host_time"] = time.monotonic()
            row["phase"] = "calibration"
            row["relative_deg"] = 0.0
            rows.append(row)
            if row["mode"] == 0 and row["cal"] == 1 and row["fault"] == 0:
                if calibrated_at is None:
                    calibrated_at = row["host_time"]
                elif row["host_time"] - calibrated_at >= 1.0:
                    zero_raw = row["raw"]
                    break
            else:
                calibrated_at = None
            if row["fault"]:
                raise RuntimeError(f"Calibration failed: {row}")
        if zero_raw is None:
            raise TimeoutError("Calibration did not complete")
        send(port, "Z", round(args.kp * 1000))
        send(port, "Y", args.max_rpm)
        collect(port, 1.0, rows, "zero", zero_raw)
        target_mrad = round(args.angle * math.pi / 180.0 * 1000.0)
        send(port, "X", target_mrad)
        collect(port, 4.0, rows, "plus", zero_raw)
        send(port, "X", 0)
        collect(port, 4.0, rows, "return", zero_raw)
        send(port, "S")
        collect(port, 1.0, rows, "stop", zero_raw)
    t0 = min(r["host_time"] for r in rows)
    with out.open("w", newline="", encoding="utf-8") as f:
        fields = ["time_s", "phase", "raw", "relative_deg", "speed_rpm",
                  "target_speed_rpm", "uq_v", "mode", "sensor_ok", "calibrated", "fault"]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({"time_s": r["host_time"] - t0, "phase": r["phase"],
                        "raw": r["raw"], "relative_deg": r["relative_deg"],
                        "speed_rpm": r["speed"] * 60 / (1000 * 2 * math.pi),
                        "target_speed_rpm": r["target_speed"] * 60 / (1000 * 2 * math.pi),
                        "uq_v": r["uq"] / 1000, "mode": r["mode"],
                        "sensor_ok": r["sensor"], "calibrated": r["cal"], "fault": r["fault"]})
    for phase in ("plus", "return"):
        rs = [r for r in rows if r["phase"] == phase]
        tail = rs[-50:]
        if tail:
            print(f"{phase}: final_angle={sum(r['relative_deg'] for r in tail)/len(tail):.2f} deg, "
                  f"speed={sum(r['speed'] for r in tail)/len(tail)*60/(1000*2*math.pi):.2f} RPM, "
                  f"faults={sorted({r['fault'] for r in rs})}")
    print(f"Recorded {len(rows)} frames: {out}")


if __name__ == "__main__":
    main()
