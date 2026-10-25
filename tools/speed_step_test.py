"""Automated real-board speed-loop step test over the external USB-UART."""
import argparse
import csv
import math
import time
from pathlib import Path

import serial


def send_command(port, op, value=0, timeout=2.0):
    for byte in f"{op}{value}\n".encode("ascii"):
        port.write(bytes([byte]))
        time.sleep(.005)
    time.sleep(.02)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue
        if line.startswith("ACK,"):
            parts = line.split(",")
            if len(parts) >= 3 and parts[1] == op:
                result = int(parts[2])
                if result != 0:
                    raise RuntimeError(f"Firmware rejected {op}{value}: {line}")
                return
    raise TimeoutError(f"No ACK for {op}{value}")


def parse_telemetry(line):
    if not line.startswith("T,"):
        return None
    parts = line.split(",")
    if len(parts) != 12:
        return None
    try:
        values = [int(x) for x in parts[1:]]
    except ValueError:
        return None
    return {
        "sequence": values[0], "tick_ms": values[1], "raw_angle": values[2],
        "speed_mrad_s": values[3], "sensor_ok": values[4], "mode": values[5],
        "calibrated": values[6], "target_mrad_s": values[7], "uq_mv": values[8],
        "fault": values[9], "calibration_mrad": values[10],
    }


def drain(port, seconds, rows, phase, target, heartbeat=True):
    end = time.monotonic() + seconds
    next_heartbeat = time.monotonic()
    while time.monotonic() < end:
        if heartbeat and time.monotonic() >= next_heartbeat:
            port.write(b"H0\n")
            next_heartbeat = time.monotonic() + .25
        line = port.readline().decode("ascii", errors="replace").strip()
        row = parse_telemetry(line)
        if row is not None:
            row["host_time_s"] = time.monotonic()
            row["phase"] = phase
            row["command_rpm"] = target
            rows.append(row)


def wait_calibrated(port, rows):
    port.reset_input_buffer()
    send_command(port, "S")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        row = parse_telemetry(line)
        if row is not None:
            row["host_time_s"] = time.monotonic()
            row["phase"] = "precalibration"
            row["command_rpm"] = 0
            rows.append(row)
            if row["sensor_ok"] == 1 and row["mode"] in (0, 3):
                break
    else:
        raise TimeoutError("No fresh sensor telemetry before calibration")
    send_command(port, "C")
    deadline = time.monotonic() + 8.0
    next_heartbeat = time.monotonic()
    while time.monotonic() < deadline:
        if time.monotonic() >= next_heartbeat:
            port.write(b"H0\n")
            next_heartbeat = time.monotonic() + .25
        line = port.readline().decode("ascii", errors="replace").strip()
        row = parse_telemetry(line)
        if row is None:
            continue
        row["host_time_s"] = time.monotonic()
        row["phase"] = "calibration"
        row["command_rpm"] = 0
        rows.append(row)
        if row["mode"] == 0 and row["calibrated"] == 1 and row["fault"] == 0:
            return
        if row["mode"] == 3 or row["fault"]:
            raise RuntimeError(f"Calibration failed: {row}")
    raise TimeoutError("Calibration did not complete within 8 seconds")


def to_rpm(mrad_s):
    return mrad_s * 60.0 / (1000.0 * 2.0 * math.pi)


def write_csv(path, rows, t0):
    fields = ["time_s", "phase", "command_rpm", "speed_rpm", "target_rpm",
              "uq_v", "mode", "sensor_ok", "calibrated", "fault", "sequence"]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                "time_s": row["host_time_s"] - t0,
                "phase": row["phase"],
                "command_rpm": row["command_rpm"],
                "speed_rpm": to_rpm(row["speed_mrad_s"]),
                "target_rpm": to_rpm(row["target_mrad_s"]),
                "uq_v": row["uq_mv"] / 1000.0,
                "mode": row["mode"], "sensor_ok": row["sensor_ok"],
                "calibrated": row["calibrated"], "fault": row["fault"],
                "sequence": row["sequence"],
            })


def summarize(rows):
    import numpy as np
    results = []
    for target in sorted({r["command_rpm"] for r in rows if r["phase"] == "step"}):
        samples = [r for r in rows if r["phase"] == "step" and r["command_rpm"] == target]
        if len(samples) < 10:
            continue
        t = np.array([r["host_time_s"] for r in samples])
        y = np.array([to_rpm(r["speed_mrad_s"]) for r in samples])
        t -= t[0]
        final = float(np.median(y[int(len(y) * .8):]))
        peak = float(np.max(y))
        low = float(np.min(y))
        error = target - final
        if target > 0:
            crossing = np.flatnonzero(y >= .9 * target)
            rise = float(t[crossing[0]]) if crossing.size else float("nan")
            overshoot = max(0.0, (peak - target) / target * 100.0)
            band = max(2.0, .05 * target)
            outside = np.flatnonzero(np.abs(y - target) > band)
            settling = float(t[outside[-1] + 1]) if outside.size and outside[-1] + 1 < len(t) else 0.0
        else:
            rise, overshoot, settling = float("nan"), float("nan"), float("nan")
        results.append({"target": target, "samples": len(samples), "final": final,
                        "steady_error": error, "peak": peak, "min": low,
                        "rise_90_s": rise, "overshoot_pct": overshoot,
                        "settling_s": settling,
                        "max_abs_uq_v": max(abs(r["uq_mv"]) for r in samples) / 1000.0,
                        "faults": sorted({r["fault"] for r in samples})})
    return results


def plot(path, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t0 = min(r["host_time_s"] for r in rows)
    t = [r["host_time_s"] - t0 for r in rows]
    speed = [to_rpm(r["speed_mrad_s"]) for r in rows]
    target = [to_rpm(r["target_mrad_s"]) for r in rows]
    uq = [r["uq_mv"] / 1000.0 for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(t, speed, label="measured RPM", linewidth=1)
    axes[0].plot(t, target, label="target RPM", linewidth=1, linestyle="--")
    axes[0].set_ylabel("Speed [RPM]")
    axes[0].grid(True, alpha=.3)
    axes[0].legend()
    axes[1].plot(t, uq, label="Uq", color="tab:orange", linewidth=1)
    axes[1].set_ylabel("Uq [V]")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True, alpha=.3)
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(path, format="svg")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="COM32")
    parser.add_argument("--output", default="speed_step_test.csv")
    parser.add_argument("--voltage", type=float, default=1.0)
    parser.add_argument("--kp", type=float, default=.0078)
    parser.add_argument("--ki", type=float, default=.284)
    parser.add_argument("--no-calibrate", action="store_true")
    args = parser.parse_args()
    output = Path(args.output).resolve()
    plot_path = output.with_suffix(".svg")
    if output.exists() or plot_path.exists():
        raise SystemExit(f"Output exists; choose another --output: {output}")
    rows = []
    with serial.Serial(args.port, 115200, timeout=.05, write_timeout=.5) as port:
        port.dtr = False
        port.rts = False
        print(f"Opened {args.port}; collecting telemetry.")
        if not args.no_calibrate:
            print("Calibrating encoder...")
            wait_calibrated(port, rows)
            print("Calibration complete.")
        send_command(port, "S")
        send_command(port, "V", round(args.voltage * 1000))
        send_command(port, "P", round(args.kp * 1000))
        send_command(port, "Q", round(args.ki * 1000))
        drain(port, 1.0, rows, "settle", 0)
        t0 = time.monotonic()
        for target in (60, 120, 180):
            send_command(port, "R", target)
            drain(port, 5.0, rows, "step", target)
        send_command(port, "S")
        drain(port, 1.5, rows, "stop", 0)
    write_csv(output, rows, t0)
    plot(plot_path, rows)
    print(f"Recorded {len(rows)} telemetry frames.")
    print(f"CSV: {output}")
    print(f"Plot: {plot_path}")
    for result in summarize(rows):
        print("target={target:.0f} RPM final={final:.2f} error={steady_error:.2f} "
              "rise90={rise_90_s:.3f}s overshoot={overshoot_pct:.1f}% "
              "settling={settling_s:.3f}s max|Uq|={max_abs_uq_v:.3f}V faults={faults}".format(**result))


if __name__ == "__main__":
    main()
