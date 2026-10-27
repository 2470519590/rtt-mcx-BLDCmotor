"""Bounded real-board position comparison; reuses calibration, captures raw UART."""
import argparse
from collections import Counter
import csv
import json
import math
import threading
import time
from pathlib import Path

import numpy as np

from compare_position_ff import summarize
from motor_port import resolve_port
from nn_dataset import LEARNING_FIELDS, parse_learning
from serial_capture import CapturedSerial
from uart_evidence import replay_capture, summarize as uart_summary


CONFIGS = {
    "demo": (8, 284, 4000, 120),
    "damped": (60, 80, 6000, 120),
    "p_only": (80, 0, 6000, 120),
    "soft": (40, 0, 8000, 120),
    "strong": (100, 20, 8000, 120),
    "highgain": (160, 284, 8000, 120),
    "highki": (80, 650, 6000, 120),
    "stiff": (160, 300, 15000, 120),
    "balanced": (120, 400, 12000, 90),
    "stiff_slow": (160, 300, 15000, 80),
    "smooth_balanced": (120, 400, 12000, 90),
    "smooth_stiff": (160, 300, 15000, 80),
    "smooth_soft": (80, 400, 10000, 90),
    "smooth_fast": (120, 650, 15000, 90),
    "smooth_fastsoft": (100, 650, 15000, 90),
    "smooth_quiet": (40, 650, 15000, 90),
    "smooth_quiet2": (60, 650, 15000, 90),
}


def capture_summary(root):
    replay = replay_capture(root)
    result = uart_summary(replay)
    result.pop("other_lines")
    learning = [parse_learning(line["data"].decode("ascii"))
                for line in replay.lines if line["data"].startswith(b"L,")]
    result.update(learning_frames=len(learning),
                  learning_seq_steps=dict(Counter(
                      (b["sequence"] - a["sequence"]) & 0xffffffff
                      for a, b in zip(learning, learning[1:]))),
                  learning_tick_steps=dict(Counter(
                      (b["tick_ms"] - a["tick_ms"]) & 0xffffffff
                      for a, b in zip(learning, learning[1:]))))
    combined = [tuple(map(int, line["data"].split(b",")[1:3]))
                for line in replay.lines if line["data"].startswith((b"T,", b"L,"))
                and len(line["data"].split(b",")) in (12, 19)]
    result.update(combined_frames=len(combined),
                  combined_seq_steps=dict(Counter(
                      (b[0] - a[0]) & 0xffffffff for a, b in zip(combined, combined[1:]))),
                  combined_tick_steps=dict(Counter(
                      (b[1] - a[1]) & 0xffffffff for a, b in zip(combined, combined[1:]))))
    return result


def metrics(rows, target):
    result = summarize(rows, target)
    ticks = np.array([row["tick_ms"] for row in rows])
    tail = ticks >= ticks[-1] - 500
    position = np.array([row["position_mrad"] for row in rows]) * 180 / (1000 * math.pi)
    speed = np.array([row["speed_mrad_s"] for row in rows]) * 60 / (1000 * 2 * math.pi)
    voltage = np.array([row["uq_mv"] for row in rows]) / 1000
    result.update(tail_p2p_deg=float(np.ptp(position[tail])),
                  tail_speed_rms_rpm=float(np.sqrt(np.mean(speed[tail] ** 2))),
                  tail_voltage_p2p_v=float(np.ptp(voltage[tail])),
                  peak_speed_rpm=float(np.max(np.abs(speed))),
                  peak_abs_voltage_v=float(np.max(np.abs(voltage))),
                  max_voltage_step_v=float(np.max(np.abs(np.diff(voltage)))),
                  end_error_deg=float(np.mean(position[-3:]) - target * 180 / (1000 * math.pi)),
                  frames=len(rows))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--configs", nargs="+", choices=CONFIGS,
                        default=["demo", "stiff", "balanced", "stiff_slow"])
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--small-only", action="store_true")
    parser.add_argument("--hold", type=float, default=2)
    parser.add_argument("--return-origin", action="store_true",
                        help="Return to each arm's origin after large steps without redefining zero")
    parser.add_argument("--analyze", action="store_true", help="Rebuild report from CSV, no hardware")
    parser.add_argument("--align-if-needed", action="store_true",
                        help="One alignment only if the flashed firmware reports calibrated=0")
    args = parser.parse_args()
    output = Path(args.output).resolve()
    if args.analyze:
        with output.with_suffix(".csv").open(encoding="utf-8") as stream:
            rows = [{key: value if key in ("config", "phase") else int(value)
                     for key, value in row.items()} for row in csv.DictReader(stream)]
        trials = []
        for key in dict.fromkeys((row["config"], row["repeat"], row["phase"]) for row in rows):
            samples = [row for row in rows
                       if (row["config"], row["repeat"], row["phase"]) == key]
            trials.append(dict(config=key[0], repeat=key[1], phase=key[2],
                               **metrics(samples, samples[0]["commanded_position_mrad"])))
        output.with_suffix(".analysis.json").write_text(json.dumps(dict(
            analysis_only=True, configs=CONFIGS, trials=trials,
            uart=capture_summary(output.with_name(output.name + "_uart"))), indent=2),
            encoding="utf-8")
        plot(output, rows)
        return
    if output.with_suffix(".json").exists() or output.with_suffix(".csv").exists():
        raise FileExistsError(output)
    if not 1 <= args.repeats <= 4 or not 1 <= args.hold <= 8:
        raise ValueError("repeats must be 1..4, hold must be 1..8 s")
    port = CapturedSerial(resolve_port(args.port), output.with_name(output.name + "_uart"))
    done = threading.Event()
    heartbeat_errors, rows, trials, events = [], [], [], []
    failure, final = None, None
    calibration_performed = False

    def heartbeat():
        while not done.wait(.15):
            try:
                port.write(b"H0\n")
            except Exception as exc:
                heartbeat_errors.append(repr(exc))
                done.set()

    def request(op, value=0, allow_uncalibrated=False):
        begin = time.perf_counter_ns()
        port.write(f"{op}{value}\n".encode("ascii"))
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            fields = port.readline().decode("ascii", "replace").strip().split(",")
            if port.last_read_end_ns < begin:
                continue
            if op == "c" and len(fields) == 6 and fields[0] == "POS":
                _, result, position, sensor, calibrated, fault = fields
                if allow_uncalibrated and (int(result), int(sensor), int(calibrated), int(fault)) == (0, 1, 0, 0):
                    return None
                if (int(result), int(sensor), int(calibrated), int(fault)) != (0, 1, 1, 0):
                    raise RuntimeError("Position query not ready: " + ",".join(fields))
                return int(position)
            if len(fields) >= 3 and fields[:2] == ["ACK", op]:
                events.append(dict(command=f"{op}{value}", reply=",".join(fields),
                                   elapsed_ms=(time.perf_counter_ns() - begin) / 1e6))
                if int(fields[2]) != 0:
                    raise RuntimeError("Command rejected: " + ",".join(fields))
                return 0
        raise TimeoutError(f"No reply to {op}{value}")

    def capture(seconds, config, repeat, phase, target, origin):
        end = time.monotonic() + seconds
        samples = []
        while time.monotonic() < end:
            if heartbeat_errors:
                raise RuntimeError(str(heartbeat_errors))
            line = port.readline().decode("ascii", "replace").strip()
            if not line.startswith("L,"):
                continue
            row = parse_learning(line)
            if row["fault"] or not row["sensor_ok"] or not row["calibrated"]:
                raise RuntimeError("Feedback invalid: " + line)
            if row["mode"] != 8 or abs(row["target_position_mrad"] - target) > 1:
                continue
            row.update(config=config, repeat=repeat, phase=phase,
                       commanded_position_mrad=target, origin_mrad=origin,
                       host_read_end_ns=port.last_read_end_ns)
            rows.append(row)
            samples.append(row)
        if len(samples) < 10:
            raise RuntimeError(f"Insufficient position telemetry: {config}/{phase}")
        result = dict(config=config, repeat=repeat, phase=phase, target_mrad=target,
                      **metrics(samples, target))
        trials.append(result)
        print(json.dumps(result), flush=True)
        return samples

    port.open()
    keeper = threading.Thread(target=heartbeat, daemon=True)
    keeper.start()
    try:
        request("S")
        request("T", 0)
        origin = request("c", allow_uncalibrated=args.align_if_needed)
        if origin is None:
            calibration_performed = True
            request("C")
            end = time.monotonic() + 4
            while time.monotonic() < end:
                port.readline()
            origin = request("c")
        if abs(origin) > 90000:
            raise RuntimeError("Origin outside test range; no calibration or reset performed")
        request("G", 0)
        request("V", 1000)
        request("T", 1)
        for repeat in range(args.repeats):
            # Alternate ordering in the second repeat, without recalibrating.
            order = args.configs if repeat % 2 == 0 else list(reversed(args.configs))
            for config in order:
                request("S")
                kp, ki, position_kp, rpm = CONFIGS[config]
                for op, value in (("P", kp), ("Q", ki), ("Z", position_kp), ("Y", rpm),
                                  ("t", 500 if config.startswith("smooth_") else 0)):
                    request(op, value)
                # Start each arm from its current position, not a forced long return.
                origin = request("c")
                steps = [("small_plus", 10, args.hold), ("small_return", 0, args.hold),
                         ("small_minus", -10, args.hold), ("small_return2", 0, args.hold)]
                if not args.small_only:
                    steps += [("large_120", 120, .5), ("large_240", 240, .5),
                              ("large_360", 360, 2)]
                    if args.return_origin:
                        steps += [("large_return", 0, 3)]
                for phase, degrees, seconds in steps:
                    target = origin + round(math.radians(degrees) * 1000)
                    request("X", target)
                    capture(seconds, config, repeat, phase, target, origin)
                request("S")
        final = request("c")
    except Exception as exc:
        failure = repr(exc)
        print("FAILED", failure, flush=True)
    finally:
        for op, value in (("S", 0), ("T", 0), ("G", 0)):
            try:
                request(op, value)
            except Exception as exc:
                events.append(dict(cleanup=f"{op}{value}", error=repr(exc)))
        done.set()
        keeper.join(1)
        port.close()
        if rows:
            with output.with_suffix(".csv").open("x", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["config", "repeat", "phase",
                    "commanded_position_mrad", "origin_mrad", "host_read_end_ns"] + LEARNING_FIELDS)
                writer.writeheader()
                writer.writerows(rows)
        result = dict(failure=failure, final_position_mrad=final, configs=CONFIGS,
                      calibration_performed=calibration_performed,
                      trials=trials, commands=events,
                      uart=capture_summary(port.root))
        output.with_suffix(".json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        if rows:
            plot(output, rows)
    if failure:
        raise SystemExit(1)


def plot(output, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = list(dict.fromkeys(row["config"] for row in rows))
    fig, axes = plt.subplots(len(names), 3, figsize=(15, 2.6 * len(names)),
                             squeeze=False, constrained_layout=True)
    for index, name in enumerate(names):
        sample = [row for row in rows if row["config"] == name and row["repeat"] == 0]
        tick = np.array([row["tick_ms"] for row in sample])
        seconds = (tick - tick[0]) / 1000
        origin = sample[0]["origin_mrad"]
        axes[index, 0].plot(seconds, [(row["position_mrad"] - origin) * 180 / (1000 * math.pi)
                                    for row in sample], label="measured")
        axes[index, 0].plot(seconds, [(row["commanded_position_mrad"] - origin) * 180 / (1000 * math.pi)
                                    for row in sample], "--", label="target")
        axes[index, 1].plot(seconds, [row["speed_mrad_s"] * 60 / (1000 * 2 * math.pi)
                                    for row in sample])
        axes[index, 2].plot(seconds, [row["uq_mv"] / 1000 for row in sample])
        for axis, title in zip(axes[index], ("Position (deg)", "Speed (RPM)", "Uq (V)")):
            axis.set_title(name + " / " + title)
            axis.grid(alpha=.25)
            axis.set_xlabel("Device time (s)")
    axes[0, 0].legend()
    fig.savefig(output.with_suffix(".png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
