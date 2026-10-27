"""Offline, measured position-control comparison; never opens hardware."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from tune_position import capture_summary


def comparison(trials):
    result = {}
    for name in ("demo", "smooth_quiet"):
        small = [row for row in trials
                 if row["config"] == name and row["phase"].startswith("small")]
        large = [row for row in trials
                 if row["config"] == name and row["phase"] == "large_360"]
        if not small or not large:
            raise ValueError(f"Missing comparison trials: {name}")
        result[name] = dict(
            small_segments=len(small), final_large_segments=len(large),
            small_rmse_deg=float(np.mean([r["rmse_deg"] for r in small])),
            small_tail_mae_deg=float(np.mean([r["tail_mae_deg"] for r in small])),
            small_tail_p2p_deg=float(np.mean([r["tail_p2p_deg"] for r in small])),
            small_tail_speed_rms_rpm=float(np.mean(
                [r["tail_speed_rms_rpm"] for r in small])),
            final_large_tail_mae_deg=float(np.mean([r["tail_mae_deg"] for r in large])),
            clipping_samples=sum(r["clipped"] for r in small + large),
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="Captured tune_position output stem")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    source, output = Path(args.source), Path(args.output)
    if any(output.with_suffix(ext).exists() for ext in (".json", ".png")):
        raise FileExistsError(output)
    record = json.loads(source.with_suffix(".json").read_text(encoding="utf-8"))
    if record["failure"]:
        raise ValueError("Source run failed: " + record["failure"])
    with source.with_suffix(".csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    summary = comparison(record["trials"])
    uart = capture_summary(source.with_name(source.name + "_uart"))
    report = dict(
        source=str(source.resolve()), summary=summary, uart=uart,
        calibration_performed=record["calibration_performed"],
        interpretation="Traditional controller and trajectory improvement, not NN gain.",
        scope="Two repeats; tail MAE uses last 0.8 s; tail P2P uses last 0.5 s. "
              "100 Hz observations do not establish higher-frequency electrical behavior. "
              "Slower shaped starts may increase large-step error relative to an instantaneous target.",
    )
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), constrained_layout=True)
    for name, color, label in (("demo", "#8b96a7", "Previous demo"),
                               ("smooth_quiet", "#087f8c", "Smooth position control")):
        for col, prefix in enumerate(("small", "large")):
            samples = [row for row in rows if row["config"] == name and
                       int(row["repeat"]) == 0 and row["phase"].startswith(prefix)
                       and row["phase"] != "large_return"]
            tick = np.array([int(row["tick_ms"]) for row in samples])
            t = (tick - tick[0]) / 1000
            origin = int(samples[0]["origin_mrad"])
            position = [(int(r["position_mrad"]) - origin) * 180 / (1000 * math.pi)
                        for r in samples]
            target = [(int(r["commanded_position_mrad"]) - origin) * 180 / (1000 * math.pi)
                      for r in samples]
            speed = [int(r["speed_mrad_s"]) * 60 / (1000 * 2 * math.pi) for r in samples]
            voltage = [int(r["uq_mv"]) / 1000 for r in samples]
            for axis, values in zip(axes[:, col], (position, speed, voltage)):
                axis.plot(t, values, color=color, lw=1.4, label=label)
            if name == "smooth_quiet":
                axes[0, col].step(t, target, where="post", color="#d59a35", ls="--",
                                  lw=1, label="Commanded target")
    for col, title in enumerate(("10 deg bidirectional steps", "Three 120 deg commands")):
        axes[0, col].set_title(title)
        for row, ylabel in enumerate(("Position (deg)", "Speed (RPM)", "Uq (V)")):
            axes[row, col].set_ylabel(ylabel)
            axes[row, col].grid(alpha=.2)
        axes[2, col].set_xlabel("Device time (s)")
    axes[0, 0].legend(fontsize=8, loc="lower left")
    old, new = summary["demo"], summary["smooth_quiet"]
    repeats = len({int(row["repeat"]) for row in rows})
    fig.suptitle("Real-board position control | same calibration, NN off\n"
                 f"{repeats}-repeat means: small-step tail MAE "
                 f"{old['small_tail_mae_deg']:.3f} -> {new['small_tail_mae_deg']:.3f} deg; "
                 f"tail motion P2P {old['small_tail_p2p_deg']:.3f} -> "
                 f"{new['small_tail_p2p_deg']:.3f} deg\n"
                 "Plots show repeat 1; MAE: last 0.8 s, P2P: last 0.5 s", fontsize=12)
    fig.savefig(output.with_suffix(".png"), dpi=160)
    plt.close(fig)
    output.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print("Combined T/L sequence steps:", uart["combined_seq_steps"])


if __name__ == "__main__":
    main()
