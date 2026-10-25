"""Plot an automated position step CSV as position, speed and Uq traces."""
import argparse
import csv
from pathlib import Path


def load_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", default="position_step_delayzero_kp8.csv")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    source = Path(args.source).resolve()
    output = Path(args.output).resolve() if args.output else source.with_name(source.stem + ".svg")
    rows = load_rows(source)
    if not rows:
        raise SystemExit("CSV has no rows")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [float(row["time_s"]) for row in rows]
    # The calibration test records raw AS5600 direction for position, while
    # the controller reports speed in calibrated direction. The tested motor
    # direction is negative raw angle, so invert position for one consistent
    # mechanical coordinate in the plot.
    angle = [-float(row["relative_deg"]) for row in rows]
    speed = [float(row["speed_rpm"]) for row in rows]
    target_speed = [float(row["target_speed_rpm"]) for row in rows]
    uq = [float(row["uq_v"]) for row in rows]

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True,
                             gridspec_kw={"height_ratios": [1.4, 1, 1]})
    ax = axes[0]
    ax.plot(t, angle, color="#1769aa", linewidth=1.1, label="measured position")
    ax.axhline(10.0, color="#c44e52", linestyle="--", linewidth=1,
               label="+10 deg command")
    ax.axhline(0.0, color="#55a868", linestyle=":", linewidth=1,
               label="zero command")
    ax.set_ylabel("Position [deg]")
    ax.set_title("Position-loop step test: delay-zero calibration, poskp=8")
    ax.legend(loc="best", ncol=3, fontsize=9)
    ax.grid(True, alpha=.3)

    ax = axes[1]
    ax.plot(t, speed, color="#dd8452", linewidth=1.0, label="measured speed")
    ax.plot(t, target_speed, color="#937860", linestyle="--", linewidth=1,
            label="speed target from position loop")
    ax.set_ylabel("Speed [RPM]")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=.3)

    ax = axes[2]
    ax.plot(t, uq, color="#8172b2", linewidth=1.0, label="Uq")
    ax.axhline(1.0, color="#c44e52", linestyle=":", linewidth=.8)
    ax.axhline(-1.0, color="#c44e52", linestyle=":", linewidth=.8)
    ax.set_ylabel("Uq [V]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=.3)

    for phase, color in (("plus", "#c44e52"), ("return", "#55a868")):
        phase_rows = [row for row in rows if row["phase"] == phase]
        if phase_rows:
            x = float(phase_rows[0]["time_s"])
            for axis in axes:
                axis.axvline(x, color=color, linestyle="--", linewidth=.7, alpha=.7)

    fig.tight_layout()
    fig.savefig(output, format=output.suffix.lstrip(".") or "svg", dpi=160)
    if output.suffix.lower() != ".png":
        png = output.with_suffix(".png")
        fig.savefig(png, format="png", dpi=160)
        print(f"PNG saved: {png}")
    plt.close(fig)
    print(f"Plot saved: {output}")


if __name__ == "__main__":
    main()
