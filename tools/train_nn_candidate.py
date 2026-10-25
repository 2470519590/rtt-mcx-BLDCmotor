"""Offline v2 candidate. Does not overwrite the deployed model or firmware.

The PD teacher is a demonstration target, not measured optimal motor voltage.
Features and derivative state require a matching firmware implementation.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from nn_model import load_learning_csv, standardize, train_mlp

FEATURES = ["target_speed_rad_s", "measured_speed_rad_s", "speed_error_rad_s",
            "filtered_error_derivative_rad_s2", "position_error_rad", "base_uq"]


def prepare(rows, raw, tau=0.05):
    """Reset causal derivative at commands, mode changes and telemetry gaps."""
    x = raw.copy()
    groups = np.zeros(len(rows), dtype=int)
    derivative = 0.0
    for i, row in enumerate(rows):
        mode = int(row["mode"])
        command = row["target_position_mrad"] if mode == 8 else row["target_speed_mrad_s"]
        key = (row.get("trial", ""), mode, command)
        dt = (int(row["tick_ms"]) - int(rows[i-1]["tick_ms"])) / 1000 if i else 0
        boundary = i == 0 or key != previous_key or not 0 < dt <= .05
        if boundary:
            derivative = 0.0
        else:
            rate = np.clip((raw[i, 2] - raw[i-1, 2]) / dt, -100., 100.)
            derivative += dt / (tau + dt) * (rate - derivative)
        if i:
            groups[i] = groups[i-1] + int(boundary)
        x[i, 3] = derivative
        x[i, 4] = raw[i, 3] - raw[i, 4] if mode == 8 else 0.0
        previous_key = key
    y = np.clip(.012 * x[:, 2] + .0008 * x[:, 3], -.15, .15)
    return x, y, groups


def split_groups(groups, seed=7):
    unique = np.unique(groups)
    if len(unique) < 5:
        raise ValueError("need at least five complete command segments for train/validation/test")
    shuffled = np.random.default_rng(seed).permutation(unique)
    count = max(1, int(len(unique) * .2))
    return tuple(np.isin(groups, ids) for ids in
                 (shuffled[2*count:], shuffled[:count], shuffled[count:2*count]))


def predict(model, x):
    return np.clip(np.maximum(x @ model["w1"].T + model["b1"], 0) @
                   model["w2"] + model["b2"], -.15, .15)


def metrics(y, pred):
    mse = float(np.mean((y - pred)**2))
    zero_mse = float(np.mean(y**2))
    variance = float(np.var(y))
    return {"rows": len(y), "rmse_v": float(np.sqrt(mse)),
            "zero_rmse_v": float(np.sqrt(zero_mse)),
            "r2": 1 - mse / variance if variance > 1e-12 else None,
            "saturation_fraction": float(np.mean(np.abs(pred) >= .14999))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input")
    parser.add_argument("--output", required=True)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--evaluate", nargs="*", default=[],
                        help="Independent runs used only after model selection")
    args = parser.parse_args()
    rows, raw = load_learning_csv(args.input)
    x, y, groups = prepare(rows, raw)
    train, validation, test = split_groups(groups, args.seed)
    _, mean, std = standardize(x[train])
    normalized = np.clip((x - mean) / std, -8., 8.)
    # Unit-scale targets avoid tiny voltage labels being overwhelmed by initial weights.
    scale = max(float(np.std(y[train])), .001)
    candidates = []
    for epochs in (200, 800, 2000):
        model = train_mlp(normalized[train], y[train] / scale,
                          epochs=epochs, seed=args.seed)
        model["w2"] *= scale
        model["b2"] *= scale
        score = metrics(y[validation], predict(model, normalized[validation]))["rmse_v"]
        candidates.append((score, epochs, model))
    _, epochs, model = min(candidates, key=lambda item: item[0])
    report = {"source": str(Path(args.input).resolve()), "schema": "residual_v2",
              "features": FEATURES, "teacher": "0.012*speed_error + 0.0008*filtered_derivative; clipped +/-0.15V",
              "derivative_tau_s": .05, "selected_epochs": epochs, "seed": args.seed,
              "warning": "Teacher imitation only; no claim of motor tracking improvement. Firmware v1 incompatible.",
              "splits": {}}
    for name, mask in zip(("train", "validation", "test"), (train, validation, test)):
        report["splits"][name] = {**metrics(y[mask], predict(model, normalized[mask])),
                                      "groups": np.unique(groups[mask]).tolist()}
    report["independent_runs"] = {}
    for path in args.evaluate:
        if Path(path).resolve() == Path(args.input).resolve():
            raise ValueError("independent evaluation must use another source file")
        eval_rows, eval_raw = load_learning_csv(path)
        eval_x, eval_y, _ = prepare(eval_rows, eval_raw)
        eval_norm = (eval_x - mean) / std
        report["independent_runs"][str(path)] = {
            **metrics(eval_y, predict(model, np.clip(eval_norm, -8, 8))),
            "input_clip_fraction": float(np.mean(np.any(np.abs(eval_norm) > 8, axis=1))),
            "by_mode": {mode: metrics(eval_y[mask], predict(model, np.clip(eval_norm[mask], -8, 8)))
                        for mode in sorted({r["mode"] for r in eval_rows})
                        for mask in [np.array([r["mode"] == mode for r in eval_rows])]}}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output, schema="residual_v2", features=FEATURES,
             input_mean=mean, input_std=std, hidden_weights=model["w1"],
             hidden_bias=model["b1"], output_weights=model["w2"],
             output_bias=model["b2"], output_limit=np.float32(.15))
    output.with_suffix(".json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), constrained_layout=True)
    prediction = predict(model, normalized[test])
    axes[0].plot(y[test], label="PD teacher", linewidth=1)
    axes[0].plot(prediction, label="held-out NN", linewidth=.8, alpha=.8)
    axes[0].set(ylabel="Residual voltage (V)", xlabel="Held-out samples (segments concatenated)")
    axes[0].legend()
    names = ["train", "validation", "test"]
    axes[1].bar(np.arange(3)-.18, [report["splits"][n]["zero_rmse_v"] for n in names],
                width=.36, label="Zero predictor")
    axes[1].bar(np.arange(3)+.18, [report["splits"][n]["rmse_v"] for n in names],
                width=.36, label="Candidate NN")
    axes[1].set(xticks=np.arange(3), xticklabels=names, ylabel="Teacher RMSE (V)")
    axes[1].legend()
    fig.savefig(output.with_suffix(".png"), dpi=150)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
