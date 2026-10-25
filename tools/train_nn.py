"""Train the demo residual MLP from firmware L-frame CSV data."""
import argparse
import json
from pathlib import Path
import numpy as np

from nn_model import load_learning_csv, standardize, teacher_target, train_mlp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input")
    parser.add_argument("--output", default="nn_speed_model.npz")
    parser.add_argument("--epochs", type=int, default=800)
    parser.add_argument("--teacher-kp", type=float, default=0.012)
    parser.add_argument("--teacher-kd", type=float, default=0.0008)
    parser.add_argument("--teacher-limit", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    if args.epochs < 1:
        parser.error("--epochs must be positive")
    rows, raw = load_learning_csv(args.input)
    x, mean, std = standardize(raw)
    y = teacher_target(rows, raw, args.teacher_kp, args.teacher_kd, args.teacher_limit)
    model = train_mlp(x, y, epochs=args.epochs, seed=args.seed)
    output = Path(args.output)
    np.savez(output, input_mean=mean, input_std=std,
             hidden_weights=model["w1"], hidden_bias=model["b1"],
             output_weights=model["w2"], output_bias=model["b2"],
             output_limit=np.float32(args.teacher_limit))
    metrics = {"rows": len(rows), "mse": model["mse"],
               "target_rms": float(np.sqrt(np.mean(y * y))),
               "teacher_kp": args.teacher_kp, "teacher_kd": args.teacher_kd,
               "teacher_limit": args.teacher_limit}
    output.with_suffix(".json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print(json.dumps({"model": str(output.resolve()), **metrics}, indent=2))


if __name__ == "__main__":
    main()
