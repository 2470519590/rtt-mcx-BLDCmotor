"""Small numpy-only MLP training helpers for the motor NN demo."""
from __future__ import annotations

from pathlib import Path
import csv
import numpy as np

FEATURES = [
    "target_speed_rad_s", "measured_speed_rad_s", "speed_error_rad_s",
    "target_position_rad", "measured_position_rad", "base_uq",
]


def load_learning_csv(path: str | Path):
    required = {"target_speed_mrad_s", "speed_mrad_s", "base_uq_mv",
                "target_position_mrad", "position_mrad", "sensor_ok",
                "calibrated", "fault", "clipped", "tick_ms"}
    rows = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "input is not an NN learning CSV; missing columns: " +
                ", ".join(sorted(missing)))
        for row in reader:
            try:
                if any(int(row.get(k, 0)) == 0 for k in ("sensor_ok", "calibrated")):
                    continue
                if any(int(row.get(k, 0)) != 0 for k in ("fault", "clipped")):
                    continue
                rows.append(row)
            except (TypeError, ValueError):
                continue
    if not rows:
        raise ValueError("no valid learning rows after sensor/fault filtering")
    x = np.array([
        [int(r["target_speed_mrad_s"]) / 1000.0,
         int(r["speed_mrad_s"]) / 1000.0,
         (int(r["target_speed_mrad_s"]) - int(r["speed_mrad_s"])) / 1000.0,
         int(r["target_position_mrad"]) / 1000.0,
         int(r["position_mrad"]) / 1000.0,
         int(r["base_uq_mv"]) / 1000.0] for r in rows], dtype=np.float32)
    return rows, x


def teacher_target(rows, x, kp=0.012, kd=0.0008, limit=0.15):
    """Create a deliberately conservative residual-control teacher.

    The firmware baseline remains the source of truth.  This teacher asks the
    NN to learn only a small speed-error correction, making alpha=0/shadow a
    safe comparison and alpha=0.25 a useful demo setting.
    """
    error = x[:, 2]
    derivative = np.zeros_like(error)
    ticks = np.array([int(r.get("tick_ms", 0)) for r in rows], dtype=np.float32)
    if len(error) > 1:
        dt = np.diff(ticks) / 1000.0
        derivative[1:] = np.divide(np.diff(error), np.maximum(dt, 1e-3))
        derivative = np.clip(derivative, -100.0, 100.0)
    target = kp * error + kd * derivative
    return np.clip(target, -limit, limit).astype(np.float32)


def standardize(x):
    mean = x.mean(axis=0).astype(np.float32)
    std = x.std(axis=0).astype(np.float32)
    std[std < 1e-5] = 1.0
    return (x - mean) / std, mean, std


def train_mlp(x, y, hidden=8, epochs=800, learning_rate=0.01, seed=7):
    rng = np.random.default_rng(seed)
    n, inputs = x.shape
    w1 = (rng.standard_normal((hidden, inputs)).astype(np.float32) * np.sqrt(2.0 / inputs))
    b1 = np.zeros(hidden, dtype=np.float32)
    w2 = (rng.standard_normal(hidden).astype(np.float32) * np.sqrt(2.0 / hidden))
    b2 = np.float32(0.0)
    # Full-batch Adam keeps the script deterministic and dependency-light.
    mw1 = np.zeros_like(w1); vw1 = np.zeros_like(w1)
    mb1 = np.zeros_like(b1); vb1 = np.zeros_like(b1)
    mw2 = np.zeros_like(w2); vw2 = np.zeros_like(w2)
    mb2 = 0.0; vb2 = 0.0
    for step in range(1, epochs + 1):
        z = x @ w1.T + b1
        h = np.maximum(z, 0.0)
        pred = h @ w2 + b2
        diff = pred - y
        grad_pred = (2.0 / n) * diff
        gw2 = h.T @ grad_pred
        gb2 = grad_pred.sum()
        gh = grad_pred[:, None] * w2[None, :]
        gz = gh * (z > 0.0)
        gw1 = gz.T @ x
        gb1 = gz.sum(axis=0)
        beta1, beta2, eps = 0.9, 0.999, 1e-8
        for grad, mom, vel in ((gw1, mw1, vw1), (gb1, mb1, vb1), (gw2, mw2, vw2)):
            mom[...] = beta1 * mom + (1 - beta1) * grad
            vel[...] = beta2 * vel + (1 - beta2) * grad * grad
        mb2 = beta1 * mb2 + (1 - beta1) * gb2
        vb2 = beta2 * vb2 + (1 - beta2) * gb2 * gb2
        corr1 = learning_rate * np.sqrt(1 - beta2 ** step) / (1 - beta1 ** step)
        w1 -= corr1 * mw1 / (np.sqrt(vw1) + eps)
        b1 -= corr1 * mb1 / (np.sqrt(vb1) + eps)
        w2 -= corr1 * mw2 / (np.sqrt(vw2) + eps)
        b2 -= corr1 * mb2 / (np.sqrt(vb2) + eps)
    pred = np.maximum(x @ w1.T + b1, 0.0) @ w2 + b2
    mse = float(np.mean((pred - y) ** 2))
    return {"w1": w1, "b1": b1, "w2": w2, "b2": np.float32(b2), "mse": mse}
