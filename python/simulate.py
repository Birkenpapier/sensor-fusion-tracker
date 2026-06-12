#!/usr/bin/env python3
"""Generate a ground-truth target trajectory and noisy multi-sensor fixes.

Two complementary sensors observe the same target:

  * radar -- precise along x (range axis), coarse along y
  * EO/IR -- precise along y (angular),   coarse along x, and at a higher rate

Each fix is written as one CSV row. The C++ tracker fuses them; visualize.py
scores the result. The target flies a constant-velocity leg followed by a
gentle turn so the constant-velocity filter is genuinely exercised.

Outputs:
  truth.csv         : t,x,y,vx,vy
  measurements.csv  : t,sensor,x,y   (time-sorted, ready for sft_app)
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def trajectory(t: float) -> tuple[float, float, float, float]:
    """Return (x, y, vx, vy) of the true target at time t (seconds)."""
    speed = 8.0  # m/s
    if t < 20.0:
        # straight leg heading north-east
        heading = math.radians(35.0)
        vx, vy = speed * math.cos(heading), speed * math.sin(heading)
        x = vx * t
        y = vy * t
        return x, y, vx, vy
    # gentle constant-rate turn after t = 20 s
    turn_rate = math.radians(4.0)  # rad/s
    h0 = math.radians(35.0)
    t0 = 20.0
    x0 = speed * math.cos(h0) * t0
    y0 = speed * math.sin(h0) * t0
    dt = t - t0
    heading = h0 + turn_rate * dt
    # integrate velocity over the turn
    x = x0 + (speed / turn_rate) * (math.sin(heading) - math.sin(h0))
    y = y0 - (speed / turn_rate) * (math.cos(heading) - math.cos(h0))
    vx = speed * math.cos(heading)
    vy = speed * math.sin(heading)
    return x, y, vx, vy


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", default="data", help="output directory")
    ap.add_argument("--duration", type=float, default=40.0, help="seconds")
    ap.add_argument("--radar-hz", type=float, default=4.0)
    ap.add_argument("--eo-hz", type=float, default=12.0)
    ap.add_argument("--radar-sigma", type=float, nargs=2, default=(1.5, 12.0),
                    metavar=("SX", "SY"))
    ap.add_argument("--eo-sigma", type=float, nargs=2, default=(12.0, 1.5),
                    metavar=("SX", "SY"))
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # --- ground truth (dense sampling for a clean reference curve) ---
    truth_dt = 0.05
    with (out_dir / "truth.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "vx", "vy"])
        t = 0.0
        while t <= args.duration + 1e-9:
            x, y, vx, vy = trajectory(t)
            w.writerow([f"{t:.3f}", f"{x:.4f}", f"{y:.4f}",
                        f"{vx:.4f}", f"{vy:.4f}"])
            t += truth_dt

    # --- sensor fixes ---
    measurements: list[tuple[float, str, float, float]] = []

    def sample(rate_hz: float, sensor: str, sx: float, sy: float) -> None:
        step = 1.0 / rate_hz
        t = step  # first fix one period in
        while t <= args.duration + 1e-9:
            x, y, _, _ = trajectory(t)
            mx = x + rng.normal(0.0, sx)
            my = y + rng.normal(0.0, sy)
            measurements.append((t, sensor, mx, my))
            t += step

    sample(args.radar_hz, "radar", *args.radar_sigma)
    sample(args.eo_hz, "eo", *args.eo_sigma)
    measurements.sort(key=lambda r: r[0])  # tracker requires ascending time

    with (out_dir / "measurements.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "sensor", "x", "y"])
        for t, sensor, mx, my in measurements:
            w.writerow([f"{t:.3f}", sensor, f"{mx:.4f}", f"{my:.4f}"])

    print(f"wrote {out_dir/'truth.csv'} and {out_dir/'measurements.csv'} "
          f"({len(measurements)} fixes)")


if __name__ == "__main__":
    main()
