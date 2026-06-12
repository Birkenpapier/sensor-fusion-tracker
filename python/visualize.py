#!/usr/bin/env python3
"""Plot and score the fused track against ground truth and raw measurements.

Reads truth.csv, measurements.csv and the fused track.csv produced by sft_app,
then:
  * prints RMSE for raw radar, raw EO, and the fused track
  * writes fusion.png -- a trajectory plot + position-error-over-time plot
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")  # headless: write a file, never open a window
import matplotlib.pyplot as plt  # noqa: E402


def truth_interp(truth: pd.DataFrame, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    tx = np.interp(t, truth["t"], truth["x"])
    ty = np.interp(t, truth["t"], truth["y"])
    return tx, ty


def rmse(px: np.ndarray, py: np.ndarray, tx: np.ndarray, ty: np.ndarray) -> float:
    return float(np.sqrt(np.mean((px - tx) ** 2 + (py - ty) ** 2)))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--track", default="data/track.csv")
    ap.add_argument("--out", default="data/fusion.png")
    args = ap.parse_args()

    data = Path(args.data_dir)
    truth = pd.read_csv(data / "truth.csv")
    meas = pd.read_csv(data / "measurements.csv")
    track = pd.read_csv(args.track)

    radar = meas[meas["sensor"] == "radar"]
    eo = meas[meas["sensor"] == "eo"]

    # --- scoring ---
    rx, ry = truth_interp(truth, radar["t"].to_numpy())
    ex, ey = truth_interp(truth, eo["t"].to_numpy())
    fx, fy = truth_interp(truth, track["t"].to_numpy())

    rmse_radar = rmse(radar["x"].to_numpy(), radar["y"].to_numpy(), rx, ry)
    rmse_eo = rmse(eo["x"].to_numpy(), eo["y"].to_numpy(), ex, ey)
    rmse_fused = rmse(track["px"].to_numpy(), track["py"].to_numpy(), fx, fy)

    print("position RMSE (metres):")
    print(f"  raw radar : {rmse_radar:7.3f}")
    print(f"  raw EO    : {rmse_eo:7.3f}")
    print(f"  fused     : {rmse_fused:7.3f}")
    best_raw = min(rmse_radar, rmse_eo)
    print(f"  fusion improves on the best single sensor by "
          f"{100 * (1 - rmse_fused / best_raw):.1f}%")

    # --- plots ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    ax1.scatter(radar["x"], radar["y"], s=8, alpha=0.3, label="radar fixes")
    ax1.scatter(eo["x"], eo["y"], s=8, alpha=0.3, label="EO fixes")
    ax1.plot(truth["x"], truth["y"], "k-", lw=2, label="ground truth")
    ax1.plot(track["px"], track["py"], "r-", lw=1.5, label="fused track")
    ax1.set_title("Trajectory")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.legend()
    ax1.axis("equal")
    ax1.grid(True, alpha=0.3)

    err = np.sqrt((track["px"].to_numpy() - fx) ** 2 +
                  (track["py"].to_numpy() - fy) ** 2)
    ax2.plot(track["t"], err, "r-", lw=1.2, label="fused position error")
    ax2.axhline(rmse_radar, color="C0", ls="--", label=f"radar RMSE {rmse_radar:.1f} m")
    ax2.axhline(rmse_eo, color="C1", ls="--", label=f"EO RMSE {rmse_eo:.1f} m")
    ax2.set_title("Fused position error over time")
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("error (m)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
