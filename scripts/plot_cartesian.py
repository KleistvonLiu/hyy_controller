#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
python3 src/plot_cartesian.py /home/kleist/Documents/Code/temp/cartesian_log.csv
python3 plot_cartesian.py cartesian_log.csv --deg
python3 plot_cartesian.py cartesian_log.csv --robot-id 1
"""

import argparse
import csv
from typing import List, Tuple

import numpy as np
import matplotlib.pyplot as plt


def load_cartesian_csv(path: str, robot_id: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    CSV each row: robot_id,time,c0,c1,c2,c3,c4,c5(,...)
    Returns:
      t: (N,)
      c: (N,6)
    """
    ts: List[float] = []
    cs: List[List[float]] = []

    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            # basic validation
            if len(row) < 8:
                continue
            try:
                rid = int(float(row[0]))
                if rid != robot_id:
                    continue
                t = float(row[1])
                vals = [float(x) for x in row[2:8]]  # only first 6 dims
            except Exception:
                continue

            ts.append(t)
            cs.append(vals)

    if not ts:
        raise RuntimeError(f"No valid data for robot_id={robot_id} in {path}")

    t = np.asarray(ts, dtype=np.float64)
    c = np.asarray(cs, dtype=np.float64)

    # sort by time, and shift to t0=0 for nicer viewing
    order = np.argsort(t)
    t = t[order]
    c = c[order]
    t = t - t[0]

    return t, c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_path", help="cartesian_log.csv")
    ap.add_argument("--robot-id", type=int, default=0, help="robot id to plot (default: 0)")
    ap.add_argument("--deg", action="store_true", help="plot roll/pitch/yaw in degrees")
    args = ap.parse_args()

    t, c = load_cartesian_csv(args.csv_path, args.robot_id)

    labels = ["x", "y", "z", "roll", "pitch", "yaw"]
    units = ["m", "m", "m", "rad", "rad", "rad"]

    if args.deg:
        c = c.copy()
        c[:, 3:6] = np.rad2deg(c[:, 3:6])
        units[3:6] = ["deg", "deg", "deg"]

    fig, axs = plt.subplots(6, 1, sharex=True, figsize=(12, 12))

    for i in range(6):
        axs[i].plot(t, c[:, i])
        axs[i].set_ylabel(f"{labels[i]} ({units[i]})")
        axs[i].grid(True, alpha=0.3)

    axs[-1].set_xlabel("t (s)")
    fig.suptitle(f"Robot {args.robot_id} Cartesian")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
