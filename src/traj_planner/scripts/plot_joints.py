#!/usr/bin/env python3
"""
plot_joint_csv.py

Standalone plotting for a joint-trajectory CSV like:
t, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3

Assumes joint columns are in RADIANS in the CSV and plots in DEGREES.
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def plot_csv(csv_path: str, title: str = "Joint Trajectory (deg)"):
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    # Required time column
    if "t" not in df.columns:
        raise ValueError(f"CSV must contain a 't' column. Found: {list(df.columns)}")

    # Default UR-style joint columns (same as your script)
    default_joint_cols = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

    # If all defaults exist, use them; otherwise fall back to "all numeric columns except t"
    if all(c in df.columns for c in default_joint_cols):
        joint_cols = default_joint_cols
    else:
        numeric_cols = [c for c in df.columns if c != "t" and pd.api.types.is_numeric_dtype(df[c])]
        if len(numeric_cols) == 0:
            raise ValueError("No numeric joint columns found (excluding 't').")
        joint_cols = numeric_cols

    time_vec = df["t"].to_numpy(dtype=float)

    traj_rad = df[joint_cols].to_numpy(dtype=float)
    traj_deg = np.rad2deg(traj_rad)

    n = traj_deg.shape[1]
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(10, 1.4 * n + 2))
    if n == 1:
        axes = [axes]

    for i, col in enumerate(joint_cols):
        axes[i].plot(time_vec, traj_deg[:, i])
        axes[i].set_ylabel(f"{col} (deg)")
        axes[i].grid(True)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(title)
    plt.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="Plot joint angles from a trajectory CSV in degrees.")
    ap.add_argument("csv", help="Path to CSV (expects 't' column).")
    ap.add_argument("--title", default="Joint Trajectory (deg)", help="Figure title.")
    args = ap.parse_args()

    try:
        plot_csv(args.csv, args.title)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
