#!/usr/bin/env python3

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ------------------------------------------------------------
# Generate trajectory using explicit final configuration
# ------------------------------------------------------------
def generate_interpolated_trajectory(
        q0,
        qf,
        interpolation_steps,
        hold_steps=0):

    traj = []

    for k in range(interpolation_steps):
        alpha = k / (interpolation_steps - 1)
        q = (1 - alpha) * q0 + alpha * qf
        traj.append(q)

    traj = np.array(traj)

    # Hold phase
    if hold_steps > 0:
        hold_segment = np.tile(traj[-1], (hold_steps, 1))
        traj = np.vstack((traj, hold_segment))

    return traj


# ------------------------------------------------------------
# Relabel time for execution at fixed rate
# ------------------------------------------------------------
def relabel_time(traj, execution_rate_hz):
    dt = 1.0 / execution_rate_hz
    return np.arange(len(traj)) * dt


# ------------------------------------------------------------
# Plot CSV
# ------------------------------------------------------------
def plot_csv(csv_path):

    df = pd.read_csv(csv_path)

    time_vec = df["t"].values
    traj = df.iloc[:, 1:7].values

    joint_names = df.columns[1:7]

    traj_deg = np.rad2deg(traj)

    fig, axes = plt.subplots(6, 1, sharex=True, figsize=(10, 8))

    for i in range(6):
        axes[i].plot(time_vec, traj_deg[:, i])
        axes[i].set_ylabel(f"{joint_names[i]} (deg)")
        axes[i].grid(True)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Interpolated Joint Trajectory")

    plt.tight_layout()
    plt.show()


# ------------------------------------------------------------
# Helper: degrees → radians
# ------------------------------------------------------------
def build_joint_vector_deg(config_deg):

    joint_order = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    ]

    return np.deg2rad(
        np.array([config_deg[j] for j in joint_order])
    )


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
def main():

    TRAJ_NAME = "whip_traj.csv"
    EXECUTION_RATE_HZ = 500.0

    INTERPOLATION_STEPS = 2     # <-- controls resolution
    HOLD_STEPS = 500              # hold final config
    PLOT = True

    # ------------------------------------------------------------
    # Initial joint configuration (degrees)
    # ------------------------------------------------------------
    q0_config_deg = {
        "shoulder_pan": 85.57,
        "shoulder_lift": -112.46,
        "elbow": 129.13,
        "wrist_1": -105.89,
        "wrist_2": -98.23,
        "wrist_3": 59.75,
    }

    # ------------------------------------------------------------
    # Final joint configuration (degrees)
    # ------------------------------------------------------------
    qf_config_deg = {
        "shoulder_pan": 85.57,
        "shoulder_lift": -112.46,
        "elbow": 39.13,       # 129.13 - 90
        "wrist_1": -195.89,   # -105.89 - 90
        "wrist_2": -98.23,
        "wrist_3": 59.75,
    }

    q0 = build_joint_vector_deg(q0_config_deg)
    qf = build_joint_vector_deg(qf_config_deg)

    # ------------------------------------------------------------
    # Generate interpolated trajectory
    # ------------------------------------------------------------
    traj = generate_interpolated_trajectory(
        q0=q0,
        qf=qf,
        interpolation_steps=INTERPOLATION_STEPS,
        hold_steps=HOLD_STEPS
    )

    # ------------------------------------------------------------
    # Relabel time
    # ------------------------------------------------------------
    time_vec = relabel_time(traj, EXECUTION_RATE_HZ)

    # ------------------------------------------------------------
    # Save CSV
    # ------------------------------------------------------------
    data = np.column_stack((time_vec, traj))

    df = pd.DataFrame(
        data,
        columns=[
            "t",
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3"
        ]
    )

    script_dir = os.path.dirname(os.path.abspath(__file__))
    waypoint_dir = os.path.join(script_dir, "..", "waypoint_data")
    os.makedirs(waypoint_dir, exist_ok=True)

    output_path = os.path.join(waypoint_dir, TRAJ_NAME)
    df.to_csv(output_path, index=False)

    print(f"Trajectory saved to: {output_path}")
    print(f"Interpolation steps: {INTERPOLATION_STEPS}")
    print(f"Execution duration: {time_vec[-1]:.3f} s")

    if PLOT:
        plot_csv(output_path)


if __name__ == "__main__":
    main()
