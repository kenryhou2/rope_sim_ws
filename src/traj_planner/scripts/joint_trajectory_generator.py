#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ------------------------------------------------------------
# Generate trajectory with per-joint start offsets (seconds)
# Each joint holds q0 until its start offset, then linearly
# interpolates to qf over interpolation_steps samples.
# After the latest joint finishes, hold final config for hold_steps.
# ------------------------------------------------------------
def generate_interpolated_trajectory_with_offsets(
    q0: np.ndarray,
    qf: np.ndarray,
    interpolation_steps: int,
    execution_rate_hz: float,
    start_offsets_s: np.ndarray,
    hold_steps: int = 0,
):
    assert q0.shape == qf.shape
    dof = q0.shape[0]
    assert start_offsets_s.shape[0] == dof
    if interpolation_steps < 2:
        raise ValueError("interpolation_steps must be >= 2")

    dt = 1.0 / execution_rate_hz

    # Convert offsets (seconds) to integer sample offsets
    start_offsets_steps = np.rint(start_offsets_s / dt).astype(int)
    if np.any(start_offsets_steps < 0):
        raise ValueError("start offsets must be >= 0")

    # Each joint finishes after its own interpolation segment
    finish_steps = start_offsets_steps + (interpolation_steps - 1)
    total_steps = int(np.max(finish_steps) + 1 + hold_steps)

    # Build global time vector and trajectory
    t = np.arange(total_steps) * dt
    traj = np.zeros((total_steps, dof), dtype=float)

    for j in range(dof):
        s0 = start_offsets_steps[j]
        sf = finish_steps[j]

        # Before start: hold q0
        traj[:s0, j] = q0[j]

        # Interpolation segment: inclusive endpoints (length = interpolation_steps)
        seg_len = sf - s0 + 1
        alphas = np.linspace(0.0, 1.0, seg_len)
        traj[s0:sf + 1, j] = (1.0 - alphas) * q0[j] + alphas * qf[j]

        # After finish: hold qf (including hold tail)
        traj[sf + 1 :, j] = qf[j]

    return t, traj


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
    fig.suptitle("Joint Trajectory (per-joint start offsets)")
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
    return np.deg2rad(np.array([config_deg[j] for j in joint_order]))


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
def main():
    TRAJ_NAME = "whip_traj.csv"
    EXECUTION_RATE_HZ = 500.0

    INTERPOLATION_STEPS = 400     # per-joint interpolation samples
    HOLD_STEPS = 500             # hold after all joints have finished
    PLOT = True

    # Initial joint configuration (degrees)
    q0_config_deg = {
        "shoulder_pan": -91.30,
        "shoulder_lift": -130.38,
        "elbow": -68.26,
        "wrist_1": -10.57,
        "wrist_2": 97.80,
        "wrist_3": 173.23,
    }

    # Final joint configuration (degrees)
    qf_config_deg = {
        "shoulder_pan": -91.30,
        "shoulder_lift": -110.33,
        "elbow": -25.61,
        "wrist_1": 100.55,
        "wrist_2": 97.80,
        "wrist_3": 173.23,
    }

    # Per-joint start offsets (seconds)
    # Example: shoulder_pan starts immediately, shoulder_lift starts at 1.0s, etc.
    start_offsets_s = {
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,   #lift
        "elbow": 0.3,           #lift
        "wrist_1": 0.0,         #lift
        "wrist_2": 0.0,
        "wrist_3": 0.0,
    }

    joint_order = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

    q0 = build_joint_vector_deg(q0_config_deg)
    qf = build_joint_vector_deg(qf_config_deg)
    offsets_vec = np.array([start_offsets_s[j] for j in joint_order], dtype=float)

    # Generate per-joint-offset trajectory
    time_vec, traj = generate_interpolated_trajectory_with_offsets(
        q0=q0,
        qf=qf,
        interpolation_steps=INTERPOLATION_STEPS,
        execution_rate_hz=EXECUTION_RATE_HZ,
        start_offsets_s=offsets_vec,
        hold_steps=HOLD_STEPS,
    )

    # Save CSV
    data = np.column_stack((time_vec, traj))
    df = pd.DataFrame(data, columns=["t"] + joint_order)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    waypoint_dir = os.path.join(script_dir, "..", "waypoint_data")
    os.makedirs(waypoint_dir, exist_ok=True)

    output_path = os.path.join(waypoint_dir, TRAJ_NAME)
    df.to_csv(output_path, index=False)

    print(f"Trajectory saved to: {output_path}")
    print(f"Per-joint interpolation steps: {INTERPOLATION_STEPS}")
    print(f"Max start offset: {offsets_vec.max():.3f} s")
    print(f"Execution duration: {time_vec[-1]:.3f} s")

    if PLOT:
        plot_csv(output_path)


if __name__ == "__main__":
    main()
