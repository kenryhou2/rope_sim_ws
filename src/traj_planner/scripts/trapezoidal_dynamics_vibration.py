#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D


# ============================================================
# Quaternion Utilities
# ============================================================

def axis_angle_to_quaternion(r):
    angle = np.linalg.norm(r)
    if angle < 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = r / angle
    s = np.sin(angle / 2.0)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, np.cos(angle/2.0)])


def quaternion_inverse(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quaternion_multiply(q, r):
    x1, y1, z1, w1 = q
    x2, y2, z2, w2 = r
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def slerp(q0, q1, t):
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        q = (1 - t) * q0 + t * q1
        return q / np.linalg.norm(q)

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t

    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = np.sin(theta) / sin_theta_0

    return s0 * q0 + s1 * q1


# ============================================================
# Clean Translation Trapezoid
# ============================================================

def generate_vector_trapezoidal_trajectory(p0, pf, v_max=0.1, a_max=1, dt=0.01):

    p0 = np.array(p0, dtype=float)
    pf = np.array(pf, dtype=float)

    disp = pf - p0
    D = np.linalg.norm(disp)

    if D == 0:
        return np.array([0.0]), np.array([p0]), np.zeros((1,3)), np.zeros((1,3)), np.array([0.0])

    n = disp / D

    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc**2

    if 2*d_acc > D:
        t_acc = np.sqrt(D / a_max)
        t_total = 2*t_acc
        trapezoidal = False
    else:
        d_cruise = D - 2*d_acc
        t_cruise = d_cruise / v_max
        t_total = 2*t_acc + t_cruise
        trapezoidal = True

    times = np.arange(0, t_total+dt, dt)

    positions = []
    velocities = []
    accelerations = []
    u_values = []

    p = p0.copy()
    v = np.zeros(3)
    u = 0.0

    for t in times:
        if t < t_acc:
            a_scalar = a_max
        elif trapezoidal and t < (t_acc + (t_total - 2*t_acc)):
            a_scalar = 0
        else:
            a_scalar = -a_max

        a = a_scalar * n
        v += a * dt

        if np.linalg.norm(v) > v_max:
            v = v_max * v / np.linalg.norm(v)

        p_new = p + v * dt
        u += np.linalg.norm(p_new - p) * 1000.0

        positions.append(p_new.copy())
        velocities.append(v.copy())
        accelerations.append(a.copy())
        u_values.append(u)

        p = p_new

    return (
        np.array(times),
        np.array(positions),
        np.array(velocities),
        np.array(accelerations),
        np.array(u_values),
    )


# ============================================================
# Clean Rotation Trapezoid
# ============================================================

def generate_rotation_trapezoidal_trajectory(r0, rf, omega_max=3, alpha_max=1, dt=0.01):

    q0 = axis_angle_to_quaternion(r0)
    qf = axis_angle_to_quaternion(rf)

    if np.dot(q0, qf) < 0:
        qf = -qf

    dot = np.clip(np.dot(q0, qf), -1.0, 1.0)
    Theta = 2 * np.arccos(dot)

    t_acc = omega_max / alpha_max
    theta_acc = 0.5 * alpha_max * t_acc**2

    if 2*theta_acc > Theta:
        t_acc = np.sqrt(Theta / alpha_max)
        t_total = 2*t_acc
        trapezoidal = False
    else:
        theta_cruise = Theta - 2*theta_acc
        t_cruise = theta_cruise / omega_max
        t_total = 2*t_acc + t_cruise
        trapezoidal = True

    times = np.arange(0, t_total+dt, dt)

    theta_list = []
    omega = 0.0
    theta = 0.0
    u = 0.0
    u_values = []

    prev_theta = 0.0

    for t in times:
        if t < t_acc:
            a_scalar = alpha_max
        elif trapezoidal and t < (t_acc + (t_total - 2*t_acc)):
            a_scalar = 0
        else:
            a_scalar = -alpha_max

        omega += a_scalar * dt
        omega = np.clip(omega, -omega_max, omega_max)
        theta += omega * dt

        theta_list.append(theta)

        delta_theta = abs(theta - prev_theta)
        u += delta_theta * 70.0
        u_values.append(u)

        prev_theta = theta

    quaternions = []
    for theta_i in theta_list:
        s = theta_i / Theta if Theta > 0 else 0.0
        q_interp = slerp(q0, qf, np.clip(s, 0.0, 1.0))
        quaternions.append(q_interp)

    return (
        np.array(times),
        np.array(quaternions),
        np.array(u_values),
    )


# ============================================================
# Clean Chirp
# ============================================================

def generate_chirp_trajectory(origin, axis="y", amp=0.002,
                              f_start=0.5, f_end=20.0,
                              duration=10.0, dt=0.002):

    origin = np.asarray(origin, dtype=float)
    axis_map = {"x":0, "y":1, "z":2}
    axis_idx = axis_map[axis]

    times = np.arange(0.0, duration+dt, dt)
    T = times[-1]

    k = (f_end - f_start) / T
    phase = 2*np.pi*(f_start*times + 0.5*k*times**2)
    signal = amp * np.sin(phase)

    positions = np.tile(origin, (len(times),1))
    positions[:,axis_idx] += signal

    velocities = np.gradient(positions, dt, axis=0)
    accelerations = np.gradient(velocities, dt, axis=0)

    speed = np.linalg.norm(velocities, axis=1)
    u_values = np.cumsum(speed*dt*1000.0)
    u_values[0] = 0.0

    return (
        np.array(times),
        np.array(positions),
        np.array(velocities),
        np.array(accelerations),
        np.array(u_values),
    )


# ============================================================
# Clean Sinusoid
# ============================================================

def generate_sinusoid_trajectory(origin, axis="y",
                                 amplitude=0.002,
                                 frequency=5.0,
                                 duration=5.0,
                                 dt=0.002):

    origin = np.asarray(origin, dtype=float)
    axis_map = {"x":0, "y":1, "z":2}
    axis_idx = axis_map[axis]

    times = np.arange(0.0, duration+dt, dt)

    signal = amplitude * np.sin(2*np.pi*frequency*times)

    positions = np.tile(origin, (len(times),1))
    positions[:,axis_idx] += signal

    velocities = np.gradient(positions, dt, axis=0)
    accelerations = np.gradient(velocities, dt, axis=0)

    speed = np.linalg.norm(velocities, axis=1)
    u_values = np.cumsum(speed*dt*1000.0)
    u_values[0] = 0.0

    return (
        np.array(times),
        np.array(positions),
        np.array(velocities),
        np.array(accelerations),
        np.array(u_values),
    )


# ============================================================
# CSV + Visualization
# ============================================================

def save_poses_to_csv(filename, translations, orientations, u_values):

    if orientations.shape[1] == 3:
        quats = np.array([axis_angle_to_quaternion(o) for o in orientations])
    else:
        quats = orientations

    poses = np.hstack((translations, quats, u_values.reshape(-1,1)))
    df = pd.DataFrame(poses, columns=["x","y","z","px","py","pz","pw","u"])
    df.to_csv(filename, index=False, float_format="%.5f")
    print("Saved:", filename)


def visualize_translation_trajectory(positions, times, title=""):

    positions = np.array(positions)
    times = np.array(times)

    # -------------------------------------------------
    # 3D Trajectory Plot
    # -------------------------------------------------
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(positions[:, 0],
            positions[:, 1],
            positions[:, 2],
            label="Trajectory")

    ax.scatter(positions[0, 0],
               positions[0, 1],
               positions[0, 2],
               color='green',
               label="Start")

    ax.scatter(positions[-1, 0],
               positions[-1, 1],
               positions[-1, 2],
               color='red',
               label="End")

    ax.set_title(f"3D Trajectory {title}")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    plt.tight_layout()

    # -------------------------------------------------
    # 2D Position vs Time Plots
    # -------------------------------------------------
    fig2, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axes[0].plot(times, positions[:, 0])
    axes[0].set_ylabel("X (m)")

    axes[1].plot(times, positions[:, 1])
    axes[1].set_ylabel("Y (m)")

    axes[2].plot(times, positions[:, 2])
    axes[2].set_ylabel("Z (m)")
    axes[2].set_xlabel("Time (s)")

    fig2.suptitle(f"Position vs Time {title}")
    plt.tight_layout()
    plt.show()



# ============================================================
# Main
# ============================================================

def main():

    dir_path = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(dir_path, "..", "waypoint_data")
    os.makedirs(out_dir, exist_ok=True)

    traj_type = "sinusoid"   # change here

    if traj_type == "translation":

        origin = [-0.37866, -0.77, 0.65034]

        times, positions, velocities, accelerations, u_values = \
            generate_vector_trapezoidal_trajectory(
                p0=origin,
                # pf=[0.13815, -0.40951, 0.69732],
                pf=[0.13813, -0.40950, 0.28221], 
                v_max=0.1,
                a_max=1.0,
                dt=0.002
            )

        orientations = np.tile(np.array([1.393, 2.457, -0.652]), # angle axis rotation vector (rad)
                               (len(times),1))

        save_poses_to_csv(
            os.path.join(out_dir,"translation.csv"),positions,
            orientations,
            u_values
        )

        visualize_translation_trajectory(positions, times, "Translation")
    
    elif traj_type == "sinusoid":
        
        origin = [-0.37866, -0.108, 0.65034]

        times, positions, velocities, accelerations, u_values = \
            generate_sinusoid_trajectory(
                origin=origin,
                axis="x",
                amplitude=0.050,
                frequency=5.0,
                duration=5.0,
                dt=0.002
            )
        orientations = np.tile(np.array([0.889, -1.985, -5.539]), # angle axis rotation vector (rad)
                               (len(times),1))
        save_poses_to_csv(
            os.path.join(out_dir,"sinusoid.csv"),positions,
            orientations,
            u_values
        )
        visualize_translation_trajectory(positions, times, "Sinusoid")



if __name__ == "__main__":
    main()
