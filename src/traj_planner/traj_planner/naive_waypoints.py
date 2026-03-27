import numpy as np

TRAJECTORY_PATH = "/home/hkou/work/cmu_biorobotics/DLO_sim/rope_sim_ws/src/traj_planner/waypoint_data/20260228/success_traj_csv_high_27_new_stiffen/min_dis_0.04_target_0.0_2.0_2.3_success_episode_000_env1_step1913.csv"



def _load_csv( path):

    data = np.loadtxt(path, delimiter=',', skiprows=1)

    if data.ndim == 1:
        data = data.reshape(1, -1)

    if data.shape[1] != 7:
        raise RuntimeError(
            f"CSV must have 7 columns (t + 6 joints). Got {data.shape}"
        )

    t_ref = data[:, 0]
    q = data[:, 1:7]

    if not np.isfinite(q).all():
        raise RuntimeError("CSV contains NaN/Inf joint values.")
    return t_ref,q, data



t, wpt, data = _load_csv(TRAJECTORY_PATH)

last = wpt[-1,-3]/2 + wpt[0,-3]/2

last_row = wpt[-1]
last_row[-3] = last

factor = 1.5

# num_t = int(wpt.shape[0]*factor)
# # new_wpt = np.linspace(wpt[0], last_row, num=wpt.shape[0])
# new_wpt = np.linspace(wpt[0], wpt[-1], num=num_t)
# new_t = np.linspace(0,t[-1]*factor, num_t)

# new_data = np.hstack((new_t.reshape((-1,1)), new_wpt))


# new_data = np.repeat(data, 2, axis=0)
# new_data[]
np.savetxt(TRAJECTORY_PATH+"_naive.csv", new_data, delimiter=',', fmt="%.6f")

