
# rope_sim_ws

ROS 2 workspace for commanding a **UR arm via RTDE** and executing high-rate Cartesian trajectories generated from CSV files.

This workspace contains:

- `ur_rtde_ros` – RTDE-based UR controller + state receiver
- `traj_planner` – CSV trajectory generator + waypoint publisher

The system architecture is:

CSV trajectory  →  WaypointPublisher  →  /ur/move_linear (MoveL)
                                         /ur/servo_tool_linear (ServoL @ 500 Hz)
                                         /base_u (path progress)

URControllerNode → RTDE → Physical UR Arm  
URReceiverNode   → Publishes joint states + TCP pose

---

# ⚙️ Hardware Setup (IMPORTANT)

## 1. Ethernet Configuration
- Connect PC directly to UR controller via Ethernet.
- Configure **static IP** on both PC and robot.
- Ensure you can ping the robot.
- On the UR pendant: Settings → Network → Static

## 2. Remote Operation Mode
On the UR pendant:
- Set robot to **Remote Control mode**
- Release protective stop if needed
- Power on + brakes released

---

# 🚀 How To Run the System

## Step 1 — Build and Source Workspace

```bash
colcon build
source install/setup.bash
```

## Step 2 — Launch UR Controller

```bash
ros2 launch ur_rtde_ros launch_controller.launch.py
```

Expected result:
- RTDE connects to robot
- Arm enters servo mode
- Servo loop running at ~500 Hz

## Step 3 — Launch Waypoint Publisher

```bash
ros2 launch traj_planner waypoint_publisher.launch.py
```

Expected result:
- CSV is loaded
- Initial MoveL sent to first waypoint
- Robot moves to starting pose
- System status becomes ARMED

## Step 4 — Start Trajectory Execution

```bash
ros2 topic pub /waypoint_publisher/command std_msgs/msg/String "data: 'Start'" --once
```

Robot streams ServoL commands at 500 Hz.

To stop:

```bash
ros2 topic pub /waypoint_publisher/command std_msgs/msg/String "data: 'Stop'" --once
```

---

# 📈 Trajectory Generation

Trajectory script is located in:

src/traj_planner/scripts/

It supports:
- Trapezoidal translation
- Trapezoidal rotation
- Chirp motion
- Sinusoidal motion

Run:

```bash
python3 trajectory_script.py
```

Output CSV is saved to:

src/traj_planner/waypoint_data/

CSV format:

| x | y | z | px | py | pz | pw | u |
|---|---|---|----|----|----|----|---|

Where:
- Position is in meters
- Orientation is quaternion
- u is path progress (mm)

---

# 🧠 Waypoint Publisher Overview

- Loads CSV trajectory
- Sends initial MoveL
- Streams ServoL at 500 Hz
- Publishes `/base_u`

States:
- Initializing
- Armed
- Running
- Stopped

---

# 🦾 UR Controller Overview

## MoveL
- Blocking Cartesian linear interpolation
- Used for initial positioning and homing

## ServoL
- Real-time streaming Cartesian control
- Requires continuous high-frequency updates
- Automatically stops if stream pauses

---

# 📂 Workspace Structure

rope_sim_ws/
├── src/
│   ├── traj_planner/
│   │   ├── scripts/
│   │   └── waypoint_data/
│   └── ur_rtde_ros/

---

# 🏁 Minimal Run Sequence

```bash
colcon build
source install/setup.bash
ros2 launch ur_rtde_ros launch_controller.launch.py
ros2 launch traj_planner waypoint_publisher.launch.py
ros2 topic pub /waypoint_publisher/command std_msgs/msg/String "data: 'Start'" --once
```
