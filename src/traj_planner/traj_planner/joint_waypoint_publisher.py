#!/usr/bin/env python3

import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class JointCsvWaypointPublisher(Node):

    def __init__(self):
        super().__init__('joint_waypoint_publisher')

        # ==========================================================
        # HARD-CODED CONFIGURATION
        # ==========================================================

        # self.TRAJECTORY_PATH = "/home/hkou/work/cmu_biorobotics/DLO_sim/rope_sim_ws/src/traj_planner/waypoint_data/whip_traj_high.csv"
        # self.TRAJECTORY_PATH = "/home/hkou/work/cmu_biorobotics/DLO_sim/rope_sim_ws/src/traj_planner/waypoint_data/success_episode_004_env0_step47011.csv"
        # self.TRAJECTORY_PATH = "/home/hkou/work/cmu_biorobotics/DLO_sim/rope_sim_ws/src/traj_planner/waypoint_data/20260301/success_traj_csv_high_29_medium_target/min_dis_0.04_target_0.0_2.0_1.5_success_episode_002_env4_step1596.csv"
        # self.TRAJECTORY_PATH = "/home/hkou/work/cmu_biorobotics/DLO_sim/rope_sim_ws/src/traj_planner/waypoint_data/bj/success_traj_csv_6_low_target/min_dis_0.045_target_0.0_1.7_0.5_success_episode_001_env6_step2230.csv"
        # self.TRAJECTORY_PATH = "/home/hkou/work/cmu_biorobotics/DLO_sim/rope_sim_ws/src/traj_planner/waypoint_data/bj/success_traj_csv_8_high_target/min_dis_0.046_target_0.0_2.0_2.3_success_episode_000_env7_step1470.csv"
        # self.TRAJECTORY_PATH = "/home/hkou/work/cmu_biorobotics/DLO_sim/rope_sim_ws/src/traj_planner/waypoint_data/bj_2/min_dis_0.049_target_0.0_2.0_2.3_success_episode_000_env6_step3794.csv"
        self.TRAJECTORY_PATH = "//home/hkou/work/cmu_biorobotics/DLO_sim/rope_sim_ws/src/traj_planner/waypoint_data/hit_apple/success_traj_csv_2/min_dis_0.049_target_0.0_2.2_1.4_success_episode_007_env0_step1893.csv"
        # self.TRAJECTORY_PATH = "/home/hkou/work/cmu_biorobotics/DLO_sim/rope_sim_ws/src/traj_planner/waypoint_data/success_episode_000_env2_step5192.csv"
        self.get_logger().info(f"\n\n\nHERE\n\n {self.TRAJECTORY_PATH}")
        self.PUBLISH_RATE_HZ = 500
        self.PREFIX = ""
        self.POSITION_TOLERANCE = 0.01
        self.STABLE_CYCLES_REQUIRED = 15

        if not os.path.exists(self.TRAJECTORY_PATH):
            raise RuntimeError(f"Trajectory file not found: {self.TRAJECTORY_PATH}")

        self.rate_hz = self.PUBLISH_RATE_HZ
        self.prefix = self.PREFIX
        self.position_tolerance = self.POSITION_TOLERANCE
        self.stable_cycles_required = self.STABLE_CYCLES_REQUIRED

        # ==========================================================
        # Topics (match URControllerNode)
        # ==========================================================

        self.servo_topic = '/ur/servo_joint'
        self.move_topic = '/ur/move_joint'
        self.cmd_topic = 'joint_waypoint_publisher/command'
        self.status_topic = 'joint_waypoint_publisher/status'

        # ==========================================================
        # Publishers
        # ==========================================================

        self.pub_servo = self.create_publisher(JointState, self.servo_topic, 10)
        self.pub_move = self.create_publisher(JointState, self.move_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)

        # ==========================================================
        # Subscribers
        # ==========================================================

        self.sub_cmd = self.create_subscription(
            String, self.cmd_topic, self._on_cmd, 10)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_joint_states = self.create_subscription(
            JointState,
            '/ur/joint_states',
            self._on_joint_state,
            sensor_qos)

        # ==========================================================
        # Joint names
        # ==========================================================

        base_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.joint_names = [self.prefix + n for n in base_names]

        # ==========================================================
        # Load CSV
        # ==========================================================

        self._load_csv(self.TRAJECTORY_PATH)

        # ==========================================================
        # Execution state
        # ==========================================================

        self.running = False
        self.initialized = False
        self.stable_counter = 0
        self.idx = 0
        self.status = "Initializing"

        self.initial_target_q = self.q[0]

        # ==========================================================
        # Delayed moveJ timer (subscriber-aware)
        # ==========================================================

        self.move_timer = self.create_timer(
            0.5,
            self._initial_move_timer_callback
        )

        # ==========================================================
        # Servo timer (500 Hz)
        # ==========================================================

        self.servo_timer = self.create_timer(
            1.0 / self.rate_hz,
            self._on_timer
        )

        self.get_logger().info(
            f"Loaded trajectory: {self.TRAJECTORY_PATH} | "
            f"Waypoints: {self.q.shape[0]} | "
            f"Rate: {self.rate_hz} Hz"
        )

    # --------------------------------------------------
    # Initial moveJ (subscriber-aware)
    # --------------------------------------------------
    def _initial_move_timer_callback(self):

        sub_count = self.pub_move.get_subscription_count()

        if sub_count == 0:
            self.get_logger().info("Waiting for move_joint subscriber...")
            return

        self.get_logger().info("Publishing initial moveJ.")

        self._send_movej(self.initial_target_q)

        self._publish_status("Armed")
        self.status = "Armed"

        self.move_timer.cancel()

    # --------------------------------------------------
    # CSV Loader
    # --------------------------------------------------
    def _load_csv(self, path):

        data = np.loadtxt(path, delimiter=',', skiprows=1)

        if data.ndim == 1:
            data = data.reshape(1, -1)

        if data.shape[1] != 7:
            raise RuntimeError(
                f"CSV must have 7 columns (t + 6 joints). Got {data.shape}"
            )

        self.t_ref = data[:, 0]
        self.q = data[:, 1:7]

        if not np.isfinite(self.q).all():
            raise RuntimeError("CSV contains NaN/Inf joint values.")

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------

    def _mk_joint_state(self, q6):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(x) for x in q6]
        return msg

    def _send_movej(self, q6):
        self.pub_move.publish(self._mk_joint_state(q6))
        self.get_logger().info(f"move j {self._mk_joint_state(q6)}")

    def _send_servoj(self, q6):
        self.pub_servo.publish(self._mk_joint_state(q6))
        self.get_logger().info(f"servo j {self._mk_joint_state(q6)}")

    def _publish_status(self, text):
        m = String()
        m.data = text
        self.pub_status.publish(m)

    # --------------------------------------------------
    # Convergence check
    # --------------------------------------------------
    def _on_joint_state(self, msg: JointState):

        if self.initialized:
            return

        if len(msg.position) < 6:
            return

        q_current = np.array(msg.position[:6])
        error = np.abs(q_current - self.initial_target_q)
        max_error = np.max(error)

        if max_error < self.position_tolerance:
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        if self.stable_counter >= self.stable_cycles_required:
            self.initialized = True
            self._publish_status("Initialized")
            self.get_logger().info("MoveJ converged.")

    # --------------------------------------------------
    # Command handler
    # --------------------------------------------------
    def _on_cmd(self, msg: String):

        cmd = msg.data.strip()

        if cmd == "Start":

            if not self.initialized:
                self.get_logger().warning("Cannot start — not initialized yet.")
                return
            
            self.idx = 0
            self.running = True
            self._publish_status("Running")
            self.get_logger().info("Starting servo streaming.")

        elif cmd == "Stop":
            self.running = False
            self._publish_status("Stopped")

    # --------------------------------------------------
    # 500 Hz servo loop
    # --------------------------------------------------
    def _on_timer(self):

        if not self.running:
            return

        if self.idx >= self.q.shape[0]:
            self.running = False
            self._publish_status("Finished")
            self.get_logger().info("Trajectory complete.")
            return

        self._send_servoj(self.q[self.idx])
        self.idx += 1


# --------------------------------------------------
# Main
# --------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = JointCsvWaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
