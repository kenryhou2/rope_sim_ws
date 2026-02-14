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

        self.TRAJECTORY_PATH = "/home/hkou/work/cmu_biorobotics/rope_sim/rope_sim_ws/src/traj_planner/waypoint_data/whip_traj.csv"
        self.PUBLISH_RATE_HZ = 500.0
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

    def _send_servoj(self, q6):
        self.pub_servo.publish(self._mk_joint_state(q6))

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
