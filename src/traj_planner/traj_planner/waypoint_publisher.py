#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # -------------------------------
        # Load CSV
        # -------------------------------
        csv_path = "/home/hkou/work/cmu_biorobotics/rope_sim/rope_sim_ws/src/traj_planner/waypoint_data/translation.csv"

        try:
            self.waypoints = np.loadtxt(csv_path, delimiter=",", skiprows=1)
            self.get_logger().info(f"Loaded {self.waypoints.shape[0]} waypoints.")
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {e}")
            raise

        # -------------------------------
        # Publishers
        # -------------------------------
        self.servo_pub = self.create_publisher(PoseStamped, '/ur/servo_tool_linear', 10)
        self.move_pub  = self.create_publisher(PoseStamped, '/ur/move_linear', 10)
        self.u_pub     = self.create_publisher(Float32, '/base_u', 10)

        # -------------------------------
        # Subscriber
        # -------------------------------
        self.cmd_sub = self.create_subscription(
            String,
            '/waypoint_publisher/command',
            self.command_callback,
            10
        )

        # -------------------------------
        # State
        # -------------------------------
        self.current_index = 0
        self.status = "Initializing"
        self.rate_hz = 500
        self.servo_timer = None  # created on Start, cancelled on Stop/end

        # -------------------------------
        # Home Pose
        # -------------------------------
        self.home_pose = self.create_pose_msg(self.waypoints[0])

        # -------------------------------
        # Delayed MoveL Timer
        # -------------------------------
        self.move_timer = self.create_timer(0.5, self.initial_move_timer_callback)

        self.get_logger().info("Waiting to send initial moveL...")

    # ---------------------------------------------------
    # Initial MoveL with timing
    # ---------------------------------------------------
    def initial_move_timer_callback(self):
        sub_count = self.move_pub.get_subscription_count()

        if sub_count == 0:
            self.get_logger().info("Waiting for /ur/move_linear subscriber...")
            return

        self.get_logger().info("Publishing initial moveL to first waypoint.")
        self.move_pub.publish(self.home_pose)

        self.get_logger().info(
            f"MoveL sent: {self.home_pose.pose.position.x:.4f}, "
            f"{self.home_pose.pose.position.y:.4f}, "
            f"{self.home_pose.pose.position.z:.4f}"
        )

        self.status = "Armed"

        # Cancel timer after successful publish
        self.move_timer.cancel()

    # ---------------------------------------------------
    # Command Callback
    # ---------------------------------------------------
    def command_callback(self, msg: String):
        cmd = msg.data.strip()

        if cmd == "Start" and self.status == "Armed":
            self.get_logger().info("START received.")
            self.status = "Running"
            self.current_index = 0
            self.start_servo_timer()

        elif cmd == "Stop" and self.status == "Running":
            self.get_logger().info("STOP received.")
            self.stop_servo_timer()
            self.status = "Stopped"

        else:
            self.get_logger().warn(f"Unknown or invalid command: {cmd}")

    # ---------------------------------------------------
    # Timer start/stop (identical pattern to script 2)
    # ---------------------------------------------------
    def start_servo_timer(self):
        # If a prior run left a timer around, cancel it.
        if self.servo_timer is not None:
            self.servo_timer.cancel()
            self.servo_timer = None

        self.get_logger().info(f"Starting waypoint publishing at {self.rate_hz} Hz (timer-based).")

        self.servo_timer = self.create_timer(
            1.0 / float(self.rate_hz),
            self.servo_timer_callback
        )

    def stop_servo_timer(self):
        if self.servo_timer is not None:
            self.servo_timer.cancel()
            self.servo_timer = None

    # ---------------------------------------------------
    # Servo timer callback: publish one waypoint per tick
    # ---------------------------------------------------
    def servo_timer_callback(self):
        if self.status != "Running":
            return

        # End condition
        if self.current_index >= self.waypoints.shape[0]:
            self.get_logger().info("Trajectory complete.")
            self.stop_servo_timer()
            self.status = "Armed"
            self.get_logger().info("Execution finished. Back to ARMED.")
            return

        pose_msg, u_msg = self.waypoint_to_msg(self.current_index)

        self.servo_pub.publish(pose_msg)
        self.u_pub.publish(u_msg)

        self.current_index += 1

    # ---------------------------------------------------
    # Waypoint Conversion
    # ---------------------------------------------------
    def waypoint_to_msg(self, index: int):
        wp = self.waypoints[index]

        pose_msg = self.create_pose_msg(wp)

        u_msg = Float32()
        u_msg.data = float(wp[7])

        return pose_msg, u_msg

    # ---------------------------------------------------
    # Pose Builder
    # ---------------------------------------------------
    def create_pose_msg(self, wp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base'

        pose_msg.pose.position.x = float(wp[0])
        pose_msg.pose.position.y = float(wp[1])
        pose_msg.pose.position.z = float(wp[2])
        pose_msg.pose.orientation.x = float(wp[3])
        pose_msg.pose.orientation.y = float(wp[4])
        pose_msg.pose.orientation.z = float(wp[5])
        pose_msg.pose.orientation.w = float(wp[6])

        return pose_msg


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
