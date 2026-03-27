#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Wrist2SinePublisher(Node):
    def __init__(self):
        super().__init__('wrist2_sine_publisher')

        # -------------------------------
        # Hard-coded config
        # -------------------------------
        self.PUBLISH_RATE_HZ = 500.0

        CYCLE_DURATION = 4.0
        N1 = 4
        self.SINE1_FREQ_HZ = N1 / CYCLE_DURATION
        self.AMP1_DEG = 18.0              # +/- 20 deg
        
        N2 = 6
        self.SINE2_FREQ_HZ = N2 / CYCLE_DURATION
        self.AMP2_DEG = 18.0

        self.PREFIX = ""

        # Base joint configuration (deg)
        # [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        self.base_deg = [-90.0, -120.0, -76.0, 16.5, 90.0, 175.0]

        self.amp1_rad = math.radians(self.AMP1_DEG)
        self.amp2_rad = math.radians(self.AMP2_DEG)
        self.base_rad = [math.radians(d) for d in self.base_deg]

        self.TOTAL_DURATION = 30.0

        # -------------------------------
        # Topics
        # -------------------------------
        self.servo_topic = '/ur/servo_joint'

        # -------------------------------
        # Publisher
        # -------------------------------
        self.pub_servo = self.create_publisher(JointState, self.servo_topic, 10)

        # -------------------------------
        # Joint names (match UR controller order)
        # -------------------------------
        base_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.joint_names = [self.PREFIX + n for n in base_names]

        # Current commanded joints (rad)
        self.q = self.base_rad.copy()

        # Start time
        self.t0 = self.get_clock().now()

        # 500 Hz timer (starts immediately)
        self.timer = self.create_timer(1.0 / self.PUBLISH_RATE_HZ, self._on_timer)

        self.get_logger().info(
            f"Publishing {self.servo_topic} at {self.PUBLISH_RATE_HZ} Hz | "
            f"wrist_2 sine around {self.base_deg[4]} deg: "
            f"+/-{self.AMP1_DEG} deg @ {self.SINE1_FREQ_HZ} Hz for 4s"
            f"stop 2s at the base pose"
            f"wrist_2 sine around {self.base_deg[4]} deg: "
            f"+/-{self.AMP2_DEG} deg @ {self.SINE2_FREQ_HZ} Hz for 4s"
            f"repeat 3 times to get 30s motions in total"
        )

    def _mk_joint_state(self, q6):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(x) for x in q6]
        return msg

    def _on_timer(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        if t >= self.TOTAL_DURATION:
            self.q = self.base_rad.copy()
            self.pub_servo.publish(self._mk_joint_state(self.q))
            self.get_logger().info("Motion Finished (30s).") 
        else:
            cycle_time = t % 10
        
            # Reset to base pose, then apply sine on wrist_2 (index 4)
            self.q = self.base_rad.copy()

            if 0.0 <= cycle_time <= 4:
                local_t = cycle_time
                self.q[4] = self.base_rad[4] + self.amp1_rad * math.sin(
                    2.0 * math.pi * self.SINE1_FREQ_HZ * local_t
                )
            elif 4.0 <= cycle_time <= 6.0:
                self.q[4] = self.base_rad[4]
            else:
                local_t = cycle_time - 6.0
                self.q[4] = self.base_rad[4] + self.amp2_rad * math.sin(
                    2.0 * math.pi * self.SINE2_FREQ_HZ * local_t
                )

            self.pub_servo.publish(self._mk_joint_state(self.q))


def main(args=None):
    rclpy.init(args=args)
    node = Wrist2SinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()