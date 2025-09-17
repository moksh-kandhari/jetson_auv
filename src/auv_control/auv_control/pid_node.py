#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import time

class ControlComputationNode(Node):
    def __init__(self):
        super().__init__('control_computation_node')

        # Subscribers
        self.create_subscription(Float32MultiArray, 'bno085/ypr', self.ypr_callback, 10)
        self.create_subscription(Float32, 'depth', self.depth_callback, 10)

        # Publisher (publishes 4 computed values)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'pid_out', 10)

        # Store latest + initial sensor values
        self.latest_ypr = None     # [yaw, pitch, roll]
        self.latest_depth = None   # depth in meters
        self.initial_ypr = None    # first received yaw, pitch, roll
        self.initial_depth = None  # first received depth
        
        self.last_time = 0
        self.now = 0
        
        self.integral_yaw = 0
        self.integral_pitch = 0
        self.integral_roll = 0
        self.integral_depth = 0

        self.yaw_prev = 0
        self.pitch_prev = 0
        self.roll_prev = 0
        self.depth_prev = 0
        
        self.set_point_depth = 0.2

        # Timer for publishing at 5 Hz
        self.timer = self.create_timer(0.2, self.timer_callback)

    def ypr_callback(self, msg):
        self.latest_ypr = msg.data  # [yaw, pitch, roll]

        # Store initial YPR once
        if self.initial_ypr is None:
            self.initial_ypr = msg.data
            self.get_logger().info(f"Stored initial YPR: {self.initial_ypr}")

    def depth_callback(self, msg):
        self.latest_depth = msg.data

        # Store initial depth once
        if self.initial_depth is None:
            self.initial_depth = msg.data
            self.get_logger().info(f"Stored initial Depth: {self.initial_depth:.3f}")

    def timer_callback(self):
        if (self.latest_ypr is not None and self.latest_depth is not None and
            self.initial_ypr is not None and self.initial_depth is not None):
        
            kp_yaw, ki_yaw, kd_yaw = 0,0,0#1.5, 0.12, 0
            kp_pitch, ki_pitch, kd_pitch = 3.5, 0, 0
            kp_roll, ki_roll, kd_roll = 6, 0, 0
            kp_depth, ki_depth, kd_depth = 600, 0, 0

            self.now = time.time()
            dt = (self.now - self.last_time) if self.last_time != 0 else 0.01
            self.last_time = self.now

            yaw, pitch, roll = self.latest_ypr
            depth = self.latest_depth

            yaw0, pitch0, roll0 = self.initial_ypr
            depth0 = self.initial_depth

            # Errors relative to initial
            error_yaw = yaw - yaw0
            error_pitch = pitch - pitch0
            error_roll = roll - roll0
            error_depth = depth - depth0 - self.set_point_depth

            # PID calculations
            self.integral_yaw += error_yaw * dt
            differential_yaw = (error_yaw - self.yaw_prev) / dt

            self.integral_pitch += error_pitch * dt
            differential_pitch = (error_pitch - self.pitch_prev) / dt

            self.integral_roll += error_roll * dt
            differential_roll = (error_roll - self.roll_prev) / dt

            self.integral_depth += error_depth * dt
            differential_depth = (error_depth - self.depth_prev) / dt

            self.yaw_prev = error_yaw
            self.pitch_prev = error_pitch
            self.roll_prev = error_roll
            self.depth_prev = error_depth

            output_yaw = kp_yaw*error_yaw + ki_yaw*self.integral_yaw + kd_yaw*differential_yaw
            output_pitch = kp_pitch*error_pitch + ki_pitch*self.integral_pitch + kd_pitch*differential_pitch
            output_roll = kp_roll*error_roll + ki_roll*self.integral_roll + kd_roll*differential_roll
            output_depth = kp_depth*error_depth + ki_depth*self.integral_depth + kd_depth*differential_depth

            # Publish results as [yaw, pitch, roll, depth]
            msg = Float32MultiArray()
            msg.data = [output_yaw, output_pitch, output_roll, output_depth]
            self.publisher_.publish(msg)

            self.get_logger().info(
                f"Published ΔOutput: "
                f"[Yaw={output_yaw:.2f}, Pitch={output_pitch:.2f}, "
                f"Roll={output_roll:.2f}, Depth={output_depth:.2f}]"
            )

    def send_zero_output(self):
        """Send [0,0,0,0] to stop motors safely"""
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.publisher_.publish(msg)
        self.get_logger().info("Sent shutdown command: [0,0,0,0]")
        time.sleep(0.1)  # allow message to be sent before shutdown


def main(args=None):
    rclpy.init(args=args)
    node = ControlComputationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_zero_output()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

