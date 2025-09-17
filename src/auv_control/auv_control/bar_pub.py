#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import ms5837
import time

class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')

        # Publisher (publishes depth in meters as Float32)
        self.publisher_ = self.create_publisher(Float32, 'depth', 10)

        # Timer to publish at 2 Hz
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize sensor
        self.sensor = ms5837.MS5837_30BA()
        if not self.sensor.init():
            self.get_logger().error("MS5837 sensor could not be initialized")
            exit(1)

        # Set fluid density (freshwater = 1000, saltwater = ms5837.DENSITY_SALTWATER)
        self.sensor.setFluidDensity(1000)

        self.get_logger().info("DepthPublisher node started, publishing on 'depth' topic")

    def timer_callback(self):
        if self.sensor.read():
            depth = self.sensor.depth()  # in meters
            msg = Float32()
            msg.data = depth
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing Depth: {depth:.3f} m")
        else:
            self.get_logger().warn("Sensor read failed!")


def main(args=None):
    rclpy.init(args=args)
    node = DepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

