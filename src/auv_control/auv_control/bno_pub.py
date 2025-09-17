import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import time
import board, busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR


def quaternion_to_euler(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = math.degrees(math.atan2(t3, t4))

    return yaw, pitch, roll


class BNO085Publisher(Node):
    def __init__(self):
        super().__init__('bno085_publisher')

        # ROS2 publisher (Float32MultiArray: [yaw, pitch, roll])
        self.publisher_ = self.create_publisher(Float32MultiArray, 'bno085/ypr', 10)

        # Initialize I2C + BNO085
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = BNO08X_I2C(i2c)
        self.sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        # Timer for publishing at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        quat = self.sensor.quaternion  # (i, j, k, real)
        if quat:
            yaw, pitch, roll = quaternion_to_euler(quat[3], quat[0], quat[1], quat[2])
            msg = Float32MultiArray()
            msg.data = [yaw, pitch, roll]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Yaw={yaw:.2f}, Pitch={pitch:.2f}, Roll={roll:.2f}")
        else:
            self.get_logger().warn("Quaternion not available")


def main(args=None):
    rclpy.init(args=args)
    node = BNO085Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

