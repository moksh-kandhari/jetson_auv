#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from adafruit_servokit import ServoKit
import time


class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')

        # === Setup ServoKit (16 channels on PCA9685) ===
        self.kit = ServoKit(channels=16)

        # Assign thrusters to channels (example: 8 thrusters on 2,3,10–15)
        self.thruster_channels = [13, 10, 11, 14, 15, 2, 3, 12]

        # Configure ESC range (T200 likes ~1100–1900 µs, 1500 neutral)
        self.kit.continuous_servo[self.thruster_channels[0]].set_pulse_width_range(1112, 1912)
        self.kit.continuous_servo[self.thruster_channels[1]].set_pulse_width_range(1112, 1912)
        self.kit.continuous_servo[self.thruster_channels[2]].set_pulse_width_range(1112, 1912)
        self.kit.continuous_servo[self.thruster_channels[3]].set_pulse_width_range(1112, 1912)
        self.kit.continuous_servo[self.thruster_channels[4]].set_pulse_width_range(1112, 1912)
        self.kit.continuous_servo[self.thruster_channels[5]].set_pulse_width_range(1112, 1912)
        self.kit.continuous_servo[self.thruster_channels[6]].set_pulse_width_range(1112, 1912)
        self.kit.continuous_servo[self.thruster_channels[7]].set_pulse_width_range(1112, 1912)

        # Arming sequence (7 sec at neutral 1500 µs)
        self.get_logger().info("Arming ESCs (neutral signal for 7 seconds)...")
        self.kit.continuous_servo[self.thruster_channels[0]].throttle = 0  # maps close to 1500 µs
        self.kit.continuous_servo[self.thruster_channels[1]].throttle = 0  # maps close to 1500 µs
        self.kit.continuous_servo[self.thruster_channels[2]].throttle = 0  # maps close to 1500 µs
        self.kit.continuous_servo[self.thruster_channels[3]].throttle = 0  # maps close to 1500 µs
        self.kit.continuous_servo[self.thruster_channels[4]].throttle = 0  # maps close to 1500 µs
        self.kit.continuous_servo[self.thruster_channels[5]].throttle = 0  # maps close to 1500 µs
        self.kit.continuous_servo[self.thruster_channels[6]].throttle = 0  # maps close to 1500 µs
        self.kit.continuous_servo[self.thruster_channels[7]].throttle = 0  # maps close to 1500 µs
        time.sleep(7)
        self.get_logger().info("ESCs armed. Ready for PID + CMD input.")

        # Subscribe to PID outputs
        self.create_subscription(Float32MultiArray, 'pid_out', self.pid_callback, 10)

        # Subscribe to Command input (single integer)
        self.create_subscription(Int32, 'command', self.cmd_callback, 10)

        # Store last received command
        self.cmd = 0

    def pwm_to_throttle(self, pwm: float) -> float:
        """
        Convert PWM µs value (around 1500 neutral) → [-1, 1] throttle.
        """
        # Clamp PWM
        if pwm > 400:
            pwm = 400
        elif pwm < -400:
            pwm = -400
        return pwm / 400.0

    def cmd_callback(self, msg: Int32):
        """Receive integer command and store in self.cmd"""
        self.cmd = msg.data

    def pid_callback(self, msg: Float32MultiArray):
        speed = 60
        if len(msg.data) < 4:
            self.get_logger().warn("PID output does not have 4 values!")
            # reset thrusters to neutral
            for ch in self.thruster_channels:
                self.kit.servo[ch].throttle = 0
            return

        out_yaw, out_pitch, out_roll, out_depth = msg.data

        # Base command for thrusters depending on received cmd
        if self.cmd == 1:
            pwm_cmd = [speed, speed, -speed, speed, speed, speed, -speed, speed]
        elif self.cmd == 4:
            pwm_cmd = [-speed, -speed, speed, -speed, -speed, -speed, speed, -speed]
        elif self.cmd == 2:
            pwm_cmd = [-speed, speed, speed, speed, -speed, speed, speed, speed]
        elif self.cmd == 3:
            pwm_cmd = [speed, -speed, -speed, -speed, speed, -speed, -speed, -speed]
        elif self.cmd == 5:
            pwm_cmd = [0, 0, 0, 0, 0, 0, 0, 0]
        else:
            pwm_cmd = [0, 0, 0, 0, 0, 0, 0, 0]

        # Combine PID output with command
        pwm1 = -out_yaw - out_pitch + out_roll + out_depth - pwm_cmd[0]
        pwm2 = out_yaw - out_pitch - out_roll + out_depth - pwm_cmd[1]
        pwm3 = -out_yaw + out_pitch - out_roll + out_depth - pwm_cmd[2]
        pwm4 = out_yaw + out_pitch + out_roll + out_depth + pwm_cmd[3]
        pwm5 = -out_yaw + out_pitch - out_roll - out_depth - pwm_cmd[4]
        pwm6 = -out_yaw - out_pitch - out_roll + out_depth + pwm_cmd[5]
        pwm7 = -out_yaw - out_pitch + out_roll - out_depth - pwm_cmd[6]
        pwm8 = -out_yaw + out_pitch + out_roll + out_depth - pwm_cmd[7]
            

        # Send PWM → throttle
        
        
        self.kit.continuous_servo[self.thruster_channels[0]].throttle = self.pwm_to_throttle(pwm1)
        self.kit.continuous_servo[self.thruster_channels[1]].throttle = self.pwm_to_throttle(pwm2)
        self.kit.continuous_servo[self.thruster_channels[2]].throttle = self.pwm_to_throttle(pwm3)
        self.kit.continuous_servo[self.thruster_channels[3]].throttle = self.pwm_to_throttle(pwm4)
        self.kit.continuous_servo[self.thruster_channels[4]].throttle = self.pwm_to_throttle(pwm5)
        self.kit.continuous_servo[self.thruster_channels[5]].throttle = self.pwm_to_throttle(pwm6)
        self.kit.continuous_servo[self.thruster_channels[6]].throttle = self.pwm_to_throttle(pwm7)
        self.kit.continuous_servo[self.thruster_channels[7]].throttle = self.pwm_to_throttle(pwm8)




def main(args=None):
    rclpy.init(args=args)
    node = ThrusterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

