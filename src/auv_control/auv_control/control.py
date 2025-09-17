#!/usr/bin/env python3

import pygame
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Initialize Pygame without opening a window
pygame.init()
pygame.display.set_mode((1, 1))


class CommandPub(Node):
    def __init__(self):
        super().__init__("Command_Pub_Node")

        self.declare_parameter("topic", value="command")
        topic_name = self.get_parameter("topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(Int32, topic_name, 10)
        self.shutdown_flag = False
    def publish(self, detection_msg):
        msg = Int32()
        msg.data = detection_msg
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
    def kill_node(self):  
        self.get_logger().info("Killing Node")
        rclpy.shutdown()

def init_ros2():
    rclpy.init()
    return CommandPub()
# Define key commands as integers (or you can use other types/messages)
def main():
    flag_comm = 0
    flag_stop_comm = 0
    command_pub = init_ros2()
    FORWARD = 1
    BACKWARD = 4
    LATERAL_LEFT = 2
    LATERAL_RIGHT = 3
    YAW_LEFT = 69
    YAW_RIGHT = 96
    DEPTH_UP = 7
    DEPTH_DOWN = 6
    STOP = 5
    KILL = 123
    START = 37
    RESET=789
    RECALIBRATE = 456
    GRIP_DOWN1 = 58
    GRIP_UP1 = 85
    GRIP_DOWN2 = 67
    GRIP_UP2 = 76

    flag_ks = 1
    flag_stop=0
    # Main loop
    running = True
    while running :
        # Process events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Check key states
        keys = pygame.key.get_pressed()

        # Track if any key is pressed
        key_pressed = False

        
        if keys[pygame.K_w]:
            command_pub.publish(FORWARD)
            print("Forward")
            key_pressed = True
            flag_stop=0
            flag_comm = 1
            flag_stop_comm = 0
            time.sleep(0.05)
        elif keys[pygame.K_s]:
            command_pub.publish(BACKWARD)
            print("Backward")
            key_pressed = True
            flag_stop=0
            flag_comm = 1
            flag_stop_comm = 0
            time.sleep(0.05)
        elif keys[pygame.K_a]:
            command_pub.publish(LATERAL_LEFT)
            print("Lateral Left")
            key_pressed = True
            flag_stop=0
            flag_comm = 1
            flag_stop_comm = 0
            time.sleep(0.05)
        elif keys[pygame.K_d]:
            command_pub.publish(LATERAL_RIGHT)
            print("Lateral Right")
            key_pressed = True
            flag_stop=0
            flag_comm = 1
            flag_stop_comm = 0
            time.sleep(0.05)
        elif keys[pygame.K_q]:
            command_pub.publish(YAW_LEFT)
            print("Yaw Left")
            key_pressed = True
            flag_stop=0
            flag_stop_comm = 0
        elif keys[pygame.K_e]:
            command_pub.publish(YAW_RIGHT)
            print("Yaw Right")
            key_pressed = True
            flag_stop=0
            flag_stop_comm = 0
        elif keys[pygame.K_c]:
            command_pub.publish(36)
            print("Yaw Right")
            key_pressed = True
            flag_stop=0
            flag_stop_comm = 0
            time.sleep(1)
        elif keys[pygame.K_UP]:
            if flag_comm == 0:
                command_pub.publish(DEPTH_UP)
                print("Depth Up")
                key_pressed = True
            flag_stop = 0
            flag_comm = 1
            flag_stop_comm = 0
        elif keys[pygame.K_DOWN]:
            if flag_comm == 0:
                command_pub.publish(DEPTH_DOWN)
                print("Depth Down")
            key_pressed = True
            flag_stop=0
            flag_comm = 1
            flag_stop_comm = 0
        elif keys[pygame.K_LEFT]:
            if flag_comm == 0:
                command_pub.publish(GRIP_UP1)
                print("Grip Up")
                key_pressed = True
            flag_stop = 0
            flag_comm = 1
            flag_stop_comm = 0
        elif keys[pygame.K_RIGHT]:
            if flag_comm == 0:
                command_pub.publish(GRIP_DOWN1)
                print("Grip Down")
            key_pressed = True
            flag_stop=0
            flag_comm = 1
            flag_stop_comm = 0
        elif keys[pygame.K_j]:
            if flag_comm == 0:
                command_pub.publish(GRIP_UP2)
                print("Grip Up")
                key_pressed = True
            flag_stop = 0
            flag_comm = 1
            flag_stop_comm = 0
        elif keys[pygame.K_l]:
            if flag_comm == 0:
                command_pub.publish(GRIP_DOWN2)
                print("Grip Down")
            key_pressed = True
            flag_stop=0
            flag_comm = 1
            flag_stop_comm = 0
            
        elif keys[pygame.K_x] and flag_ks % 2 == 1:
            command_pub.publish(KILL)
            print("Kill")
            key_pressed = True   
            flag_ks += 1
            flag_stop_comm = 0
            time.sleep(2)
        elif keys[pygame.K_x] and flag_ks % 2 == 0:
            command_pub.publish(START)
            print("Start")
            key_pressed = True   
            flag_ks += 1
            flag_stop_comm = 0
            time.sleep(2)
        elif keys[pygame.K_z]:
            command_pub.publish(RESET)
            print("Reset")
            key_pressed = True
            flag_stop=0
            flag_stop_comm = 0
        elif keys[pygame.K_r]:
            command_pub.publish(RECALIBRATE)
            print("Recalibrate")
            key_pressed = True
            flag_stop=0
            flag_stop_comm = 0
            

        # If no key is pressed, publish "Stop"
        
        if not key_pressed:
            if flag_stop == 5:
                if flag_stop_comm == 0:    
                    command_pub.publish(STOP)
                    print("Stop") 
                flag_stop_comm = 1
                flag_comm = 0
            else:
                flag_stop+=1
        
        # Sleep to prevent too fast printing
        time.sleep(0.05)  # Adjust the sleep time as needed

    # Quit Pygame and shutdown ROS
    pygame.quit()
    sys.exit()

if __name__=='__main__':
    main()