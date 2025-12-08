#!/usr/bin/env python3 

import rclpy 
import time
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from adafruit_servokit import ServoKit

class ThrottlePower(Node):
    
    def __init__(self):

        super().__init__('throttle_power')

        self.throttle_power_subscriber = self.create_subscription(
            Int32MultiArray, 'rc_channels', self.servo_callback,10)
        
        self.get_logger().info('Throttles being turned on')

        self.kit = ServoKit(channels = 16)
        self.kit.servo[4].set_pulse_width_range(800.2100)
        self.kit.servo[4].actuation_range = 180
        self.kit.servo[8].set_pulse_width_range(800,2100)
        self.kit.servo[8].actuation_range = 180
        
        

    def servo_callback(self,msg):

        self.ch5_val = msg.data[4]

        if self.ch5_val < 1450: # Left servo press
            self.kit.servo[4].angle = 90 
            time.sleep(3)
            self.kit.servo[4].angle = 56
            time.sleep(1.5)

        elif self.ch5_val > 1550: # Right Servo Press
            self.kit.servo[8].angle = 90
            time.sleep(3)
            self.kit.servo[8].angle = 62
            time.sleep(1.5)

        else:
            self.kit.servo[4].angle = 90
            self.kit.servo[8].angle = 90

def main(args=None):
    rclpy.init(args=args)
    node = ThrottlePower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()