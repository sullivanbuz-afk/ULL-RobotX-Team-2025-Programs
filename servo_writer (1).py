#!/usr/bin/env python3 

import rclpy 
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from adafruit_servokit import ServoKit



class ServoWriter(Node):

    def __init__(self):
        
        super().__init__('servo_writer')

        self.working_mode_subscriber = self.create_subscription(
            String, 'working_mode', self.mode_callback, 10)
        self.angle_subscriber = self.create_subscription(
            Int32MultiArray, 'servo_angles', self.angle_callback, 10)
        
        self.get_logger().info("Servo Writer has been started")

        self.kit = ServoKit(channels=16)
        self.kit.servo[0].set_pulse_width_range(800,2100)
        self.kit.servo[0].actuation_range = 180
        self.kit.servo[15].set_pulse_width_range(800,2100)
        self.kit.servo[15].actuation_range = 180
        self.working_mode = "STANDBY"

    def mode_callback(self,msg):
        self.working_mode = msg.data

    def angle_callback(self,msg):

        self.remote_left = msg.data[0]
        self.remote_right = msg.data[1]
        self.auto_left = msg.data[2]
        self.auto_right = msg.data[3]

        if self.working_mode == "REMOTE CONTROL" or self.working_mode == "EMERGENCY STOP" or self.working_mode == "STANDBY":
            self.kit.servo[0].angle = self.remote_left
            self.kit.servo[15].angle = self.remote_right

        elif self.working_mode == "AUTONOMOUS":
            self.kit.servo[0].angle = self.auto_left
            self.kit.servo[15].angle = self.auto_right

        else:
            self.kit.servo[0].angle = 80
            self.kit.servo[15].angle = 80

def main():
    rclpy.init()
    node = ServoWriter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()