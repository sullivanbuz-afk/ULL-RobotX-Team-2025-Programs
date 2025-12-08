#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO


class LEDControlNode(Node):
    def __init__(self):
        super().__init__('led_control')

        self.working_mode_subscriber_ = self.create_subscription(
            String, 'working_mode', self.mode_callback, 10)
        
        self.get_logger().info("LED Control Started")


        self.GREEN_relay_pin = 23
        self.YELLOW_relay_pin = 22
        self.RED_relay_pin = 24

        self.ACTIVE_LOW_G = True
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GREEN_relay_pin, GPIO.OUT,
                   initial=GPIO.HIGH if self.ACTIVE_LOW_G else GPIO.LOW)
        self.GREEN_relay(False)  # start with relay OFF

        self.ACTIVE_LOW_Y = True
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.YELLOW_relay_pin, GPIO.OUT,
                   initial=GPIO.HIGH if self.ACTIVE_LOW_Y else GPIO.LOW)
        self.YELLOW_relay(False)  # start with relay OFF

        self.ACTIVE_LOW_R = True
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RED_relay_pin, GPIO.OUT,
                   initial=GPIO.HIGH if self.ACTIVE_LOW_R else GPIO.LOW)
        self.RED_relay(False)  # start with relay OFF

    
    def GREEN_relay(self, on:bool):
        GPIO.output(
            self.GREEN_relay_pin,
            GPIO.LOW if (on and self.ACTIVE_LOW_G) else
            GPIO.HIGH if (on and not self.ACTIVE_LOW_G) else
            GPIO.HIGH if (not on and self.ACTIVE_LOW_G) else
            GPIO.LOW
        )

    def YELLOW_relay(self, on:bool):
        GPIO.output(
            self.YELLOW_relay_pin,
            GPIO.LOW if (on and self.ACTIVE_LOW_Y) else
            GPIO.HIGH if (on and not self.ACTIVE_LOW_Y) else
            GPIO.HIGH if (not on and self.ACTIVE_LOW_Y) else
            GPIO.LOW
        )

    def RED_relay(self, on:bool):
        GPIO.output(
            self.RED_relay_pin,
            GPIO.LOW if (on and self.ACTIVE_LOW_R) else
            GPIO.HIGH if (on and not self.ACTIVE_LOW_R) else
            GPIO.HIGH if (not on and self.ACTIVE_LOW_R) else
            GPIO.LOW
        )

    def mode_callback(self,msg):
        self.working_mode = msg.data

        if self.working_mode == "REMOTE CONTROL":
            self.GREEN_relay(True)
            self.YELLOW_relay(True)
            self.RED_relay(False)

        elif self.working_mode == "STANDBY":
            self.GREEN_relay(False)
            self.YELLOW_relay(True)
            self.RED_relay(False)

        elif self.working_mode == "AUTONOMOUS":
            self.GREEN_relay(True)
            self.YELLOW_relay(False)
            self.RED_relay(False)

        else:
            self.GREEN_relay(False)
            self.YELLOW_relay(False)
            self.RED_relay(True)

    def destroy_node(self):
        self.GREEN_relay(False)
        self.YELLOW_relay(False)
        self.RED_relay(False)
        GPIO.output(self.GREEN_relay_pin, GPIO.HIGH if self.ACTIVE_LOW_G else GPIO.LOW)
        GPIO.output(self.YELLOW_relay_pin, GPIO.HIGH if self.ACTIVE_LOW_Y else GPIO.LOW)
        GPIO.output(self.RED_relay_pin, GPIO.HIGH if self.ACTIVE_LOW_R else GPIO.LOW)
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = LEDControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
