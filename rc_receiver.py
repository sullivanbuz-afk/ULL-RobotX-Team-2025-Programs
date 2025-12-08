#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class RCReceiverNode(Node):
    
    def __init__(self):
        super().__init__('rc_receiver')
        
        self.publisher_ = self.create_publisher(Int32MultiArray,'rc_channels',10)

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.timer=self.create_timer(0.05,self.read_serial)
        self.get_logger().info("RC Receiver Topic has been started")

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            
            if line:
                values = [int(v) for v in line.split(',')]

                msg = Int32MultiArray()
                msg.data = values
                self.publisher_.publish(msg)
                #self.get_logger().info(f"Published: {values}")
        except Exception as e:
            self.get_logger().warn(f"Serial Read Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RCReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    