#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class AngleCombiner(Node):

    def __init__(self):


        super().__init__('angle_combiner')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.remote_subscriber = self.create_subscription(Int32MultiArray, 'remote_angle', self.remote_callback, qos)
        self.auto_subscriber = self.create_subscription(Int32MultiArray, 'auto_angle', self.auto_callback, qos)
        self.angle_pub = self.create_publisher(Int32MultiArray, 'servo_angles', qos)

        self.get_logger().info("Angle Combiner has started")


        self.remote_left = 80
        self.remote_right = 80
        self.auto_left = 80
        self.auto_right = 80


    def remote_callback(self,msg):
        self.remote_left = int(msg.data[0])
        self.remote_right = int(msg.data[1])
        self.try_publish()

    def auto_callback(self,msg):
        self.auto_left = int(msg.data[0])
        self.auto_right = int(msg.data[1])

        

        self.try_publish()


    def try_publish(self):

        if None in (self.remote_left, self.remote_right, self.auto_left, self.auto_right):
            return
        
        out = Int32MultiArray()
        out.data = [self.remote_left, self.remote_right, self.auto_left, self.auto_right]
        self.angle_pub.publish(out)

def main():
    rclpy.init()
    node = AngleCombiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()