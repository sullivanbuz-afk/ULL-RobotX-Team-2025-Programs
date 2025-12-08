#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray



class WorkingModePublisher(Node):

    def __init__(self):

        super().__init__('mode_publisher')

        self.rc_subscriber_ = self.create_subscription(Int32MultiArray, 
                        'rc_channels', self.mode_callback, 10)


        self.mode_publisher_ = self.create_publisher(String, 
                                'working_mode', 10)

    
        
        self.get_logger().info("Working Mode Topic has started...")

    

        self.working_mode = "STANDBY"

    def mode_callback(self,msg):
        try:

            ch4_value = msg.data[3]
            ch3_value = msg.data[2]
            ch2_value = msg.data[1]
            ch1_value = msg.data[0]

            if ch1_value < 1000:
                self.working_mode = "EMERGENCY STOP"
                #self.get_logger().warn("REMOTE NOT CONNECTED")

            else:
                
                if ch3_value == 0:
                    self.working_mode = "EMERGENCY STOP"
                    #self.get_logger().warn("REMOTE NOT CONNECTED")
                elif 0 < ch3_value < 1500:
                    if ch4_value < 1200:
                        self.working_mode = "STANDBY"
                        #Commands for standby mode

                    elif 1400 < ch4_value < 1600:
                        self.working_mode = "REMOTE CONTROL"
                        #Command for servo movement based on CH1 and Ch2 Values

                    else:
                        self.working_mode = "AUTONOMOUS"
                        #Autonomous 
                    
                else:
                    self.working_mode = "EMERGENCY STOP"
                    #kill

            mode_msg = String()
            mode_msg.data = self.working_mode
            self.mode_publisher_.publish(mode_msg)

        except IndexError:
            self.get_logger().warn("Received bad msg. Try again Stupid")
                


    




def main(args=None):
    rclpy.init(args=args)
    node = WorkingModePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()