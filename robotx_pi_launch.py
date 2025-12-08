#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='robotx_pi',
            executable='rc_receiver',
            name='RCReceiverNode',
            output='screen'
        ),

        Node(
            package='robotx_pi',
            executable='working_mode',
            name='WorkingModePublisher',
            output='screen'
        ),

        Node(
            package='robotx_pi',
            executable='remote_control',
            name='RemoteThrottleNode',
            output='screen'
        ),

        Node(
            package='robotx_pi',
            executable='servo_writer',
            name='ServoWriterNode',
            output='screen'
        ),

        Node(
            package='robotx_pi',
            executable='led_control',
            name='LEDControlNode',
            output='screen'
        )
        
    ])