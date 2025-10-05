#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diffbot_control',
            executable='simple_motor_driver',
            name='simple_motor_driver',
            output='screen',
            parameters=['/home/rooster/skid_steer_robot/differential_drive_ws/src/diffbot_control/config/motor_driver_4.yaml'],
            remappings=[('/cmd_vel', '/diffbot/cmd_vel')],
        )
    ])
