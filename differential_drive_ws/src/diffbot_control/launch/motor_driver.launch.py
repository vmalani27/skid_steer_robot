#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diffbot_control',
            executable='motor_driver',
            name='motor_driver_node',
            output='screen',
            parameters=[{
                'rc1_pin': 18,          # Left motor group GPIO pin
                'rc2_pin': 19,          # Right motor group GPIO pin
                'pwm_frequency': 50,    # PWM frequency in Hz
                'max_linear_velocity': 1.0,   # Maximum linear velocity (m/s)
                'max_angular_velocity': 1.0,  # Maximum angular velocity (rad/s)
            }],
            remappings=[
                ('/cmd_vel', '/diffbot/cmd_vel'),
            ]
        ),
    ])