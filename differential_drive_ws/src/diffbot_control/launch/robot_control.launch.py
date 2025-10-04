#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Whether to run in simulation mode (no GPIO)'
    )
    
    max_linear_velocity_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='1.0',
        description='Maximum linear velocity in m/s'
    )
    
    max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='2.0',
        description='Maximum angular velocity in rad/s'
    )

    # Motor driver node
    motor_driver_node = Node(
        package='diffbot_control',
        executable='motor_driver',
        name='motor_driver',
        parameters=[{
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'rc1_pin': 18,
            'rc2_pin': 19,
            'pwm_frequency': 50
        }],
        output='screen'
    )

    # Robot controller service node
    robot_controller_node = Node(
        package='diffbot_control',
        executable='robot_controller',
        name='robot_controller',
        output='screen'
    )

    return LaunchDescription([
        use_simulation_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        LogInfo(
            msg=['Starting differential drive robot control with max_linear_vel=', 
                 LaunchConfiguration('max_linear_velocity'),
                 ', max_angular_vel=',
                 LaunchConfiguration('max_angular_velocity')]
        ),
        motor_driver_node,
        robot_controller_node,
    ])