#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('diffbot_sensors')
    
    # Declare launch arguments
    use_config_file_arg = DeclareLaunchArgument(
        'use_config_file',
        default_value='true',
        description='Whether to use config file parameters'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='15.0',
        description='Sensor publishing rate in Hz'
    )
    
    trig_pin_arg = DeclareLaunchArgument(
        'trig_pin',
        default_value='23',
        description='TRIG pin number (BCM)'
    )
    
    echo_pin_arg = DeclareLaunchArgument(
        'echo_pin',
        default_value='24',
        description='ECHO pin number (BCM)'
    )

    # Configuration file path
    config_file = os.path.join(pkg_dir, 'config', 'front_sensor.yaml')
    
    # Node parameters - either from config file or launch arguments
    node_params = []
    use_config = LaunchConfiguration('use_config_file')
    
    # Always include config file if it exists, launch args will override
    if os.path.exists(config_file):
        node_params.append(config_file)
    
    # Add launch arguments as parameters
    node_params.extend([
        {'rate': LaunchConfiguration('rate')},
        {'trig_pin': LaunchConfiguration('trig_pin')},
        {'echo_pin': LaunchConfiguration('echo_pin')},
    ])

    # Front ultrasonic sensor node
    front_ultrasonic_node = Node(
        package='diffbot_sensors',
        executable='front_ultrasonic_node.py',
        name='front_ultrasonic_node',
        output='screen',
        parameters=node_params,
        remappings=[
            ('/ultrasonic_front/range', '/sensors/ultrasonic_front/range'),
        ]
    )

    return LaunchDescription([
        use_config_file_arg,
        rate_arg,
        trig_pin_arg,
        echo_pin_arg,
        front_ultrasonic_node,
    ])