#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    use_single_sensors_arg = DeclareLaunchArgument(
        'use_single_sensors',
        default_value='false',
        description='Use individual sensor nodes instead of multi-sensor node'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Sensor publish rate in Hz'
    )
    
    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='0.02',
        description='Minimum sensor range in meters'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='4.0',
        description='Maximum sensor range in meters'
    )
    
    # Get launch configurations
    use_single_sensors = LaunchConfiguration('use_single_sensors')
    publish_rate = LaunchConfiguration('publish_rate')
    min_range = LaunchConfiguration('min_range')
    max_range = LaunchConfiguration('max_range')
    
    # Multi-sensor node (default) - now configured for single front sensor
    multi_ultrasonic_node = Node(
        package='diffbot_sensors',
        executable='multi_ultrasonic',
        name='front_ultrasonic_node',
        output='screen',
        parameters=[{
            'publish_rate': publish_rate,
            'min_range': min_range,
            'max_range': max_range,
            'timeout_ms': 20,
            'sensor_delay_us': 1000,
        }],
        condition=IfCondition(['not ', use_single_sensors])
    )
    
    # Individual sensor nodes (alternative) - only front sensor configured
    single_sensors_group = GroupAction([
        Node(
            package='diffbot_sensors',
            executable='ultrasonic_sensor',
            name='ultrasonic_front_node',
            output='screen',
            parameters=[{
                'sensor_name': 'ultrasonic_front',
                'frame_id': 'ultrasonic_front_link',
                'trig_pin': 23,
                'echo_pin': 24,
                'publish_rate': publish_rate,
                'min_range': min_range,
                'max_range': max_range,
            }]
        )
    ], condition=IfCondition(use_single_sensors))
    
    # Sensor aggregator node - configured for front sensor only
    sensor_aggregator_node = Node(
        package='diffbot_sensors',
        executable='sensor_aggregator',
        name='sensor_aggregator_node',
        output='screen',
        parameters=[{
            'obstacle_threshold': 0.3,  # 30cm obstacle threshold
            'sensor_topics': [
                '/ultrasonic_front/range'
            ]
        }]
    )
    
    return LaunchDescription([
        use_single_sensors_arg,
        publish_rate_arg,
        min_range_arg,
        max_range_arg,
        multi_ultrasonic_node,
        single_sensors_group,
        sensor_aggregator_node,
    ])