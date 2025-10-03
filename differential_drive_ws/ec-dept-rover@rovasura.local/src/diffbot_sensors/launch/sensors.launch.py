#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Front ultrasonic sensor
        Node(
            package='diffbot_sensors',
            executable='ultrasonic_sensor',
            name='ultrasonic_front_node',
            parameters=[{
                'sensor_name': 'ultrasonic_front',
                'frame_id': 'ultrasonic_front',
                'min_range': 0.02,
                'max_range': 4.0,
                'field_of_view': 0.5,
                'publish_rate': 10.0
            }],
            output='screen'
        ),
        
        # Rear ultrasonic sensor
        Node(
            package='diffbot_sensors',
            executable='ultrasonic_sensor',
            name='ultrasonic_rear_node',
            parameters=[{
                'sensor_name': 'ultrasonic_rear',
                'frame_id': 'ultrasonic_rear',
                'min_range': 0.02,
                'max_range': 4.0,
                'field_of_view': 0.5,
                'publish_rate': 10.0
            }],
            output='screen'
        ),
        
        # Left ultrasonic sensor
        Node(
            package='diffbot_sensors',
            executable='ultrasonic_sensor',
            name='ultrasonic_left_node',
            parameters=[{
                'sensor_name': 'ultrasonic_left',
                'frame_id': 'ultrasonic_left',
                'min_range': 0.02,
                'max_range': 4.0,
                'field_of_view': 0.5,
                'publish_rate': 10.0
            }],
            output='screen'
        ),
        
        # Right ultrasonic sensor
        Node(
            package='diffbot_sensors',
            executable='ultrasonic_sensor',
            name='ultrasonic_right_node',
            parameters=[{
                'sensor_name': 'ultrasonic_right',
                'frame_id': 'ultrasonic_right',
                'min_range': 0.02,
                'max_range': 4.0,
                'field_of_view': 0.5,
                'publish_rate': 10.0
            }],
            output='screen'
        ),
        
        # Sensor aggregator
        Node(
            package='diffbot_sensors',
            executable='sensor_aggregator',
            name='sensor_aggregator_node',
            parameters=[{
                'obstacle_threshold': 0.5,
                'publish_rate': 10.0
            }],
            output='screen'
        ),
    ])