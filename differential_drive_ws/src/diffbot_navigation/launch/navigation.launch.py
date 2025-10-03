#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch arguments
    behavior_mode_arg = DeclareLaunchArgument(
        'behavior_mode',
        default_value='obstacle_avoidance',
        description='Navigation behavior mode: obstacle_avoidance or wander'
    )
    
    behavior_mode = LaunchConfiguration('behavior_mode')
    
    return LaunchDescription([
        behavior_mode_arg,
        
        # Obstacle avoidance node (default)
        Node(
            package='diffbot_navigation',
            executable='obstacle_avoidance',
            name='obstacle_avoidance_node',
            parameters=[{
                'linear_speed': 0.5,
                'angular_speed': 0.8,
                'obstacle_distance': 0.7,
                'use_laser': True
            }],
            output='screen',
            condition=lambda context: context.perform_substitution(behavior_mode) == 'obstacle_avoidance'
        ),
        
        # Wander behavior node (alternative)
        Node(
            package='diffbot_navigation',
            executable='wander_behavior',
            name='wander_behavior_node',
            parameters=[{
                'linear_speed': 0.3,
                'angular_speed': 0.5,
                'wander_timeout': 5.0,
                'turn_timeout': 2.0
            }],
            output='screen',
            condition=lambda context: context.perform_substitution(behavior_mode) == 'wander'
        ),
    ])