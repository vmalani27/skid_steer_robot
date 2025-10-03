#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    
    # Package Directories
    pkg_diffbot_description = get_package_share_directory('diffbot_description')
    
    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_diffbot_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Joint State Publisher (for manual control of joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Static transform publishers for sensor frames
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'laser', 'base_laser_link']
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        static_tf_laser,
    ])