#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Package Directories
    pkg_diffbot_description = get_package_share_directory('diffbot_description')
    pkg_diffbot_bringup = get_package_share_directory('diffbot_bringup')
    
    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    world = LaunchConfiguration('world')

    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_diffbot_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'diffbot',
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', '0.01'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_diffbot_bringup, 'worlds', 'diffbot_world.world'), ''],
            description='SDF world file'),
        DeclareLaunchArgument('x_pose', default_value='0.0',
                              description='x pose'),
        DeclareLaunchArgument('y_pose', default_value='0.0',
                              description='y pose'),
        
        robot_state_publisher,
        gazebo,
        spawn_entity,
    ])