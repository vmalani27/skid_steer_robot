from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('skid_steer_robot')
    
    # Path to world file
    world_file = os.path.join(pkg_share, 'worlds', 'world.sdf')
    
    # Start Gazebo with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': f"-r {world_file}"}.items(),
    )

    return LaunchDescription([gazebo])
