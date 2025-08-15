from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('skid_steer_robot')
    
    # Paths to files
    world_file = os.path.join(pkg_share, 'worlds', 'world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'skid_steer.urdf.xacro')
    config_file = os.path.join(pkg_share, 'config', 'ros2_control.yaml')
    
    # Start Gazebo with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': f"-r {world_file}"}.items(),
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Spawn the robot using URDF
    spawn_robot = Node(
        package='ros_gz',
        executable='create',
        arguments=[
            '-name', 'skid_steer_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file],
        output='screen',
        remappings=[('joint_states', '/joint_states')]
    )

    # Teleop node
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        prefix='xterm -e',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        controller_manager,
        teleop
    ])
