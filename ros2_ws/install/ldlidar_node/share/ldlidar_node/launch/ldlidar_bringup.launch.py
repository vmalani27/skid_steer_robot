# Copyright 2022 Walter Lucetti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###########################################################################

import os
from re import L

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    return_array = []

    # Define LaunchDescription variable
    ld = LaunchDescription()

    node_ns = LaunchConfiguration('node_namespace')
    node_name = LaunchConfiguration('node_name')
    container_name = LaunchConfiguration('container_name')

    # Lidar node configuration file
    lidar_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'ldlidar.yaml'
    )

    # Handle Parameter values
    node_namespace_val = node_ns.perform(context)
    node_name_val = node_name.perform(context)
    container_name_val = container_name.perform(context)

    if node_namespace_val != '':
        node_namespace_val = '/' + node_namespace_val

        # URDF path
    urdf_file_name = 'ldlidar_descr.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=node_ns,
        name='ldlidar_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )
    return_array.append(rsp_node)

    # LDLidar component if required
    if container_name_val=='':
        container_name_val = 'ldlidar_container'
        # ROS 2 Component Container
        distro = os.environ['ROS_DISTRO']
        if distro == 'foxy':
            # Foxy does not support the isolated mode
            container_exec='component_container'
        else:
            container_exec='component_container_isolated'
        
        ldlidar_container = ComposableNodeContainer(
                name=container_name_val,
                namespace=node_ns,
                package='rclcpp_components',
                executable=container_exec,
                composable_node_descriptions=[
                ],
                output='screen',
        )
        return_array.append(ldlidar_container)
    
    # LDLidar component
    ldlidar_component = ComposableNode(
            package='ldlidar_component',
            namespace=node_ns,
            plugin='ldlidar::LdLidarComponent',
            name=node_name,
            parameters=[
                # YAML files
                lidar_config_path  # Parameters
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    
    # LDLidar Lifecycle node in container
    full_container_name = node_namespace_val + '/' + container_name_val
    info = '* Loading node: ' + node_name_val + ' in container: ' + full_container_name
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[ldlidar_component]
    )
    return_array.append(load_composable_node)

    return return_array



def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
            'node_namespace',
            default_value='',
            description='Namespace of the node'
            ),
            DeclareLaunchArgument(
                'node_name',
                default_value='ldlidar_node',
                description='Name of the node'
            ),
            DeclareLaunchArgument(
                'container_name',
                default_value='',
                description='Name of an exister container to load the Lidar component. If empty a new container will be created.'
            ),
            OpaqueFunction(function=launch_setup)
        ]
    )
