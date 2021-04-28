# Copyright 2020 RT Corporation
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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():
    declare_lidar = DeclareLaunchArgument(
        'lidar', default_value='lds',
        description='Set LiDAR name: [lds, urg]'
    )

    slam_node = Node(
        package='slam_toolbox', node_executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[
            get_package_share_directory(
                'raspimouse_ros2_examples')
            + '/config/mapper_params_offline.yaml'
        ],
    )

    rviz2_node = Node(
        node_name='rviz2',
        package='rviz2', node_executable='rviz2', output='screen',
        arguments=[
            '-d',
            get_package_share_directory('raspimouse_ros2_examples')
            + '/config/default.rviz'],
    )

    def run_transform_publisher(context):
        if context.launch_configurations['lidar'] == 'lds':
            return [Node(
                package='tf2_ros',
                node_executable='static_transform_publisher', output='screen',
                arguments=['0', '0', '0.1', '0', '3.14', '3.14',
                           'base_footprint', 'laser'])]
        elif context.launch_configurations['lidar'] == 'urg':
            return [Node(
                package='tf2_ros',
                node_executable='static_transform_publisher', output='screen',
                arguments=['0', '0', '0.14', '0', '0', '0',
                           'base_footprint', 'laser'])]

    transform_node = OpaqueFunction(function=run_transform_publisher)

    ld = LaunchDescription()
    ld.add_action(declare_lidar)
    ld.add_action(slam_node)
    ld.add_action(rviz2_node)
    ld.add_action(transform_node)

    return ld
