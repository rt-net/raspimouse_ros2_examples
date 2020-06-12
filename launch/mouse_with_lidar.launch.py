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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    declare_lidar = DeclareLaunchArgument(
        'lidar', default_value="lds",
        description='LiDAR: urg, lds,'
    )

    mouse_node = LifecycleNode(
        node_name='raspimouse',
        package='raspimouse', node_executable='raspimouse', output='screen',
        parameters=[os.path.join(get_package_share_directory(
            'raspimouse_ros2_examples'), 'config', 'mouse.yml')]
            )

    def func_launch_lidar_node(context):
        if context.launch_configurations['lidar'] == "lds":
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('hls_lfcd_lds_driver'), 'launch'),
                    '/hlds_laser.launch.py'
                    ]),)]
    launch_lidar_node = OpaqueFunction(function=func_launch_lidar_node)

    ld = LaunchDescription()
    ld.add_action(declare_lidar)

    ld.add_action(mouse_node)
    ld.add_action(launch_lidar_node)

    return ld
