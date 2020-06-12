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
from launch_ros.actions import Node

def generate_launch_description():
    slam_node = Node(
        node_name='async_slam',
        package='slam_toolbox', node_executable='async_slam_toolbox_node', output='screen',
        parameters=[
            get_package_share_directory("slam_toolbox") + '/config/mapper_params_online_async.yaml'
        ],
        )

    ld = LaunchDescription()
    ld.add_action(slam_node)

    return ld