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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            node_name='object_tracking_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='raspimouse_ros2_examples',
                    node_plugin='object_tracking::Tracker',
                    node_name='tracker'),
                ComposableNode(
                    package='raspimouse',
                    node_plugin='raspimouse::Raspimouse',
                    node_name='raspimouse',
                    parameters=[{'use_light_sensors': False}]),
            ],
            output='screen',
    )

    object_tracking_core = Node(
        node_name='manager',
        package='raspimouse_ros2_examples',
        node_executable='lifecycle_node_manager',
        output='screen',
        parameters=[{'components': ['raspimouse', 'tracker']}]

    )

    return launch.LaunchDescription([container, object_tracking_core])
