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
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple components."""
    mouse_node = LifecycleNode(
        name='raspimouse',
        namespace='',
        package='raspimouse',
        executable='raspimouse',
        output='screen',
        parameters=[
            {
                'use_light_sensors': False,
            }
        ],
    )

    imu_driver = LifecycleNode(
        name='rt_usb_9axisimu_driver',
        namespace='',
        package='rt_usb_9axisimu_driver',
        executable='rt_usb_9axisimu_driver',
        output='screen',
        parameters=[
            {
                'port': '/dev/ttyACM0',
            }
        ],
    )

    direction_controller = Node(
        package='raspimouse_ros2_examples',
        executable='direction_controller',
        output='screen',
    )

    manager = Node(
        name='manager',
        package='raspimouse_ros2_examples',
        executable='lifecycle_node_manager',
        output='screen',
        parameters=[{'components': ['raspimouse', 'rt_usb_9axisimu_driver']}],
    )

    return launch.LaunchDescription(
        [mouse_node, imu_driver, direction_controller, manager]
    )
