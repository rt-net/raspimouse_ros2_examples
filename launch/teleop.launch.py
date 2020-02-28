# Copyright 2020 RT-Corporation
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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # parameter
    joydev = LaunchConfiguration('joydev')

    declare_joydev = DeclareLaunchArgument(
        'joydev', default_value='/dev/input/js0',
        description='Device file for JoyStick Controller'
    )

    joy_node = Node(
        package='joy',
        node_executable='joy_node',
        parameters=[{'dev': joydev}]
    )

    joystick_control_node = Node(
        package='raspimouse_ros2_examples',
        node_executable='joystick_control',
    )

    ld = LaunchDescription()

    ld.add_action(declare_joydev)
    ld.add_action(joy_node)
    ld.add_action(joystick_control_node)

    return ld
