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

from launch import LaunchIntrospector
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_name = 'joy_dualshock3.yml'
    # config_file_name = 'joy_f710.yml'
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
        node_executable='joystick_control.py',
        parameters=[os.path.join(get_package_share_directory(
            'raspimouse_ros2_examples'), 'config', config_file_name)],
    )

    ld = LaunchDescription()

    ld.add_action(declare_joydev)
    ld.add_action(joy_node)
    ld.add_action(joystick_control_node)

    print(LaunchIntrospector().format_launch_description(ld))

    return ld
