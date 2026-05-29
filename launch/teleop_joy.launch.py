# Copyright 2020-2024 RT Corporation
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
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    joydev = LaunchConfiguration('joydev')
    declare_joydev = DeclareLaunchArgument(
        'joydev',
        default_value='/dev/input/js0',
        description='Device file for JoyStick Controller',
    )

    declare_joyconfig = DeclareLaunchArgument(
        'joyconfig',
        default_value='f710',
        description='Keyconfig of joystick controllers: supported: f710, dualshock3',
    )

    declare_mouse = DeclareLaunchArgument(
        'mouse', default_value='true', description='Launch raspimouse node'
    )

    joy_param = [
        os.path.join(get_package_share_directory('raspimouse_ros2_examples')),
        '/config',
        '/joy_',
        LaunchConfiguration('joyconfig'),
        '.yml',
    ]

    mouse_param = [
        os.path.join(get_package_share_directory('raspimouse_ros2_examples')),
        '/config',
        '/mouse.yaml',
    ]

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[{'dev': joydev}],
    )

    joystick_control_node = Node(
        package='raspimouse_ros2_examples',
        executable='joystick_control.py',
        parameters=[joy_param],
        on_exit=Shutdown(),
    )

    mouse_node = LifecycleNode(
        name='raspimouse',
        namespace='',
        package='raspimouse',
        executable='raspimouse',
        output='screen',
        parameters=[mouse_param],
        condition=IfCondition(LaunchConfiguration('mouse')),
    )

    return LaunchDescription(
        [
            declare_joydev,
            declare_joyconfig,
            declare_mouse,
            joy_node,
            joystick_control_node,
            mouse_node,
        ]
    )
