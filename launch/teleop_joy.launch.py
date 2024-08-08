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
from launch import LaunchIntrospector
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.events import Shutdown


def generate_launch_description():

    joydev = LaunchConfiguration('joydev')
    declare_joydev = DeclareLaunchArgument(
        'joydev', default_value='/dev/input/js0',
        description='Device file for JoyStick Controller'
    )

    declare_joyconfig = DeclareLaunchArgument(
        'joyconfig', default_value='f710',
        description='Keyconfig of joystick controllers: supported: f710, dualshock3'
    )

    declare_mouse = DeclareLaunchArgument(
        'mouse', default_value="true",
        description='Launch raspimouse node'
    )

    def func_get_joyconfig_file_name(context):
        param_file = os.path.join(
            get_package_share_directory('raspimouse_ros2_examples'),
            'config',
            'joy_' + context.launch_configurations['joyconfig'] + '.yml')

        if os.path.exists(param_file):
            return [SetLaunchConfiguration('joyconfig_filename', param_file)]
        else:
            return [LogInfo(msg=param_file + " is not exist.")]
    get_joyconfig_file_name = OpaqueFunction(function=func_get_joyconfig_file_name)

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[{'dev': joydev}]
    )

    joystick_control_node = Node(
        package='raspimouse_ros2_examples',
        executable='joystick_control.py',
        parameters=[LaunchConfiguration('joyconfig_filename')],
        on_exit=Shutdown(),
    )

    def func_launch_mouse_node(context):
        if context.launch_configurations['mouse'] == "true":
            return [LifecycleNode(
                name='raspimouse', namespace="",
                package='raspimouse', executable='raspimouse', output='screen',
                parameters=[os.path.join(get_package_share_directory(
                    'raspimouse_ros2_examples'), 'config', 'mouse.yml')]
            )]
    mouse_node = OpaqueFunction(function=func_launch_mouse_node)

    ld = LaunchDescription()
    ld.add_action(declare_joydev)
    ld.add_action(declare_joyconfig)
    ld.add_action(declare_mouse)

    ld.add_action(get_joyconfig_file_name)

    ld.add_action(joy_node)
    ld.add_action(joystick_control_node)
    ld.add_action(mouse_node)

    print(LaunchIntrospector().format_launch_description(ld))

    return ld
