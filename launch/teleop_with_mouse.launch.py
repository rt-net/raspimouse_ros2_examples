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

import sys
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchIntrospector
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode


def generate_launch_description(argv=sys.argv[1:]):

    joydev = LaunchConfiguration('joydev')

    declare_joydev = DeclareLaunchArgument(
        'joydev', default_value='/dev/input/js0',
        description='Device file for JoyStick Controller'
    )

    teleop_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('raspimouse_ros2_examples'), 'launch'),
            '/teleop.launch.py'
            ]),
        # launch_arguments={'joydev': '/dev/input/js0'}.items(),
        launch_arguments={'joydev': joydev}.items(),
    )

    mouse_node = LifecycleNode(
        node_name='raspimouse',
        package='raspimouse', node_executable='raspimouse', output='screen',
        parameters=[os.path.join(get_package_share_directory(
            'raspimouse_ros2_examples'), 'config', 'mouse.yml')]
    )

    ld = LaunchDescription()
    ld.add_action(declare_joydev)
    ld.add_action(teleop_launch_description)
    ld.add_action(mouse_node)

    print(LaunchIntrospector().format_launch_description(ld))

    return ld


if __name__ == '__main__':
    generate_launch_description()
