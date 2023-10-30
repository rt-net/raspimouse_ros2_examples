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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    declare_mouse = DeclareLaunchArgument(
        'mouse',
        default_value="true",
        description='Launch raspimouse node'
    )
    declare_use_camera_node = DeclareLaunchArgument(
        'use_camera_node',
        default_value='true',
        description='Use camera node.'
    )
    declare_video_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Set video device.'
    )

    """Generate launch description with multiple components."""
    mouse_node = LifecycleNode(
        name='raspimouse',
        namespace="",
        package='raspimouse',
        executable='raspimouse',
        output='screen',
        parameters=[{'use_light_sensors': False, }],
        condition=IfCondition(LaunchConfiguration('mouse'))
    )

    container = ComposableNodeContainer(
            name='object_tracking_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='raspimouse_ros2_examples',
                    plugin='object_tracking::Tracker',
                    name='tracker'),
            ],
            output='screen',
    )

    manager = Node(
        name='manager',
        package='raspimouse_ros2_examples',
        executable='lifecycle_node_manager',
        output='screen',
        parameters=[{'components': ['raspimouse', 'tracker']}]
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        remappings=[('image_raw', 'camera/color/image_raw')],
        parameters=[
            {'video_device': LaunchConfiguration('video_device')},
            {'frame_id': 'camera_color_optical_frame'},
            {'pixel_format': 'yuyv2rgb'}
        ],
        condition=IfCondition(LaunchConfiguration('use_camera_node'))
    )

    return launch.LaunchDescription([
        declare_mouse,
        declare_use_camera_node,
        declare_video_device,
        mouse_node,
        container,
        manager,
        usb_cam_node
    ])
