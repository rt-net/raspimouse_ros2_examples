# Copyright 2023 RT Corporation
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
    container = ComposableNodeContainer(
            name='camera_line_follower_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='raspimouse_ros2_examples',
                    plugin='camera_line_follower::Camera_Follower',
                    name='camera_follower',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='raspimouse',
                    plugin='raspimouse::Raspimouse',
                    name='raspimouse',
                    parameters=[{'use_light_sensors': False}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    condition=IfCondition(LaunchConfiguration('mouse'))),
                ComposableNode(
                    package='usb_cam',
                    plugin='usb_cam::UsbCamNode',
                    name='usb_cam',
                    remappings=[('image_raw', 'camera/color/image_raw')],
                    parameters=[
                        {'video_device': LaunchConfiguration('video_device')},
                        {'frame_id': 'camera_color_optical_frame'},
                        {'pixel_format': 'yuyv2rgb'},
                        {'image_width': 320},
                        {'image_height': 240},
                        {'auto_white_balance': False},
                        {'autoexposure': False}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    condition=IfCondition(LaunchConfiguration('use_camera_node'))),
            ],
            output='screen',
    )

    manager = Node(
        name='manager',
        package='raspimouse_ros2_examples',
        executable='lifecycle_node_manager',
        output='screen',
        parameters=[{'components': ['raspimouse', 'camera_follower']}]
    )

    return launch.LaunchDescription([
        declare_mouse,
        declare_use_camera_node,
        declare_video_device,
        container,
        manager
    ])
