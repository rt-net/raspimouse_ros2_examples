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

import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition


def generate_launch_description(argv=sys.argv[1:]):
    ld = LaunchDescription()

    teleop_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('raspimouse_ros2_examples'), 'launch'),
            '/teleop.launch.py'
            ]),
        launch_arguments={'joydev': '/dev/input/js0'}.items(),
    )

    mouse_node = LifecycleNode(
        node_name='raspimouse',
        package='raspimouse', node_executable='raspimouse', output='screen',
        parameters=[os.path.join(get_package_share_directory(
            'raspimouse_ros2_examples'), 'config', 'mouse.yml')]
    )

    register_event_handler_for_inactive_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mouse_node,
            goal_state='inactive',
            entities=[
                LogInfo(msg="mouse_node reached the inactive state, activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(mouse_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                )),
            ],
        )
    )

    register_event_handler_for_active_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=mouse_node,
            goal_state='active',
            entities=[
                LogInfo(msg="mouse_node reached the active state, launching telop nodes."),
                teleop_launch_description,
            ],
        )
    )

    emit_event_to_request_that_mouse_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(mouse_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    ld.add_action(register_event_handler_for_inactive_state)
    ld.add_action(register_event_handler_for_active_state)
    ld.add_action(mouse_node)
    ld.add_action(emit_event_to_request_that_mouse_does_configure_transition)

    print(launch.LaunchIntrospector().format_launch_description(ld))

    return ld


if __name__ == '__main__':
    generate_launch_description()
