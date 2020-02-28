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
