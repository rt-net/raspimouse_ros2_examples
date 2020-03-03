#!/usr/bin/env python3
# coding: UTF-8

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


import rclpy
import math
from time import sleep
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class JoyWrapper(Node):

    def __init__(self):
        super().__init__('joystick_control')

        parameters = [
            ('button_shutdown_1', 8),
            ('button_shutdown_2', 9),
            ('button_motor_off', 8),
            ('button_motor_on', 9),
            ('button_cmd_enable', 4),
            ('axis_cmd_linear_x', 1),
            ('axis_cmd_angular_z', 3),
            ('analog_d_pad', False),
            ('d_pad_up', 13),
            ('d_pad_down', 14),
            ('d_pad_left', 15),
            ('d_pad_right', 16),
            ('d_pad_up_is_positive', True),
            ('d_pad_right_is_positive', False),
            ('button_buzzer_enable', 5),
            ('dpad_buzzer0', "up"),
            ('dpad_buzzer1', "right"),
            ('dpad_buzzer2', "down"),
            ('dpad_buzzer3', "left"),
            ('button_buzzer4', 2),
            ('button_buzzer5', 1),
            ('button_buzzer6', 0),
            ('button_buzzer7', 3),
            ('button_sensor_sound_en', 7),
            ('button_config_enable', 6),
        ]
        self.declare_parameters('', parameters)

        self._BUTTON_SHUTDOWN_1 = self.get_parameter('button_shutdown_1').value
        self._BUTTON_SHUTDOWN_2 = self.get_parameter('button_shutdown_2').value

        self._BUTTON_MOTOR_ON = self.get_parameter('button_motor_on').value
        self._BUTTON_MOTOR_OFF = self.get_parameter('button_motor_off').value

        self._BUTTON_CMD_ENABLE = self.get_parameter('button_cmd_enable').value
        self._AXIS_CMD_LINEAR_X = self.get_parameter('axis_cmd_linear_x').value
        self._AXIS_CMD_ANGULAR_Z = self.get_parameter('axis_cmd_angular_z').value

        self._ANALOG_D_PAD = self.get_parameter('analog_d_pad').value
        self._D_PAD_UP = self.get_parameter('d_pad_up').value
        self._D_PAD_DOWN = self.get_parameter('d_pad_down').value
        self._D_PAD_LEFT = self.get_parameter('d_pad_left').value
        self._D_PAD_RIGHT = self.get_parameter('d_pad_right').value
        self._D_UP_IS_POSITIVE = self.get_parameter('d_pad_up_is_positive').value
        self._D_RIGHT_IS_POSITIVE = self.get_parameter('d_pad_right_is_positive').value

        self._BUTTON_BUZZER_ENABLE = self.get_parameter('button_buzzer_enable').value
        self._DPAD_BUZZER0 = self.get_parameter('dpad_buzzer0').value
        self._DPAD_BUZZER1 = self.get_parameter('dpad_buzzer1').value
        self._DPAD_BUZZER2 = self.get_parameter('dpad_buzzer2').value
        self._DPAD_BUZZER3 = self.get_parameter('dpad_buzzer3').value
        self._BUTTON_BUZZER4 = self.get_parameter('button_buzzer4').value
        self._BUTTON_BUZZER5 = self.get_parameter('button_buzzer5').value
        self._BUTTON_BUZZER6 = self.get_parameter('button_buzzer6').value
        self._BUTTON_BUZZER7 = self.get_parameter('button_buzzer7').value

        self._BUTTON_SENSOR_SOUND_EN = self.get_parameter('button_sensor_sound_en').value
        self._BUTTON_CONFIG_ENABLE = self.get_parameter('button_config_enable').value

        # for _joy_velocity_config()
        self._MAX_VEL_LINEAR_X = 2.0 # m/s
        self._MAX_VEL_ANGULAR_Z = 2.0 * math.pi # rad/s
        self._DEFAULT_VEL_LINEAR_X = 0.5 # m/s
        self._DEFAULT_VEL_ANGULAR_Z = 1.0 * math.pi # rad/s

        self._joy_msg = None
        self._cmdvel_has_value = False
        self._buzzer_has_value = False
        self._vel_linear_x = self._DEFAULT_VEL_LINEAR_X
        self._vel_angular_z = self._DEFAULT_VEL_ANGULAR_Z

        self._node_logger = self.get_logger()

        self._pub_cmdvel = self.create_publisher(Twist, 'cmd_vel', 10)
        self._pub_buzzer = self.create_publisher(Int16, 'buzzer', 10)
        self._sub_joy = self.create_subscription(
            Joy,
            'joy',
            self._callback_joy,
            10)
        self._sub_joy  # prevent unused variable warning

        self._cli = self.create_client(SetBool, 'motor_power')
        while not self._cli.wait_for_service(timeout_sec=1.0):
            self._node_logger.info('service not available')
            self.destroy_node()
        self._motor_on()

        timer_period = 0.01   # seconds
        self.timer = self.create_timer(timer_period, self._callback_timer)
    
    def __del__(self):
        self._motor_off()
    
    def _motor_request(self, request_data=False):
        request = SetBool.Request()
        request.data = request_data
        future = self._cli.call_async(request)

    def _motor_on(self):
        self._motor_request(True)

    def _motor_off(self):
        self._motor_request(False)

    def _callback_joy(self, msg):
        self._joy_msg = msg

    def _callback_timer(self):
        if self._joy_msg is None:
            return

        self._joy_cmdvel(self._joy_msg)
        self._joy_buzzer_freq(self._joy_msg)

    def _joy_dpad(self, joy_msg, target_pad, positive_on):
        # d pad inputs of f710 controller are analog
        # d pad inputs of dualshock3 controller are digital
        if self._ANALOG_D_PAD:
            if positive_on:
                return joy_msg.axes[target_pad] > 0
            else:
                return joy_msg.axes[target_pad] < 0
        else:
            return joy_msg.buttons[target_pad]

    def _dpad_up(self, joy_msg):
        positive_on = self._D_UP_IS_POSITIVE
        return self._joy_dpad(joy_msg, self._D_PAD_UP, positive_on)

    def _dpad_down(self, joy_msg):
        positive_on = not self._D_UP_IS_POSITIVE
        return self._joy_dpad(joy_msg, self._D_PAD_DOWN, positive_on)

    def _dpad_left(self, joy_msg):
        positive_on = not self._D_RIGHT_IS_POSITIVE
        return self._joy_dpad(joy_msg, self._D_PAD_LEFT, positive_on)

    def _dpad_right(self, joy_msg):
        positive_on = self._D_RIGHT_IS_POSITIVE
        return self._joy_dpad(joy_msg, self._D_PAD_RIGHT, positive_on)

    def _dpad(self, joy_msg, target):
        if target == "up":
            return self._dpad_up(joy_msg)
        elif target == "down":
            return self._dpad_down(joy_msg)
        elif target == "left":
            return self._dpad_left(joy_msg)
        elif target == "right":
            return self._dpad_right(joy_msg)
        else:
            return False

    def _beep_buzzer(self, freq, beep_time=0):
        self._pub_buzzer.publish(freq)
        sleep(beep_time)
        self._pub_buzzer.publish(0)

    def _joy_buzzer_freq(self, joy_msg):
        freq = Int16()
        buttons = [
                self._dpad(joy_msg, self._DPAD_BUZZER0),
                self._dpad(joy_msg, self._DPAD_BUZZER1),
                self._dpad(joy_msg, self._DPAD_BUZZER2),
                self._dpad(joy_msg, self._DPAD_BUZZER3),
                joy_msg.buttons[self._BUTTON_BUZZER4],
                joy_msg.buttons[self._BUTTON_BUZZER5],
                joy_msg.buttons[self._BUTTON_BUZZER6],
                joy_msg.buttons[self._BUTTON_BUZZER7],
                ]
        # buzzer frequency Hz
        SCALES = [
                523, 587, 659, 699,
                784, 880, 987, 1046
                ]

        if joy_msg.buttons[self._BUTTON_BUZZER_ENABLE]:
            for i, button in enumerate(buttons):
                if button:
                    freq.data = SCALES[i]
                    break
            self._pub_buzzer.publish(freq)
            self._node_logger.info(str(freq))

            self._buzzer_has_value = True
        else:
            if self._buzzer_has_value:
                self._pub_buzzer.publish(freq)
                self._buzzer_has_value = False

    def _joy_cmdvel(self, joy_msg):
        cmdvel = Twist()
        if joy_msg.buttons[self._BUTTON_CMD_ENABLE]:
            cmdvel.linear.x = self._vel_linear_x * joy_msg.axes[self._AXIS_CMD_LINEAR_X]
            cmdvel.angular.z = self._vel_angular_z * joy_msg.axes[self._AXIS_CMD_ANGULAR_Z]
            self._node_logger.info(
                "linear_x:" + str(cmdvel.linear.x) +\
                ", angular_z:" + str(cmdvel.angular.z))
            self._pub_cmdvel.publish(cmdvel)

            self._cmdvel_has_value = True
        else:
            if self._cmdvel_has_value:
                self._pub_cmdvel.publish(cmdvel)
                self._cmdvel_has_value = False

def main(args=None):
    rclpy.init(args=args)

    joy_wrapper = JoyWrapper()

    rclpy.spin(joy_wrapper)

    joy_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
