#!/usr/bin/env python3
# coding: UTF-8

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


import math
from time import sleep

from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from raspimouse_msgs.msg import Leds
from raspimouse_msgs.msg import LightSensors
from raspimouse_msgs.msg import Switches

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
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
            ('dpad_buzzer0', 'up'),
            ('dpad_buzzer1', 'right'),
            ('dpad_buzzer2', 'down'),
            ('dpad_buzzer3', 'left'),
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
        self._MAX_VEL_LINEAR_X = 2.0  # m/s
        self._MAX_VEL_ANGULAR_Z = 2.0 * math.pi  # rad/s
        self._DEFAULT_VEL_LINEAR_X = 0.5  # m/s
        self._DEFAULT_VEL_ANGULAR_Z = 1.0 * math.pi  # rad/s

        self._lightsensors = LightSensors()
        self._mouse_switches = Switches()
        self._cmdvel_has_value = False
        self._buzzer_has_value = False
        self._switch_has_been_pressed = False
        self._sensor_sound_has_value = False
        self._vel_linear_x = self._DEFAULT_VEL_LINEAR_X
        self._vel_angular_z = self._DEFAULT_VEL_ANGULAR_Z

        self._node_logger = self.get_logger()

        self._pub_cmdvel = self.create_publisher(Twist, 'cmd_vel', 1)
        self._pub_buzzer = self.create_publisher(Int16, 'buzzer', 1)
        self._pub_leds = self.create_publisher(Leds, 'leds', 1)

        self._sub_cb_group = MutuallyExclusiveCallbackGroup()
        self._client_cb_group = MutuallyExclusiveCallbackGroup()

        self._client_get_state = self.create_client(
            GetState, 'raspimouse/get_state', callback_group=self._client_cb_group)
        while not self._client_get_state.wait_for_service(timeout_sec=1.0):
            self._node_logger.warn(self._client_get_state.srv_name + ' service not available')

        self._client_change_state = self.create_client(
            ChangeState, 'raspimouse/change_state', callback_group=self._client_cb_group)
        while not self._client_change_state.wait_for_service(timeout_sec=1.0):
            self._node_logger.warn(self._client_change_state.srv_name + ' service not available')
        self._activate_raspimouse()

        self._client_motor_power = self.create_client(
            SetBool, 'motor_power', callback_group=self._client_cb_group)
        while not self._client_motor_power.wait_for_service(timeout_sec=1.0):
            self._node_logger.warn(self._client_motor_power.srv_name + ' service not available')
        self._motor_on()

        self._sub_joy = self.create_subscription(
            Joy, 'joy', self._callback_joy, 1,
            callback_group=self._sub_cb_group)
        self._sub_lightsensor = self.create_subscription(
            LightSensors, 'light_sensors', self._callback_lightsensors, 1,
            callback_group=self._sub_cb_group)
        self._sub_switches = self.create_subscription(
            Switches, 'switches', self._callback_switches, 1,
            callback_group=self._sub_cb_group)

    def _activate_raspimouse(self):
        self._set_mouse_lifecycle_state(Transition.TRANSITION_CONFIGURE)
        self._set_mouse_lifecycle_state(Transition.TRANSITION_ACTIVATE)
        self._node_logger.info('Mouse state is '
                               + self._get_mouse_lifecycle_state())

    def _set_mouse_lifecycle_state(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self._client_change_state.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def _get_mouse_lifecycle_state(self):
        future = self._client_get_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result().current_state.label

    def _motor_request(self, request_data=False):
        request = SetBool.Request()
        request.data = request_data
        self._client_motor_power.call_async(request)

    def _motor_on(self):
        self._motor_request(True)

    def _motor_off(self):
        self._motor_request(False)

    def _callback_joy(self, msg):
        self._joy_motor_onoff(msg)
        self._joy_cmdvel(msg)
        self._joy_buzzer_freq(msg)
        self._joy_lightsensor_sound(msg)
        self._joy_velocity_config(msg)
        self._joy_leds(msg)
        self._joy_shutdown(msg)

    def _callback_lightsensors(self, msg):
        self._lightsensors = msg

    def _callback_switches(self, msg):
        self._mouse_switches = msg

    def _joy_shutdown(self, joy_msg):
        if joy_msg.buttons[self._BUTTON_SHUTDOWN_1] and\
                joy_msg.buttons[self._BUTTON_SHUTDOWN_2]:
            self._pub_leds.publish(Leds())
            self._motor_off()
            self._set_mouse_lifecycle_state(Transition.TRANSITION_DEACTIVATE)
            self.destroy_node()
            raise SystemExit

    def _joy_motor_onoff(self, joy_msg):
        if joy_msg.buttons[self._BUTTON_MOTOR_ON]:
            self._motor_on()

        if joy_msg.buttons[self._BUTTON_MOTOR_OFF]:
            self._motor_off()

    def _joy_cmdvel(self, joy_msg):
        cmdvel = Twist()
        if joy_msg.buttons[self._BUTTON_CMD_ENABLE]:
            cmdvel.linear.x = self._vel_linear_x * joy_msg.axes[self._AXIS_CMD_LINEAR_X]
            cmdvel.angular.z = self._vel_angular_z * joy_msg.axes[self._AXIS_CMD_ANGULAR_Z]
            self._node_logger.info(
                'linear_x:' + str(cmdvel.linear.x) +
                ', angular_z:' + str(cmdvel.angular.z))
            self._pub_cmdvel.publish(cmdvel)

            self._cmdvel_has_value = True
        else:
            if self._cmdvel_has_value:
                self._pub_cmdvel.publish(cmdvel)
                self._cmdvel_has_value = False

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
        if target == 'up':
            return self._dpad_up(joy_msg)
        elif target == 'down':
            return self._dpad_down(joy_msg)
        elif target == 'left':
            return self._dpad_left(joy_msg)
        elif target == 'right':
            return self._dpad_right(joy_msg)
        else:
            return False

    def _beep_buzzer(self, freq_data, beep_time=0):
        freq = Int16()
        freq.data = freq_data
        self._pub_buzzer.publish(freq)
        sleep(beep_time)
        freq.data = 0
        self._pub_buzzer.publish(freq)

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

    def _joy_lightsensor_sound(self, joy_msg):
        freq = Int16()
        if joy_msg.buttons[self._BUTTON_SENSOR_SOUND_EN]:
            self._node_logger.info(str(self._lightsensors))
            freq.data += self._positive(self._lightsensors.left)
            freq.data += self._positive(self._lightsensors.forward_l)
            freq.data += self._positive(self._lightsensors.forward_r)
            freq.data += self._positive(self._lightsensors.right)

            self._pub_buzzer.publish(freq)
            self._sensor_sound_has_value = True
        else:
            if self._sensor_sound_has_value:
                self._pub_buzzer.publish(freq)
                self._sensor_sound_has_value = False

    def _positive(self, value):
        if value < 0:
            return 0
        else:
            return value

    def _joy_velocity_config(self, joy_msg):
        ADD_VEL_LINEAR_X = 0.1  # m/s
        ADD_VEL_ANGULAR_Z = 0.1 * math.pi  # m/s
        BUZZER_FREQ_ADD = 880  # Hz
        BUZZER_FREQ_SUB = 440  # Hz
        BUZZER_FREQ_RESET = 660  # Hz
        BUZZER_BEEP_TIME = 0.2  # sec

        if joy_msg.buttons[self._BUTTON_CONFIG_ENABLE]:
            any_switch_pressed = False
            if any([self._mouse_switches.switch0,
                    self._mouse_switches.switch1,
                    self._mouse_switches.switch2]):
                any_switch_pressed = True

            if not self._switch_has_been_pressed:
                if self._mouse_switches.switch0:
                    self._vel_linear_x = self._config_velocity(
                            self._vel_linear_x, ADD_VEL_LINEAR_X,
                            0, self._MAX_VEL_LINEAR_X)
                    self._vel_angular_z = self._config_velocity(
                            self._vel_angular_z, ADD_VEL_ANGULAR_Z,
                            0, self._MAX_VEL_ANGULAR_Z)
                    self._beep_buzzer(BUZZER_FREQ_ADD, BUZZER_BEEP_TIME)

                elif self._mouse_switches.switch2:
                    self._vel_linear_x = self._config_velocity(
                            self._vel_linear_x, -ADD_VEL_LINEAR_X,
                            0, self._MAX_VEL_LINEAR_X)
                    self._vel_angular_z = self._config_velocity(
                            self._vel_angular_z, -ADD_VEL_ANGULAR_Z,
                            0, self._MAX_VEL_ANGULAR_Z)
                    self._beep_buzzer(BUZZER_FREQ_SUB, BUZZER_BEEP_TIME)

                elif self._mouse_switches.switch1:
                    self._vel_linear_x = self._DEFAULT_VEL_LINEAR_X
                    self._vel_angular_z = self._DEFAULT_VEL_ANGULAR_Z
                    self._beep_buzzer(BUZZER_FREQ_RESET, BUZZER_BEEP_TIME)

            self._switch_has_been_pressed = any_switch_pressed
            self._node_logger.info(
                    'linear_x:' + str(self._vel_linear_x) +
                    ', angular_z:' + str(self._vel_angular_z)
                    )

    def _config_velocity(self, current, add, lowerlimit, upperlimit):
        output = current + add

        if output < lowerlimit:
            output = lowerlimit
        if output > upperlimit:
            output = upperlimit

        return output

    def _joy_leds(self, joy_msg):
        leds = Leds()

        if joy_msg.buttons[self._BUTTON_CMD_ENABLE]:
            leds.led3 = True

        if joy_msg.buttons[self._BUTTON_BUZZER_ENABLE]:
            leds.led2 = True

        if joy_msg.buttons[self._BUTTON_SENSOR_SOUND_EN]:
            leds.led1 = True

        if joy_msg.buttons[self._BUTTON_CONFIG_ENABLE]:
            leds.led0 = True

        self._pub_leds.publish(leds)


def main(args=None):
    rclpy.init(args=args)

    joy_wrapper = JoyWrapper()

    try:
        rclpy.spin(joy_wrapper)
    except SystemExit:
        rclpy.logging.get_logger("joystick_control").info('_joy_shutdown() has been executed')

    joy_wrapper.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
