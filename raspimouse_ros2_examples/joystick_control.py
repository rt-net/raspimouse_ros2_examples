import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy


class JoyWrapper(Node):

    def __init__(self):
        super().__init__('joystick_control')

        self._joy_msg = Joy()

        self._sub_joy = self.create_subscription(
            Joy,
            'joy',
            self._callback_joy,
            10)
        self._sub_joy  # prevent unused variable warning

        timer_period = 0.1   # seconds
        self.timer = self.create_timer(timer_period, self._callback_timer)

    def _callback_joy(self, msg):
        self._joy_msg = msg

    def _callback_timer(self):
        if self._joy_msg is None:
            return


def main(args=None):
    rclpy.init(args=args)

    joy_wrapper = JoyWrapper()

    rclpy.spin(joy_wrapper)

    joy_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
