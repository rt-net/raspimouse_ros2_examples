[English](README.en.md) | [日本語](README.md)

# raspimouse_ros2_examples

[![industrial_ci](https://github.com/rt-net/raspimouse_ros2_examples/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse_ros2_examples/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

Raspberry Pi MouseのROS 2サンプルコード集です。

ROS1のサンプルコード集は[こちら](https://github.com/rt-net/raspimouse_ros_examples)。

<img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/raspberry_pi_mouse.JPG" width="500" />

## Requirements

- Raspberry Pi Mouse
  - https://rt-net.jp/products/raspberrypimousev3/
  - Linux OS
    - Ubuntu server 18.04
    - https://wiki.ubuntu.com/ARM/RaspberryPi
  - Device Driver
    - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
  - ROS
    - [Dashing Diademata](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)
  - Raspberry Pi Mouse ROS 2 package
    - https://github.com/rt-net/raspimouse2
- Remote Computer (Optional)
  - ROS
    - [Dashing Diademata](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)
  - Raspberry Pi Mouse ROS 2 package
    - https://github.com/rt-net/raspimouse2

## Installation

```sh
$ cd ~/ros2_ws/src
# Clone package
$ git clone https://github.com/rt-net/raspimouse_ros2_examples
$ git clone https://github.com/rt-net/raspimouse2

# Install dependencies
$ rosdep install -r -y --from-paths . --ignore-src

# Build & Install
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash
```

## License

このリポジトリはApache 2.0ライセンスの元、公開されています。 
ライセンスについては[LICENSE](./LICENSE)を参照ください。

## How To Use Examples

- [joystick_control](#joystick_control)

---

### joystick_control

ジョイスティックコントローラでRaspberryPiMouseを動かすコード例です。

#### Requirements 

- Joystick Controller
  - [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
  - [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)

#### How to use

次のコマンドでノードを起動します。

```sh
$ ros2 launch raspimouse_ros2_examples teleop_with_mouse.launch.py joydev:="/dev/input/js0"
```

デフォルトのキー割り当てはこちらです。

![joystick_control_keyconfig](https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/joystick_control_keyconfig.png)

#### Configure

[./launch/teleop.launch.py](./launch/teleop.launch.py)を編集してコンフィグファイルを切り替えます。

```python
def generate_launch_description():
    config_file_name = 'joy_dualshock3.yml'
    # config_file_name = 'joy_f710.yml'
```

[./config/joy_f710.yml](./config/joy_f710.yml)、[./config/joy_dualshock3.yml](./config/joy_dualshock3.yml)
のキー番号を編集することで、キー割り当てを変更できます。

```yaml
button_shutdown_1       : 8
button_shutdown_2       : 9

button_motor_off        : 8
button_motor_on         : 9

button_cmd_enable       : 4
```

#### Videos

[![joystick_control](http://img.youtube.com/vi/GswxdB8Ia0Y/sddefault.jpg)](https://youtu.be/GswxdB8Ia0Y)

[back to example list](#how-to-use-examples)

--- 
