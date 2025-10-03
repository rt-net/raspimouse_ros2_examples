[English](README.en.md) | [日本語](README.md)

# raspimouse_ros2_examples

[![industrial_ci](https://github.com/rt-net/raspimouse_ros2_examples/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse_ros2_examples/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

ROS 2 examples for Raspberry Pi Mouse.

For running in Gazebo (simulator), see the [rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim/blob/ros2/README.md) packages.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/raspberry_pi_mouse.JPG width=500 />

## Table of Contents

- [raspimouse_ros2_examples](#raspimouse_ros2_examples)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS distributions](#supported-ros-distributions)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [Binary Installation](#binary-installation)
    - [Source Build](#source-build)
  - [QuickStart](#quickstart)
  - [How To Use Examples](#how-to-use-examples)
  - [License](#license)
  - [Contributing](#contributing)

## Supported ROS distributions

### ROS 2

- [Humble Hawksbill](https://github.com/rt-net/raspimouse_ros2_examples/tree/humble)
- [Jazzy Jalisco](https://github.com/rt-net/raspimouse_ros2_examples/tree/jazzy)

## Requirements

- Raspberry Pi Mouse
  - https://rt-net.jp/products/raspberrypimousev3/
  - Linux OS
    - Ubuntu server 24.04
  - Device Driver
    - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
  - ROS 2
    - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
  - Raspberry Pi Mouse ROS 2 package
    - https://github.com/rt-net/raspimouse2
- Remote Computer (Optional)
  - ROS 2
    - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
  - Raspberry Pi Mouse ROS 2 package
    - https://github.com/rt-net/raspimouse2


## Installation

### Binary Installation

```sh
sudo apt install raspimouse-ros2-examples
```

### Source Build

```sh
# Create workspace directory
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone package
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_ros2_examples.git

# Install dependencies
rosdep install -r -y -i --from-paths .

# Build & Install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## QuickStart

Control the Raspberry Pi Mouse remotely with a joystick controller.

- Supported Controllers
  - [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
  - [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)

```sh
# on RaspberryPiMouse
$ ros2 run raspimouse raspimouse
# on remote computer
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py mouse:=false
```

## How to Use Examples

The detailed operation of the sample program is explained in[Examples](./Examples.md).

- Examples
  - Joystick Control
  - Object Tracking
  - Line Follower
  - Camera Line Follower
  - Direction Controller
  - SLAM & Navigation (relocated [rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2))

## License

(C) 2022 RT Corporation \<support@rt-net.jp\>

Each file is licensed as stated in their headers.  
If no license is specified, the file is licensed under the MIT License.  
The full license text is available in the [LICENSE](./LICENSE) file or at [https://opensource.org/license/MIT](https://opensource.org/license/MIT).

## Contributing

- This software is open source, but its development is not open.
- This software is essentially provided as open source software on an “AS IS” (in its current state) basis.
- No free support is available for this software.
- Requests for bug fixes and corrections of typographical errors are always accepted; however, requests for additional features will be subject to our internal guidelines. For further details, please refer to the [Contribution Guidelines](https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md).
