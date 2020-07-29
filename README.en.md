[English](README.en.md) | [日本語](README.md)

# raspimouse_ros2_examples

[![industrial_ci](https://github.com/rt-net/raspimouse_ros2_examples/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse_ros2_examples/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

ROS 2 examples for Raspberry Pi Mouse.

ROS1 examples is [here](https://github.com/rt-net/raspimouse_ros_examples).

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

This repository is licensed under the Apache 2.0, see [LICENSE](./LICENSE) for details.

## How To Use Examples

- [joystick_control](#joystick_control)
- [object_tracking](#object_tracking)
- [line_follower](#line_follower)
- [SLAM](#slam)

---

### joystick_control

This is an example to use joystick controller to control a Raspberry Pi Mouse.

#### Requirements 

- Joystick Controller
  - [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
  - [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)

#### How to use

Launch nodes with the following command:

```sh
# Use F710
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py joydev:="/dev/input/js0" joyconfig:=f710 mouse:=true

# Use DUALSHOCK 3
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py joydev:="/dev/input/js0" joyconfig:=dualshock3 mouse:=true

# Control from remote computer
## on RaspberryPiMouse
$ ros2 run raspimouse raspimouse
## on remote computer
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py mouse:=false
```

This picture shows the default key configuration.

To use Logicool Wireless Gamepad F710, set the input mode to  __D__ (DirectInput Mode).

![joystick_control_keyconfig](https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/joystick_control_keyconfig.png)

#### Configure

Key assignments can be edited with key numbers in [./config/joy_f710.yml](./config/joy_f710.yml) or
[./config/joy_dualshock3.yml](./config/joy_dualshock3.yml).

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

### object_tracking

<img src="https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/object_tracking.JPG" width=500 />

This is an example to use RGB camera images and OpenCV library for object tracking.

#### Requirements 

- Web camera
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- Camera mount
  - [Raspberry Pi Mouse Option kit No.4 \[Webcam mount\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584&language=en)
- Orange ball（Optional）
  - [Soft Ball (Orange)](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701&language=en)
- Software
  - OpenCV
  - v4l-utils

#### Installation

Install a camera mount and a web camera to Raspberry Pi Mouse, then connect the camera to the Raspberry Pi．

#### How to use

Turn off automatic adjustment parameters of a camera (auto focus, auto while balance, etc.) with the following command:

```sh
$ cd ~/ros2_ws/src/raspimouse_ros2_examples/config
$ ./configure_camera.bash
```

Then, launch nodes with the following command:

```sh
$ ros2 launch raspimouse_ros2_examples object_tracking.launch.py
```

This sample publishes two topics: `raw_image` for the camera image and `result_image` for the object detection image.
These images can be viewed with [RViz](https://index.ros.org/r/rviz/)
or [rqt_image_view](https://index.ros.org/doc/ros2/Tutorials/RQt-Overview-Usage/).

**Viewing an image may cause the node to behave unstable and not publish cmd_vel or image topics.**

<img src="https://github.com/rt-net/raspimouse_ros2_examples/blob/images/object_tracking_images.png" width=500 />

#### Configure

Edit [`./src/object_tracking_component.cpp`](./src/object_tracking_component.cpp)
to change a color of tracking target.

If the object detection accuracy is poor, adjust the camera exposure and parameters in the function

```cpp
void Tracker::tracking(const cv::Mat & input_frame, cv::Mat & result_frame)
{
  cv::inRange(hsv, cv::Scalar(9, 100, 100), cv::Scalar(29, 255, 255), extracted_bin);  // Orange
  // cv::inRange(hsv, cv::Scalar(60, 100, 100), cv::Scalar(80, 255, 255), extracted_bin);  // Green
  // cv::inRange(hsv, cv::Scalar(100, 100, 100), cv::Scalar(120, 255, 255), extracted_bin);  // Blue
```

#### Videos

[![object_tracking](http://img.youtube.com/vi/8lgmSTScP98/sddefault.jpg)](https://youtu.be/8lgmSTScP98)

[back to example list](#how-to-use-examples)

--- 

### line_follower

![mouse_with_line_trace_sensor](https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_line_trace_sensor.JPG)

This is an example for line following.

#### Requirements

- Line following sensor
  - [Raspberry Pi Mouse Option kit No.3 \[Line follower\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3591&language=en)
- Field and lines for following (Optional)

#### Installation

Install a line following sensor unit to Raspberry Pi Mouse.

#### How to use

Launch nodes with the following command:

```sh
$ ros2 launch raspimouse_ros2_examples line_follower.launch.py
```

Next, place Raspberry Pi Mouse on a field and press SW2 to sample sensor values on the field.

<img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/field_calibration.JPG" width=500 />

Then, place Raspberry Pi Mouse to detect a line and press SW1 to sample sensor values on the line.

<img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/line_calibration.JPG" width=500 />

Last, place Raspberry Pi Mouse on the line and press SW0 to start line following.

<img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/start_trace.JPG" width=500 />

Press SW0 again to stop the following.

#### Configure

Edit [`./src/line_follower_component.cpp`](./src/line_follower_component.cpp) to change a velocity command.

```cpp
void Follower::publish_cmdvel_for_line_following(void)
{
  const double VEL_LINEAR_X = 0.08;  // m/s
  const double VEL_ANGULAR_Z = 0.8;  // rad/s
  const double LOW_VEL_ANGULAR_Z = 0.5;  // rad/s
```

#### Videos

[![line_follower](http://img.youtube.com/vi/oPm0sW2V_tY/sddefault.jpg)](https://youtu.be/oPm0sW2V_tY)

[back to example list](#how-to-use-examples)

--- 

### SLAM

<img src="https://github.com/rt-net/raspimouse_ros2_examples/blob/images/slam_toolbox.png" width=500 />

This is an example to use LiDAR and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) for SLAM (Simultaneous Localization And Mapping).

#### Requirements 

- LiDAR
  <!-- - [URG-04LX-UG01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1296&products_id=2816&language=en)
  - [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1) -->
  - [LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676&language=en)
- [LiDAR Mount](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867&language=en)
- Joystick Controller (Optional)
  
#### Installation

Install a LiDAR to the Raspberry Pi Mouse.

<!-- - URG-04LX-UG01
  - <img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_urg.JPG" width=500 />
- RPLIDAR A1
  - <img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_rplidar.png" width=500 /> -->
- LDS-01
  - <img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_lds01.JPG" width=500 />
  
#### How to use

Launch nodes on Raspberry Pi Mouse with the following command:

```sh
# LDS
$ ros2 launch raspimouse_ros2_examples mouse_with_lidar.launch.py lidar:=lds
```

Next, launch `teleop_joy.launch.py` to control Raspberry Pi Mouse with the following command:

```sh
# Use DUALSHOCK 3
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py joydev:="/dev/input/js0" joyconfig:=dualshock3 mouse:=false
```

Then, launch the slam_toolbox package (on a remote computer recommend) with the following command:

```sh
$ ros2 launch raspimouse_ros2_examples slam.launch.py
```

After moving Raspberry Pi Mouse and making a map, run a node to save the map with the following command:

```sh
$ mkdir ~/maps
$ ros2 run nav2_map_server map_saver -f ~/maps/mymap
```

#### Configure

Edit [./config/mapper_params_offline.yaml](./config/mapper_params_offline.yaml) to configure parameters of [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) package.

<!-- #### Videos

[![slam_urg](http://img.youtube.com/vi/gWozU47UqVE/sddefault.jpg)](https://youtu.be/gWozU47UqVE)

[![slam_urg](http://img.youtube.com/vi/hV68UqAntfo/sddefault.jpg)](https://youtu.be/hV68UqAntfo) -->

[back to example list](#how-to-use-examples)
