[English](README.en.md) | [日本語](README.md)

# raspimouse_ros2_examples

[![industrial_ci](https://github.com/rt-net/raspimouse_ros2_examples/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse_ros2_examples/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

ROS 2 examples for Raspberry Pi Mouse.

ROS1 examples is [here](https://github.com/rt-net/raspimouse_ros_examples/blob/master/README.en.md).

To run on Gazebo, click [here](https://github.com/rt-net/raspimouse_sim/blob/ros2/README.en.md).

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/raspberry_pi_mouse.JPG width=500 />

## Supported ROS 2 distributions

- [Humble](https://github.com/rt-net/raspimouse_ros2_examples/tree/humble)
- [Jazzy](https://github.com/rt-net/raspimouse_ros2_examples/tree/jazzy) (This branch)

## Requirements

- Raspberry Pi Mouse
  - https://rt-net.jp/products/raspberrypimousev3/
  - Linux OS
    - Ubuntu server 24.04
  - Device Driver
    - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
  - ROS
    - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
  - Raspberry Pi Mouse ROS 2 package
    - https://github.com/rt-net/raspimouse2
- Remote Computer (Optional)
  - ROS
    - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
  - Raspberry Pi Mouse ROS 2 package
    - https://github.com/rt-net/raspimouse2

## Installation

```sh
$ cd ~/ros2_ws/src
# Clone package
$ git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_ros2_examples.git

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
- [camera_line_follower](#camera_line_follower)
- [SLAM](#slam)
- [direction_controller](#direction_controller)

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

![](https://rt-net.github.io/images/raspberry-pi-mouse/joystick_control_keyconfig.png)

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

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking.JPG width=500 />

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
$ ros2 launch raspimouse_ros2_examples object_tracking.launch.py video_device:=/dev/video0
```

This sample publishes two topics: `camera/color/image_raw` for the camera image and `result_image` for the object detection image.
These images can be viewed with [RViz](https://index.ros.org/r/rviz/)
or [rqt_image_view](https://index.ros.org/p/rqt_image_view/).

**Viewing an image may cause the node to behave unstable and not publish cmd_vel or image topics.**

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking_ros2.png width=500 />

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

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_line_trace_sensor.JPG width=500 />

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

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/field_calibration.JPG width=500 />

Then, place Raspberry Pi Mouse to detect a line and press SW1 to sample sensor values on the line.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/line_calibration.JPG width=500 />

Last, place Raspberry Pi Mouse on the line and press SW0 to start line following.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/start_trace.JPG width=500 />

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

### camera_line_follower

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_camera_line_trace_2.png width=500 />

This is an example for line following by RGB camera.

#### Requirements

- Web camera
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- Camera mount
  - [Raspberry Pi Mouse Option kit No.4 \[Webcam mount\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584&language=en)

#### Installation

Install a camera mount and a web camera to Raspberry Pi Mouse, then connect the camera to the Raspberry Pi．

#### How to use

Then, launch nodes with the following command:

```sh
$ ros2 launch raspimouse_ros2_examples camera_line_follower.launch.py video_device:=/dev/video0
```

Place Raspberry Pi Mouse on the line and press SW2 to start line following.

Press SW0 to stop the following.

This sample publishes two topics: `camera/color/image_raw` for the camera image and `result_image` for the object detection image.
These images can be viewed with [RViz](https://index.ros.org/r/rviz/)
or [rqt_image_view](https://index.ros.org/p/rqt_image_view/).

**Viewing an image may cause the node to behave unstable and not publish cmd_vel or image topics.**

**If the line detection accuracy is poor, please adjust the camera's exposure and white balance.**

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/camera_line_trace.png width=500 />

#### Parameters

- `max_brightness`
  - Type: `int`
  - Default: 90
  - Maximum threshold value for image binarisation.
- `min_brightness`
  - Type: `int`
  - Default: 0
  - Minimum threshold value for image binarisation.
- `max_linear_vel`
  - Type: `double`
  - Default: 0.05
  - Maximum linear velocity.
- `max_angular_vel`
  - Type: `double`
  - Default: 0.8
  - Maximum angular velocity.
- `area_threthold`
  - Type: `double`
  - Default: 0.20
  - Threshold value of the area of the line to start following.

```sh
ros2 param set /camera_follower max_brightness 80
```

[back to example list](#how-to-use-examples)

---

### SLAM

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.png width=500 />

SLAM and Navigation examples for Raspberry Pi Mouse is [here](https://github.com/rt-net/raspimouse_slam_navigation_ros2).

[back to example list](#how-to-use-examples)

---

### direction_controller

<img src=https://www.rt-shop.jp/images/RT/RT-USB-9axisIMU.png width=200 /> <img src=https://www.rt-shop.jp/images/RT/%E8%A3%BD%E5%93%81%E5%86%99%E7%9C%9F.JPG height=200>

This is an example to use an IMU sensor for direction control.

#### Requirements

- [USB output 9 degrees IMU sensor module](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3416&language=en)
- [LiDAR Mount](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)
- RT-USB-9axisIMU ROS Package.
  - https://github.com/rt-net/rt_usb_9axisimu_driver

#### Installation

Install the IMU sensor module to the LiDAR mount.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_2.JPG width=500 />

Install the LiDAR mount to the Raspberry Pi Mouse.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_1.JPG width=500 />

#### How to use

Launch nodes on Raspberry Pi Mouse with the following command:

```sh
$ ros2 launch raspimouse_ros2_examples direction_controller.launch.py
```

Then, press SW0 ~ SW2 to change the control mode as following,

- SW0: Calibrate the gyroscope bias and reset a heading angle of Raspberry Pi Mouse to 0 rad.
- SW1: Start a direction control to keep the heading angle to 0 rad.
  - Press SW0 ~ SW2 or tilt the body to sideways to finish the control.
- SW2: Start a direction control to change the heading angle to `-π ~ π rad`.
  - Press SW0 ~ SW2 or tilt the body to sideways to finish the control.

### Troubleshooting

The IMU might not be connected correctly.
Reconnect the USB cable several times and re-execute the above command.

#### Configure


Set parameters to configure gains of a PID controller for the direction control.

```sh
$ ros2 param set /direction_controller p_gain 10.0
Set parameter successful

$ ros2 param set /direction_controller i_gain 0.5
Set parameter successful

$ ros2 param set /direction_controller d_gain 0.0
Set parameter successful
```
#### Parameters

- p_gain
  - Proportional gain of a PID controller for the direction control
  - default: 10.0, min:0.0, max:30.0
  - type: double
- i_gain
  - Integral gain of a PID controller for the direction control
  - default: 0.0, min:0.0, max:5.0
  - type: double
- d_gain
  - Derivative gain of a PID controller for the direction control
  - default: 20.0, min:0.0, max:30.0
  - type: double
- target_angle
  - Target angle for the SW1 control mode.
  - default: 0.0, min:-π, max:+π
  - type: double

#### Publish topics

- heading_angle
  - Heading angle of the robot that calculated from the IMU module sensor values.
  - type: std_msgs/Float64

#### Videos

[![](http://img.youtube.com/vi/ghcCYOh9_MM/sddefault.jpg)](https://youtu.be/ghcCYOh9_MM)

[back to example list](#how-to-use-examples)
