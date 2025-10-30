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
sudo apt install ros-$ROS_DISTRO-raspimouse-ros2-examples
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

## How to Use Examples

Sample programs for the Raspberry Pi Mouse.

- [Examples](#examples)
  - [Joystick Control](#joystick-control)
  - [Object Tracking](#object-tracking)
  - [Line Follower](#line-follower)
  - [Camera Line Follower](#camera-line-follower)
  - [Direction Controller](#direction-controller)
  - [SLAM & Navigation](#slam--navigation) (Relocated [rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2))

### Joystick Control

This is an example with a joystick controller to operate a Raspberry Pi Mouse.

<a href="https://youtu.be/GswxdB8Ia0Y" target="_blank" rel="noopener noreferrer">
  <img src="https://img.youtube.com/vi/GswxdB8Ia0Y/sddefault.jpg" alt="joystick_control" width="650">
</a>

<details>
<summary>Details</summary>

#### Usage

Launch nodes with the following command:

```sh
# Controlled directly on Raspberry Pi Mouse
## Use F710
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py joydev:="/dev/input/js0" joyconfig:=f710 mouse:=true
## Use DUALSHOCK 3
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py joydev:="/dev/input/js0" joyconfig:=dualshock3 mouse:=true

# Control from remote computer
## on RaspberryPiMouse
$ ros2 run raspimouse raspimouse
## on remote computer
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py mouse:=false
```

#### Configure

This picture shows the default key configuration.

To use Logicool Wireless Gamepad F710, set the input mode to  __D__ (DirectInput Mode).

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/joystick_control_keyconfig.png width=450 />

Key assignments can be edited with key numbers in [./config/joy_f710.yml](./config/joy_f710.yml) or [./config/joy_dualshock3.yml](./config/joy_dualshock3.yml).

```yaml
button_shutdown_1       : 8
button_shutdown_2       : 9

button_motor_off        : 8
button_motor_on         : 9

button_cmd_enable       : 4
```

</details>

[Back to example list](#how-to-use-examples)

---

### Object Tracking

This is a code example for tracking an orange ball based on color information.
The ball tracking is performed with a USB webcam and the OpenCV library.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking.JPG width=650 />

<details>
<summary>Details</summary>

#### Requirements

- Web camera
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- Camera mount
  - [Raspberry Pi Mouse Option kit No.4 \[Webcam mount\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584&language=en)
- Orange ball (Optional)
  - [Soft Ball (Orange)](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701&language=en)
- Software
  - OpenCV
  - v4l-utils

#### Usage

Disable the automatic camera adjustment parameters (autofocus, auto white balance, etc.) with the following command:

```sh
$ cd ~/ros2_ws/src/raspimouse_ros2_examples/config
$ ./configure_camera.bash
```

Then, launch nodes with the following command:

```sh
$ ros2 launch raspimouse_ros2_examples object_tracking.launch.py video_device:=/dev/video0
```

This sample publishes two topics: `camera/color/image_raw` for the camera image and `result_image` for the object detection image.  
These images can be viewed with [RViz](https://index.ros.org/r/rviz/) or [rqt_image_view](https://index.ros.org/p/rqt_image_view/).

> :warning: Note
>
> Viewing the images may cause the node to become unstable, resulting in cmd_vel or image topics not being published.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking_ros2.png width=450 />

#### Configure

Edit [`./src/object_tracking_component.cpp`](./src/object_tracking_component.cpp) to change the color of the tracking target.

If the object detection accuracy is poor, adjust the camera exposure and the parameters in the function.

```cpp
void Tracker::tracking(const cv::Mat & input_frame, cv::Mat & result_frame)
{
  cv::inRange(hsv, cv::Scalar(9, 100, 100), cv::Scalar(29, 255, 255), extracted_bin);  // Orange
  // cv::inRange(hsv, cv::Scalar(60, 100, 100), cv::Scalar(80, 255, 255), extracted_bin);  // Green
  // cv::inRange(hsv, cv::Scalar(100, 100, 100), cv::Scalar(120, 255, 255), extracted_bin);  // Blue
```

</details>

[Back to example list](#how-to-use-examples)

---

### Line Follower

This is an example for line following.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_line_trace_sensor.JPG width=650 />

<details>
<summary>Details</summary>

#### Requirements

- Line following sensor
  - [Raspberry Pi Mouse Option kit No.3 \[Line follower\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3591&language=en)
- Field and lines for following (Optional)

#### Usage

Launch nodes with the following command:

```sh
$ ros2 launch raspimouse_ros2_examples line_follower.launch.py
```

Next, place the Raspberry Pi Mouse on a field and press SW2 to sample sensor values on the field.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/field_calibration.JPG width=450 />

Then, place the Raspberry Pi Mouse on the line and press SW1 to sample sensor values.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/line_calibration.JPG width=450 />

Finally, place the Raspberry Pi Mouse on the line and press SW0 to start line following.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/start_trace.JPG width=450 />

Press SW0 again to stop the line following.

<a href="https://youtu.be/oPm0sW2V_tY" target="_blank" rel="noopener noreferrer">
  <img src="http://img.youtube.com/vi/oPm0sW2V_tY/sddefault.jpg" alt="line_follwer" width="450">
</a>

#### Configure

Edit [`./src/line_follower_component.cpp`](./src/line_follower_component.cpp) to change the robot velocity.

```cpp
void Follower::publish_cmdvel_for_line_following(void)
{
  const double VEL_LINEAR_X = 0.08;  // [m/s]
  const double VEL_ANGULAR_Z = 0.8;  // [rad/s]
  const double LOW_VEL_ANGULAR_Z = 0.5;  // [rad/s]
```

</details>

[Back to example list](#how-to-use-examples)

---

### Camera Line Follower

This is an example for line following by RGB camera.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_camera_line_trace_2.png width=650 />

<details>
<summary>Details</summary>

#### Requirements

- Web camera
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- Camera mount
  - [Raspberry Pi Mouse Option kit No.4 \[Webcam mount\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584&language=en)

#### Usage

Launch nodes with the following command:

```sh
$ ros2 launch raspimouse_ros2_examples camera_line_follower.launch.py video_device:=/dev/video0
```

Place Raspberry Pi Mouse on the line and press SW2 to start line following.
Press SW0 to stop the line following.

This sample publishes two topics: `camera/color/image_raw` for the camera image and `result_image` for the object detection image.
These images can be viewed in [RViz](https://index.ros.org/r/rviz/) or [rqt_image_view](https://index.ros.org/p/rqt_image_view/).

> :warning: Note
>
> Viewing the images may cause the node to become unstable, resulting in cmd_vel or image topics not being published.

#### Configure

If the line detection accuracy is poor, adjust the camera exposure and white balance.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/camera_line_trace.png width=450 />

#### Parameters

- `max_brightness`
  - Type: `int`
  - Default: 90
  - Maximum threshold value for image binarization.
- `min_brightness`
  - Type: `int`
  - Default: 0
  - Minimum threshold value for image binarization.
- `max_linear_vel`
  - Type: `double`
  - Default: 0.05
  - Maximum linear velocity.
- `max_angular_vel`
  - Type: `double`
  - Default: 0.8
  - Maximum angular velocity.
- `area_threshold`
  - Type: `double`
  - Default: 0.20
  - Threshold value of the area of the line to start following.

Run the following command to set the parameters:

```sh
ros2 param set /camera_follower max_brightness 80
```

</details>

[Back to example list](#how-to-use-examples)

---

### Direction Controller

This is an example of using an IMU sensor for direction control.

<img src=https://www.rt-shop.jp/images/RT/RT-USB-9axisIMU.png height=280 /><img src=https://www.rt-shop.jp/images/RT/%E8%A3%BD%E5%93%81%E5%86%99%E7%9C%9F.JPG height=280 />

<details>
<summary>Details</summary>

#### Requirements

- [USB output 9 degrees IMU sensor module](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3416&language=en)
- [LiDAR Mount](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)
- RT-USB-9axisIMU ROS Package.
  - https://github.com/rt-net/rt_usb_9axisimu_driver

Attach the LiDAR mount with the IMU sensor module to the Raspberry Pi Mouse. For details, refer to the [Multi-LiDAR Mount Assembly Manual (in Japanese)](https://rt-net.jp/wp-content/uploads/2020/04/RaspberryPiMouseOptionKitManual_No08.pdf).

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_2.JPG width=250 /> <img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_1.JPG width=250 />

#### Usage

Launch nodes on the Raspberry Pi Mouse with the following command:

```sh
$ ros2 launch raspimouse_ros2_examples direction_controller.launch.py
```

Then, press SW0–SW2 to change the control mode as follows:

- SW0: Calibrate the gyroscope bias and reset the Raspberry Pi Mouse's heading angle to `0`[rad].
- SW1: Start direction control to keep the heading angle at `0`[rad].
  - Press SW0–SW2 or tilt the body sideways to terminate the control.
- SW2: Start direction control to change the heading angle between `-π` and `π`[rad].
  - Press SW0–SW2 or tilt the body sideways to terminate the control.

> :warning: Note
>
> The IMU might not be connected correctly.
> If this happens, unplug and reconnect the USB cable, then run the above command again.

#### Parameters

- `p_gain`
  - Type: `double`
  - Default: 10.0, min:0.0, max:30.0
  - Proportional gain of a PID controller for the direction control
- `i_gain`
  - Type: `double`
  - Default: 0.0, min:0.0, max:5.0
  - Integral gain of a PID controller for the direction control
- `d_gain`
  - Type: `double`
  - Default: 20.0, min:0.0, max:30.0
  - Derivative gain of a PID controller for the direction control
- `target_angle`
  - Type: `double`
  - Default: 0.0, min:-π, max:+π
  - Target angle for the SW1 (direction control mode).

#### Published

- `heading_angle`
  - Type: `std_msgs/Float64`
  - Heading angle of the robot calculated from IMU module sensor values

</details>

[Back to example list](#how-to-use-examples)

---

### SLAM & Navigation

This is an example of SLAM & Navigation.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.png height=650 />

> :warning: Note
>
> The sample for SLAM and Navigation with Raspberry Pi Mouse has been moved to [rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2).

[Back to example list](#how-to-use-examples)

---

## License

(C) 2022 RT Corporation <support@rt-net.jp>

Each file is licensed as stated in their headers.  
If no license is specified, the file is licensed under the Apache License, Version 2.0.  
The full license text is available in the [LICENSE](./LICENSE) file or at [https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0).

## Contributing

- This software is open source, but its development is not open.
- This software is essentially provided as open source software on an “AS IS” (in its current state) basis.
- No free support is available for this software.
- Requests for bug fixes and corrections of typographical errors are always accepted; however, requests for additional features will be subject to our internal guidelines. For further details, please refer to the [Contribution Guidelines](https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md).
