[English](README.en.md) | [日本語](README.md)

# raspimouse_ros2_examples

[![industrial_ci](https://github.com/rt-net/raspimouse_ros2_examples/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse_ros2_examples/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

Raspberry Pi MouseのROS 2サンプルコード集です。

ROS1のサンプルコード集は[こちら](https://github.com/rt-net/raspimouse_ros_examples)。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/raspberry_pi_mouse.JPG width=500 />

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
$ git clone -b dashing-devel https://github.com/rt-net/raspimouse_ros2_examples

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
- [object_tracking](#object_tracking)
- [line_follower](#line_follower)
- [SLAM](#slam)
- [direction_controller](#direction_controller)

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

デフォルトのキー割り当てはこちらです。

Logicool Wireless Gamepad F710を使う場合はモード切替スイッチを __D__ (DirectInput Mode)に設定します。

![](https://rt-net.github.io/images/raspberry-pi-mouse/joystick_control_keyconfig.png)

#### Configure

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

### object_tracking

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking.JPG width=500 />

色情報をもとにオレンジ色のボールの追跡を行うコード例です。
USB接続のWebカメラとOpenCVを使ってボール追跡をします。

#### Requirements 

- Webカメラ
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- カメラマウント
  - [Raspberry Pi Mouse オプションキット No.4 \[Webカメラマウント\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584)
- ボール（Optional）
  - [ソフトボール（オレンジ）](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)
- Software
  - OpenCV
  - v4l-utils

#### Installation

Raspberry Pi Mouseにカメラマウントを取り付け、WebカメラをRaspberry Piに接続します。

#### How to use

次のスクリプトを実行して、カメラの自動調節機能（自動露光，オートホワイトバランス等）を切ります。

```sh
$ cd ~/ros2_ws/src/raspimouse_ros2_examples/config
$ ./configure_camera.bash
```

次のコマンドでノードを起動します。

```sh
$ ros2 launch raspimouse_ros2_examples object_tracking.launch.py
```

カメラ画像は`raw_image`、物体検出画像は`result_image`というトピックとして発行されます。
これらの画像は[RViz](https://index.ros.org/r/rviz/)
や[rqt_image_view](https://index.ros.org/doc/ros2/Tutorials/RQt-Overview-Usage/)
で表示できます。

**画像を表示するとノードの動作が不安定になり、cmd_velや画像トピックが発行されないことがあります。**

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking_ros2.png width=500 />

#### Configure

追跡対象の色を変更するには
[`./src/object_tracking_component.cpp`](./src/object_tracking_component.cpp)
を編集します。

物体検出精度が悪い時にはカメラの露光や関数内のパラメータを調整して下さい。

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

ライントレースのコード例です。

#### Requirements

- ライントレースセンサ
  - [Raspberry Pi Mouse オプションキット No.3 \[ライントレース\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3591)
- フィールドとライン (Optional)

#### Installation

Raspberry Pi Mouseにライントレースセンサを取り付けます。

#### How to use

次のコマンドでノードを起動します。

```sh
$ ros2 launch raspimouse_ros2_examples line_follower.launch.py
```

Raspberry Pi Mouseをフィールドに置き、SW2を押してフィールド上のセンサ値をサンプリングします。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/field_calibration.JPG width=500 />

次に、センサとラインが重なるようにRaspberry Pi Mouseを置き、SW1を押してライン上のセンサ値をサンプリングします。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/line_calibration.JPG width=500 />

最後に、ライン上にRaspberry Pi Mouseを置き、SW0を押してライントレースを開始します。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/start_trace.JPG width=500 />

もう一度SW0を押すとライントレースを停止します。

#### Configure

走行速度を変更するには[`./src/line_follower_component.cpp`](./src/line_follower_component.cpp)を編集します。

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

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.png width=500 />

LiDARと[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
を使ってSLAM（自己位置推定と地図作成）を行うサンプルです。

#### Requirements 

- LiDAR
  <!-- - [~URG-04LX-UG01~](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1296&products_id=2816)
  - [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1) -->
  - [LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676)
- [LiDAR Mount](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)
- Joystick Controller (Optional)
  
#### Installation

Raspberry Pi MouseにLiDARを取り付けます。

<!-- - URG-04LX-UG01
  - <img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_urg.JPG" width=500 />
- RPLIDAR A1
  - <img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_rplidar.png" width=500 /> -->
- LDS-01
  - <img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_lds01.JPG width=500 />
  
#### How to use

Raspberry Pi Mouse上で次のコマンドでノードを起動します。

```sh
# LDS
$ ros2 launch raspimouse_ros2_examples mouse_with_lidar.launch.py lidar:=lds
```

Raspberry Pi Mouseを動かすため`teleop_joy.launch.py`を起動します

```sh
# Use DUALSHOCK 3
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py joydev:="/dev/input/js0" joyconfig:=dualshock3 mouse:=false
```

次のコマンドでslam_toolboxパッケージを起動します。（Remote computerでの実行推奨）

```sh
$ ros2 launch raspimouse_ros2_examples slam.launch.py
```

Raspberry Pi Mouseを動かして地図を作成します。

次のコマンドで作成した地図を保存します。

```sh
$ mkdir ~/maps
$ ros2 run nav2_map_server map_saver -f ~/maps/mymap
```

#### Configure

[./config/mapper_params_offline.yaml](./config/mapper_params_offline.yaml)で[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)パッケージのパラメータを調節します。

<!-- #### Videos

[![slam_urg](http://img.youtube.com/vi/gWozU47UqVE/sddefault.jpg)](https://youtu.be/gWozU47UqVE)

[![slam_urg](http://img.youtube.com/vi/hV68UqAntfo/sddefault.jpg)](https://youtu.be/hV68UqAntfo) -->

[back to example list](#how-to-use-examples)

---

### direction_controller

<img src=https://www.rt-net.jp/wp-content/uploads/2018/02/img-usb9s_01.png width=500 />

IMUセンサを使用した角度制御のコード例です。

#### Requirements

- [USB出力9軸IMUセンサモジュール](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1&products_id=3416&language=ja)
- LiDAR Mount ([Raspberry Pi Mouse オプションキットNo.8 [マルチLiDARマウント]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867))
- RT-USB-9axisIMU ROS Package (`dashing-devel` branch).
  - https://github.com/rt-net/rt_usb_9axisimu_driver

#### Installation

LiDAR MountにIMUセンサモジュールを取り付けます。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_2.JPG width=500 />

Raspberry Pi Mouse にLiDAR Mountを取り付けます。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_1.JPG width=500 />

#### How to use

次のコマンドでノードを起動します。

```sh
$ ros2 launch raspimouse_ros2_examples direction_controller.launch.py
```

SW0 ~ SW2を押して動作モードを切り替えます。

- SW0: ジャイロセンサのバイアスをキャリブレーションし、ラズパイマウスの方位角を`0 rad`にリセットします
- SW1: 方位角を`0 rad`に維持する角度制御を開始します
  - SW0 ~ SW2を押すか、ラズパイマウス本体を横に傾けると終了します
- SW2: 方位角を`-π ~ π rad`に変化させる角度制御を開始します
  - SW0 ~ SW2を押すか、ラズパイマウス本体を横に傾けると終了します

#### Configure

パラメータで角度制御に使うPIDゲインを変更できます。

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
