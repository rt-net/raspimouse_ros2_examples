[English](README.en.md) | [日本語](README.md)

# raspimouse_ros2_examples

[![industrial_ci](https://github.com/rt-net/raspimouse_ros2_examples/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse_ros2_examples/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

Raspberry Pi MouseのROS 2サンプルコード集です。

Gazebo（シミュレータ）で動作させる場合は、[rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim/blob/ros2/README.md)パッケージを参照してください。

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
    - [Joystick Control](#joystick-control)
    - [Object Tracking](#object-tracking)
    - [Line Follower](#line-follower)
    - [Camera Line Follower](#camera-line-follower)
    - [Direction Controller](#direction-controller)
    - [SLAM & Navigation](#slam--navigation)([rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2)に移動しました)
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

ジョイスティックコントローラで、Raspberry Pi Mouseをリモート操作します。

- 対応コントローラ
  - [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
  - [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)

```sh
# on RaspberryPiMouse
$ ros2 run raspimouse raspimouse
# on remote computer
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py mouse:=false
```

## How to Use Examples

Raspberry Pi Mouseを動作させるサンプル集です。

- Examples
  - [Joystick Control](#joystick-control)
  - [Object Tracking](#object-tracking)
  - [Line Follower](#line-follower)
  - [Camera Line Follower](#camera-line-follower)
  - [Direction Controller](#direction-controller)
  - [SLAM & Navigation](#slam--navigation)([rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2)に移動しました)

### Joystick Control

ジョイスティックコントローラでRaspberryPiMouseを動かすコード例です。

<a href="https://youtu.be/GswxdB8Ia0Y" target="_blank" rel="noopener noreferrer">
  <img src="https://img.youtube.com/vi/GswxdB8Ia0Y/sddefault.jpg" alt="joystick_control" width="650">
</a>

<details>
<summary>Details</summary>

#### Usage

次のコマンドでノードを起動します。

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

デフォルトのキー割り当てはこちらです。

Logicool Wireless Gamepad F710を使う場合はモード切替スイッチを __D__ (DirectInput Mode)に設定します。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/joystick_control_keyconfig.png width=450 />

[./config/joy_f710.yml](./config/joy_f710.yml)、[./config/joy_dualshock3.yml](./config/joy_dualshock3.yml)のキー番号を編集することで、キー割り当てを変更できます。

```yaml
button_shutdown_1       : 8
button_shutdown_2       : 9

button_motor_off        : 8
button_motor_on         : 9

button_cmd_enable       : 4
```

</details>

[back to example list](#examples)

---

### Object Tracking

色情報をもとにオレンジ色のボールの追跡を行うコード例です。
USB接続のWebカメラとOpenCVを使ってボール追跡をします。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking.JPG width=650 />

<details>
<summary>Details</summary>

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

#### Usage

次のスクリプトを実行して、カメラの自動調節機能（自動露光、オートホワイトバランス等）を切ります。

```sh
$ cd ~/ros2_ws/src/raspimouse_ros2_examples/config
$ ./configure_camera.bash
```

次のコマンドでノードを起動します。

```sh
$ ros2 launch raspimouse_ros2_examples object_tracking.launch.py video_device:=/dev/video0
```

カメラ画像は`camera/color/image_raw`、物体検出画像は`result_image`というトピックとして発行されます。
これらの画像は[RViz](https://index.ros.org/r/rviz/)や[rqt_image_view](https://index.ros.org/p/rqt_image_view/)で表示できます。

> [!NOTE]
> 画像を表示するとノードの動作が不安定になり、cmd_velや画像トピックが発行されないことがあります。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking_ros2.png width=450 />

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

</details>

[back to example list](#examples)

---

### Line Follower

ライントレースのコード例です。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_line_trace_sensor.JPG width=650 />

<details>
<summary>Details</summary>

#### Requirements

- ライントレースセンサ
  - [Raspberry Pi Mouse オプションキット No.3 \[ライントレース\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3591)
- フィールドとライン (Optional)

#### Usage

次のコマンドでノードを起動します。

```sh
$ ros2 launch raspimouse_ros2_examples line_follower.launch.py
```

Raspberry Pi Mouseをフィールドに置き、SW2を押してフィールド上のセンサ値をサンプリングします。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/field_calibration.JPG width=450 />

次に、センサとラインが重なるようにRaspberry Pi Mouseを置き、SW1を押してライン上のセンサ値をサンプリングします。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/line_calibration.JPG width=450 />

最後に、ライン上にRaspberry Pi Mouseを置き、SW0を押してライントレースを開始します。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/start_trace.JPG width=450 />

もう一度SW0を押すとライントレースを停止します。

<a href="https://youtu.be/oPm0sW2V_tY" target="_blank" rel="noopener noreferrer">
  <img src="http://img.youtube.com/vi/oPm0sW2V_tY/sddefault.jpg" alt="joystick_control" width="450">
</a>

#### Configure

走行速度を変更するには[`./src/line_follower_component.cpp`](./src/line_follower_component.cpp)を編集します。

```cpp
void Follower::publish_cmdvel_for_line_following(void)
{
  const double VEL_LINEAR_X = 0.08;  // m/s
  const double VEL_ANGULAR_Z = 0.8;  // rad/s
  const double LOW_VEL_ANGULAR_Z = 0.5;  // rad/s
```

</details>

[back to example list](#examples)

---

### Camera Line Follower

RGBカメラによるライントレースのコード例です。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_camera_line_trace_2.png width=650 />

<details>
<summary>Details</summary>

#### Requirements

- Webカメラ
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- カメラマウント
  - [Raspberry Pi Mouse オプションキット No.4 \[Webカメラマウント\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584)

#### Usage

次のコマンドでノードを起動します。

```sh
$ ros2 launch raspimouse_ros2_examples camera_line_follower.launch.py video_device:=/dev/video0
```

ライン上にRaspberry Pi Mouseを置き、SW2を押してライントレースを開始します。
停止させる場合はSW0を押します。

カメラ画像は`camera/color/image_raw`、物体検出画像は`result_image`というトピックとして発行されます。
これらの画像は[RViz](https://index.ros.org/r/rviz/)や[rqt_image_view](https://index.ros.org/p/rqt_image_view/)
で表示できます。

> [!NOTE]
> 画像を表示するとノードの動作が不安定になり、cmd_velや画像トピックが発行されないことがあります。

#### Configure

ラインの検出精度が悪い場合はカメラの露光やホワイトバランスの調整を行ってください。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/camera_line_trace.png width=450 />

#### Parameters

- `max_brightness`
  - Type: `int`
  - Default: 90
  - 画像の2値化のしきい値の最大値
- `min_brightness`
  - Type: `int`
  - Default: 0
  - 画像の2値化のしきい値の最小値
- `max_linear_vel`
  - Type: `double`
  - Default: 0.05
  - 直進速度の最大値
- `max_angular_vel`
  - Type: `double`
  - Default: 0.8
  - 旋回速度の最大値
- `area_threshold`
  - Type: `double`
  - Default: 0.20
  - 走行を開始するためのライン面積のしきい値

各種パラメータを設定する際は、以下のコマンドを実行します。

```sh
ros2 param set /camera_follower max_brightness 80
```

</details>

[back to example list](#examples)

---

### Direction Controller

IMUセンサを使用した角度制御のコード例です。

<img src=https://www.rt-shop.jp/images/RT/RT-USB-9axisIMU.png height=280 /><img src=https://www.rt-shop.jp/images/RT/%E8%A3%BD%E5%93%81%E5%86%99%E7%9C%9F.JPG height=280 />

<details>
<summary>Details</summary>

#### Requirements

- [USB出力9軸IMUセンサモジュール](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1&products_id=3416&language=ja)
- LiDAR Mount ([Raspberry Pi Mouse オプションキットNo.8 [マルチLiDARマウント]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867))
- RT-USB-9axisIMU ROS Package
  - https://github.com/rt-net/rt_usb_9axisimu_driver

IMUセンサモジュールを取り付けたLiDAR MountをRaspberry Pi Mouseに取り付けます。詳細は、[マルチLiDARマウント組み立てマニュアル](https://rt-net.jp/wp-content/uploads/2020/04/RaspberryPiMouseOptionKitManual_No08.pdf)を参照してください。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_2.JPG width=250 /> <img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_1.JPG width=250 />

#### Usage

次のコマンドでノードを起動します。

```sh
$ ros2 launch raspimouse_ros2_examples direction_controller.launch.py
```

SW0-SW2を押して動作モードを切り替えます。

- SW0: ジャイロセンサのバイアスをキャリブレーションし、ラズパイマウスの方位角を`0`[rad]にリセットします
- SW1: 方位角を`0`[rad]に維持する角度制御を開始します
  - SW0–SW2を押すか、ラズパイマウス本体を横に傾けると終了します
- SW2: 方位角を`-π ~ π`[rad]に変化させる角度制御を開始します
  - SW0–SW2を押すか、ラズパイマウス本体を横に傾けると終了します

> [!NOTE]
> IMUの接続が正常に行われない場合があります。
> その際は、IMUのUSBケーブルを抜き差しした後、コマンドを再度実行してください。

#### Parameters

- `p_gain`
  - Type: `double`
  - Default: 10.0, min:0.0, max:30.0
  - 角度制御用PIDコントローラの比例ゲイン
- `i_gain`
  - Type: `double`
  - Default: 0.0, min:0.0, max:5.0
  - 角度制御用PIDコントローラの積分ゲイン
- `d_gain`
  - Type: `double`
  - Default: 20.0, min:0.0, max:30.0
  - 角度制御用PIDコントローラの微分ゲイン
- `target_angle`
  - Type: `double`
  - Default: 0.0, min:-π, max:+π
  - SW1（角度制御モード）の目標角度

#### Published
- `heading_angle`
  - Type: `std_msgs/Float64`
  - IMUモジュールのセンサ値をもとに計算されたロボットの向き（進行方向の角度）



[back to example list](#examples)

---

### SLAM & Navigation

SLAMとNavigationを行います。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.png height=650 />

> [!NOTE]
> Raspberry Pi MouseでSLAMとNavigationを行うサンプルは[rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2)へ移行しました。

[back to example list](#examples)

---

## License

(C) 2022 RT Corporation <support@rt-net.jp>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。
特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。
ライセンスの全文はLICENSEまたはhttps://www.apache.org/licenses/LICENSE-2.0から確認できます。

## Contributing

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、
それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md)に従ってください。
