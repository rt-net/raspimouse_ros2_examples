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
- [object_tracking](#object_tracking)
- [line_follower](#line_follower)

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
$ ros2 launch raspimouse_ros2_examples teleop_joy_with_mouse.launch.py joydev:="/dev/input/js0"

# Control from remote computer
## on RaspberryPiMouse
$ ros2 run raspimouse raspimouse
## on remote computer
$ ros2 launch raspimouse_ros2_examples teleop_joy.launch.py
```

デフォルトのキー割り当てはこちらです。

![joystick_control_keyconfig](https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/joystick_control_keyconfig.png)

#### Configure

[./launch/teleop_joy.launch.py](./launch/teleop_joy.launch.py)を編集してコンフィグファイルを切り替えます。

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

### object_tracking

<img src="https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/object_tracking.JPG" width=500 />

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

<img src="https://github.com/rt-net/raspimouse_ros2_examples/blob/images/object_tracking_images.png" width=500 />

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

[![object_tracking](http://img.youtube.com/vi/U6_BuvrjyFc/sddefault.jpg)](https://youtu.be/U6_BuvrjyFc)

[back to example list](#how-to-use-examples)

--- 

### line_follower

<img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_line_trace_sensor.JPG" width=500 />

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

<img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/field_calibration.JPG" width=500 />

次に、センサとラインが重なるようにRaspberry Pi Mouseを置き、SW1を押してライン上のセンサ値をサンプリングします。

<img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/line_calibration.JPG" width=500 />

最後に、ライン上にRaspberry Pi Mouseを置き、SW0を押してライントレースを開始します。

<img src="https://github.com/rt-net/raspimouse_ros_examples/blob/images/start_trace.JPG" width=500 />

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
