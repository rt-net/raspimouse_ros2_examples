# Examples

Raspberry Pi Mouseを動作させるサンプル集です。

- [Examples](#examples)
  - [joystick/_control](#joystick_control)
  - [object/_tracking](#object_tracking)
  - [line](#line_follower)
  - [camera/_line/_follower](#camera_line_follower)
  - [direction/_controller](#direction_controller)
  - [slam](#slam)

## joystick_control

ジョイスティックコントローラでRaspberryPiMouseを動かすコード例です。

<a href="https://youtu.be/GswxdB8Ia0Y" target="_blank" rel="noopener noreferrer">
  <img src="https://img.youtube.com/vi/GswxdB8Ia0Y/sddefault.jpg" alt="joystick_control" width="650">
</a>

### Usages

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

### Configure

デフォルトのキー割り当てはこちらです。

Logicool Wireless Gamepad F710を使う場合はモード切替スイッチを __D__ (DirectInput Mode)に設定します。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/joystick_control_keyconfig.png width=450 />

[./config/joy_f710.yml](./config/joy_f710.yml)、[./config/joy_dualshock3.yml](./config/joy_dualshock3.yml)
のキー番号を編集することで、キー割り当てを変更できます。

```yaml
button_shutdown_1       : 8
button_shutdown_2       : 9

button_motor_off        : 8
button_motor_on         : 9

button_cmd_enable       : 4
```

[back to example list](#how-to-use-examples)

---

## object_tracking

色情報をもとにオレンジ色のボールの追跡を行うコード例です。
USB接続のWebカメラとOpenCVを使ってボール追跡をします。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking.JPG width=650 />

### Requirements

- Webカメラ
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- カメラマウント
  - [Raspberry Pi Mouse オプションキット No.4 \[Webカメラマウント\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584)
- ボール（Optional）
  - [ソフトボール（オレンジ）](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)
- Software
  - OpenCV
  - v4l-utils

### Usages

次のスクリプトを実行して、カメラの自動調節機能（自動露光，オートホワイトバランス等）を切ります。

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

**画像を表示するとノードの動作が不安定になり、cmd_velや画像トピックが発行されないことがあります。**

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking_ros2.png width=450 />

### Configure

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

[back to example list](#how-to-use-examples)

---

## line_follower

ライントレースのコード例です。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_line_trace_sensor.JPG width=650 />

### Requirements

- ライントレースセンサ
  - [Raspberry Pi Mouse オプションキット No.3 \[ライントレース\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3591)
- フィールドとライン (Optional)

### Usages

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

### Configure

走行速度を変更するには[`./src/line_follower_component.cpp`](./src/line_follower_component.cpp)を編集します。

```cpp
void Follower::publish_cmdvel_for_line_following(void)
{
  const double VEL_LINEAR_X = 0.08;  // m/s
  const double VEL_ANGULAR_Z = 0.8;  // rad/s
  const double LOW_VEL_ANGULAR_Z = 0.5;  // rad/s
```

[back to example list](#how-to-use-examples)

---

## camera_line_follower

RGBカメラによるライントレースのコード例です。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_camera_line_trace_2.png width=650 />

### Requirements

- Webカメラ
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- カメラマウント
  - [Raspberry Pi Mouse オプションキット No.4 \[Webカメラマウント\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584)

### Usages

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

### Configure

ラインの検出精度が悪い場合はカメラの露光やホワイトバランスの調整を行ってください。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/camera_line_trace.png width=450 />

### Parameters

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

```sh
ros2 param set /camera_follower max_brightness 80
```

[back to example list](#how-to-use-examples)

---

## direction_controller

IMUセンサを使用した角度制御のコード例です。

<img src=https://www.rt-shop.jp/images/RT/RT-USB-9axisIMU.png height=280 /><img src=https://www.rt-shop.jp/images/RT/%E8%A3%BD%E5%93%81%E5%86%99%E7%9C%9F.JPG height=280 />

### Requirements

- [USB出力9軸IMUセンサモジュール](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1&products_id=3416&language=ja)
- LiDAR Mount ([Raspberry Pi Mouse オプションキットNo.8 [マルチLiDARマウント]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867))
- RT-USB-9axisIMU ROS Package
  - https://github.com/rt-net/rt_usb_9axisimu_driver

IMUセンサモジュールを取り付けたLiDAR MountをRaspberry Pi Mouseに取り付けます。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_2.JPG width=250 /><img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_1.JPG width=250 />

### Usages

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

> [!NOTE]
> IMUの接続が正常に行われない場合があります。
> その際は、IMUのUSBケーブルを抜き差しした後、コマンドを再度実行してください。

### Parameters

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
  - default: 0.0, min:-π, max:+π
  - SW1（角度制御モード）の目標角度

### Published
- `heading_angle`
  - Type: `std_msgs/Float64`
  - IMUモジュールのセンサ値をもとに計算されたロボットの向き（進行方向の角度）

[back to example list](#how-to-use-examples)

---

## SLAM/Navigation

SLAMとNavigationを行います。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.png height=650 />

> [!NOTE]
> Raspberry Pi MouseでSLAMとNavigationを行うサンプルは[rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2)へ移行しました。

[back to example list](#how-to-use-examples)

---
