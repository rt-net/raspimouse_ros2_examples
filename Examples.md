# Examples

## joystick_control

ジョイスティックコントローラでRaspberryPiMouseを動かすコード例です。

<a href="https://youtu.be/GswxdB8Ia0Y" target="_blank" rel="noopener noreferrer">
  <img src="https://img.youtube.com/vi/GswxdB8Ia0Y/sddefault.jpg" alt="joystick_control" width="650">
</a>

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

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/joystick_control_keyconfig.png width=450 />

[back to example list](#how-to-use-examples)

---

## object_tracking

色情報をもとにオレンジ色のボールの追跡を行うコード例です。
USB接続のWebカメラとOpenCVを使ってボール追跡をします。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking.JPG width=650 />

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

[back to example list](#how-to-use-examples)

---

## line_follower

ライントレースのコード例です。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_line_trace_sensor.JPG width=650 />

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

[back to example list](#how-to-use-examples)

---

## camera_line_follower

RGBカメラによるライントレースのコード例です。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_camera_line_trace_2.png width=650 />

```bash
# Terminal 1
ros2 run ...

# Terminal 2
ros2 launch ...
```

次のコマンドでノードを起動します。

```sh
$ ros2 launch raspimouse_ros2_examples camera_line_follower.launch.py video_device:=/dev/video0
```

ライン上にRaspberry Pi Mouseを置き、SW2を押してライントレースを開始します。
停止させる場合はSW0を押します。

カメラ画像は`camera/color/image_raw`、物体検出画像は`result_image`というトピックとして発行されます。
これらの画像は[RViz](https://index.ros.org/r/rviz/)や[rqt_image_view](https://index.ros.org/p/rqt_image_view/)
で表示できます。

[back to example list](#how-to-use-examples)

---

## direction_controller

IMUセンサを使用した角度制御のコード例です。

<img src=https://www.rt-shop.jp/images/RT/RT-USB-9axisIMU.png height=280 /><img src=https://www.rt-shop.jp/images/RT/%E8%A3%BD%E5%93%81%E5%86%99%E7%9C%9F.JPG height=280 />


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

[back to example list](#how-to-use-examples)

---

## SLAM

> [!NOTE]
> Raspberry Pi MouseでSLAMとNavigationを行うサンプルは[rt-net/raspimouse_slam_navigation_ros2](https://github.com/rt-net/raspimouse_slam_navigation_ros2)へ移行しました。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_toolbox_ros2.png height=650 />

[back to example list](#how-to-use-examples)

---
