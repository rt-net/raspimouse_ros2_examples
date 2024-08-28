^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package raspimouse_ros2_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.1 (2024-08-28)
------------------
* サービスクライアントでexecutorを使用しない (`#59 <https://github.com/rt-net/raspimouse_ros2_examples/issues/59>`_)
* SubscriberとService Clientに別々のcallback_groupを設定 (`#58 <https://github.com/rt-net/raspimouse_ros2_examples/issues/58>`_)
* Contributors: ShotaAk, YusukeKato

2.2.0 (2024-03-05)
------------------
* READMEにSLAM&Navigationパッケージの案内を追加 (`#53 <https://github.com/rt-net/raspimouse_ros2_examples/issues/53>`_)
* Camera_FollowerクラスをCameraFollowerに変更 (`#52 <https://github.com/rt-net/raspimouse_ros2_examples/issues/52>`_)
* Update camera line follower: Set motor power with switch input. Add area_threthold param. (`#51 <https://github.com/rt-net/raspimouse_ros2_examples/issues/51>`_)
* Add velocity parameters for camera_line_follower (`#50 <https://github.com/rt-net/raspimouse_ros2_examples/issues/50>`_)
* カメラライントレースを修正 (`#49 <https://github.com/rt-net/raspimouse_ros2_examples/issues/49>`_)
* Change threthold of line detection
* Add usb_cam dependency (`#48 <https://github.com/rt-net/raspimouse_ros2_examples/issues/48>`_)
* RGBカメラによるライントレースの実装 (`#47 <https://github.com/rt-net/raspimouse_ros2_examples/issues/47>`_)
* リリースのためにCHANGELOG.rstとpackage.xmlを更新 (`#45 <https://github.com/rt-net/raspimouse_ros2_examples/issues/45>`_)
* Contributors: Shota Aoki, ShotaAk, YusukeKato

2.1.0 (2023-11-07)
------------------
* READMEにGazeboでも実行できることを追記 (`#44 <https://github.com/rt-net/raspimouse_ros2_examples/issues/44>`_)
* object_trackingにおいて画像トピックをサブスクライブするように変更 (`#43 <https://github.com/rt-net/raspimouse_ros2_examples/issues/43>`_)
* Contributors: YusukeKato

2.0.0 (2023-08-03)
------------------
* Humble対応 (`#41 <https://github.com/rt-net/raspimouse_ros2_examples/issues/41>`_)
* Contributors: Shuhei Kozasa

1.0.0 (2022-07-28)
------------------
* Update map command (`#38 <https://github.com/rt-net/raspimouse_ros2_examples/issues/38>`_)
* Adds config file for DUALSHOCK4 (`#36 <https://github.com/rt-net/raspimouse_ros2_examples/issues/36>`_)
* Update README for foxy-devel (`#34 <https://github.com/rt-net/raspimouse_ros2_examples/issues/34>`_)
* Remove node\_ prefix from launch files (`#33 <https://github.com/rt-net/raspimouse_ros2_examples/issues/33>`_)
* Use ament_export_targets instead of ament_export_interfaces. (`#31 <https://github.com/rt-net/raspimouse_ros2_examples/issues/31>`_)
* Remove dashing check from CI (`#32 <https://github.com/rt-net/raspimouse_ros2_examples/issues/32>`_)
* Update rviz config to show scan and graph topics (`#29 <https://github.com/rt-net/raspimouse_ros2_examples/issues/29>`_)
* Add descriptions to READMEs for use_pulse_counters param settings (`#28 <https://github.com/rt-net/raspimouse_ros2_examples/issues/28>`_)
* Use joy_linux instead of joy (`#27 <https://github.com/rt-net/raspimouse_ros2_examples/issues/27>`_)
* Update CI to support ROS Foxy (`#26 <https://github.com/rt-net/raspimouse_ros2_examples/issues/26>`_)
* Update package.xml (`#25 <https://github.com/rt-net/raspimouse_ros2_examples/issues/25>`_)
* Install raspimouse2 and imu packages via rosdep command (`#22 <https://github.com/rt-net/raspimouse_ros2_examples/issues/22>`_)
* Add rt_usb_9axisimu_driver dependency to package.xml (`#21 <https://github.com/rt-net/raspimouse_ros2_examples/issues/21>`_)
* Add direction control example (`#18 <https://github.com/rt-net/raspimouse_ros2_examples/issues/18>`_)
* Use images of rt-net/images repo. (`#17 <https://github.com/rt-net/raspimouse_ros2_examples/issues/17>`_)
* Add lidar example (`#14 <https://github.com/rt-net/raspimouse_ros2_examples/issues/14>`_)
* Turn on/off leds with joy inputs (`#15 <https://github.com/rt-net/raspimouse_ros2_examples/issues/15>`_)
* Update Gamepad F710 usage in README (`#13 <https://github.com/rt-net/raspimouse_ros2_examples/issues/13>`_)
* Use multi threads in the object tracking example to stabilize the tracking (`#11 <https://github.com/rt-net/raspimouse_ros2_examples/issues/11>`_)
* update video link (`#12 <https://github.com/rt-net/raspimouse_ros2_examples/issues/12>`_)
* Merge teleop_joy launch files into one file. (`#10 <https://github.com/rt-net/raspimouse_ros2_examples/issues/10>`_)
* Add line follower examples (`#9 <https://github.com/rt-net/raspimouse_ros2_examples/issues/9>`_)
* Add object tracking sample (`#8 <https://github.com/rt-net/raspimouse_ros2_examples/issues/8>`_)
* Rename launch files (`#7 <https://github.com/rt-net/raspimouse_ros2_examples/issues/7>`_)
* Refactoring (`#6 <https://github.com/rt-net/raspimouse_ros2_examples/issues/6>`_)
* Support remote control (`#5 <https://github.com/rt-net/raspimouse_ros2_examples/issues/5>`_)
* Add Joystic example (`#4 <https://github.com/rt-net/raspimouse_ros2_examples/issues/4>`_)
* Add industrial_ci test settings (`#3 <https://github.com/rt-net/raspimouse_ros2_examples/issues/3>`_)
* Fix teleop.launch for flake8 check (`#2 <https://github.com/rt-net/raspimouse_ros2_examples/issues/2>`_)
* Add github workflow (`#1 <https://github.com/rt-net/raspimouse_ros2_examples/issues/1>`_)
* Contributors: Daisuke Sato, Shota Aoki, Shuhei Kozasa
