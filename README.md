[English](README.en.md) | [日本語](README.md)

# raspimouse_ros2_examples

<!-- 本リポジトリのワークフローステータスバッジを添付します（詳細：https://wiki.moon-rabbit.rt-net.jp/670f811056b3aca9041b9aa3）-->
[![industrial_ci](https://github.com/rt-net/<リポジトリのパス>/actions/workflows/industrial_ci.yml/badge.svg?branch=<対象のブランチ>)](https://github.com/rt-net/<リポジトリのパス>/actions/workflows/industrial_ci.yml)

Raspberry Pi MouseのROS 2サンプルコード集です。

ROS1のサンプルコード集は[こちら](https://github.com/rt-net/raspimouse_ros_examples/blob/master/README.md)。

Gazebo（シミュレータ）でも動作します。詳細は[こちら](https://github.com/rt-net/raspimouse_sim/blob/ros2/README.md)。

![raspberry_pi_mouse](https://rt-net.github.io/images/raspberry-pi-mouse/raspberry_pi_mouse.JPG)

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
    - [<Sample名>](#sample名)
  - [Packages](#packages)
  - [Topics](#topics)
    - [Subscribed](#subscribed)
    - [Published](#published)
  - [Services](#services)
  - [Actions](#actions)
  - [Parameters](#parameters)
  - [<etc...Lifecycle,Description)>](#etc-lifecycle-description等)
  - [License](#license)
  - [Contributing](#contributing)
  - [Contributors](#contributors)

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

```bash
sudo apt install <パッケージ名>
```

### Source Build

```bash
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

```bash
# Terminal 1
ros2 run ...

# Terminal 2
ros2 launch ...
```

## How to Use Examples

サンプルプログラムの詳細な動作方法は、[EXAMPLES](./EXAMPLES.md)で説明しています。

### <Sample名>

サンプルプログラムの説明

<!-- 可能な限り画像や動画を用いて説明します -->
![画像](画像ファイルパス)
[![動画名](表示画像パス)](Youtubeのリンクパス)

<!-- 動作コマンドや注意事項を説明します -->

```bash
# Terminal 1
ros2 run ...

# Terminal 2
ros2 launch ...
```

## Topics

<!-- 説明が必要な場合は記述します -->

### Subscribed

- `<Topic名>`
  - Type: `<型名>`
  - 説明の記述
  
### Published

- `<Topic名>`
  - Type: `<型名>`
  - 説明の記述

## Services

<!-- 説明が必要な場合は記述します -->

- `<Service名>`
  - Type: `<型名>`
  - 説明の記述

## Actions

<!-- 説明が必要な場合は記述します -->

- `<Action名>`
  - Type: `<型名>`
  - 説明の記述

## Parameters

<!-- 説明が必要な場合は記述します -->

- `<Parameter名>`
  - Type: `<型名>`
  - Default: `<デフォルト値>`
  - 説明の記述

## <etc... (Lifecycle, Description、等)>

<!-- Lifecycleの管理方法や独自の使用方法があれば記述します -->

## License

(C) 2022 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。 特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

## Contributing

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、
それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md)に従ってください。
