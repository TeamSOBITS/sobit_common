# SOBIT Common

SOBIT Commonのパッケージでは、SOBITSとしてよく使われるレポジトリとライブラリをまとめています。

## インストール
1. まず、本レポジトリの必要なパッケージをインストールします。

```bash
# 本レポジトリをcloneする
cd ~/catkin_ws/src/
git clone https://github.com/TeamSOBITS/sobit_common

# インストーラを実行する
cd sobit_common
bash install.sh
```

2. その次、`catkin_make`でコンパイルします。

```
cd ~/catkin_ws/
catkin_make
```

## install.shの内容
```bash
# 必要なパッケージをダウンロードする
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-geometry*\
    ros-${ROS_DISTRO}-image-geometry

# Install DynamixelSDK
git clone -b noetic-devel https://github.com/TeamSOBITS/DynamixelSDK

# Install TurtleBot2
git clone https://github.com/TeamSOBITS/turtlebot2_on_noetic
cd turtlebot2_on_noetic
bash install.sh
```