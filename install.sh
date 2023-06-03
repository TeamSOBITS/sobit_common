#!/bin/bash


echo "╔══╣ Install: SOBITS Common (STARTING) ╠══╗"

# 必要なパッケージをダウンロードする
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-geometry* \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-smach*

# Install DynamixelSDK
git clone -b noetic-devel https://github.com/TeamSOBITS/DynamixelSDK

# Install TurtleBot2
git clone https://github.com/TeamSOBITS/turtlebot2_on_noetic
cd turtlebot2_on_noetic
bash install.sh

# Link Kobuki library with gazebo (not necessary?)
# sudo cp ~/catkin_ws/src/sobit_common/turtlebot2_on_noetic/turtlebot_simulator/turtlebot_gazebo/libgazebo_ros_kobuki.so /opt/ros/${ROS_DISTRO}/lib


echo "╚══╣ Install: SOBITS Common (FINISHED) ╠══╝"
