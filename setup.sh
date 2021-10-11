#!/bin/bash

sudo apt update 
sudo apt install ros-melodic-kobuki-* -y
sudo apt install ros-melodic-ecl-streams -y
sudo apt install ros-melodic-joy -y
sudo apt install ros-melodic-joint-state-publisher* -y
sudo cp ~/catkin_ws/src/sobit_common/turtlebot2/turtlebot_simulator/turtlebot_gazebo/libgazebo_ros_kobuki.so /opt/ros/melodic/lib
# 関係ない
sudo apt install ros-melodic-pcl-*
sudo apt install ros-melodic-openni2-*
