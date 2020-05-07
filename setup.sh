#!/bin/sh

sudo apt update 
sudo apt install ros-melodic-kobuki-* -y
sudo apt install ros-melodic-ecl-streams -y
sudo apt install ros-melodic-joy -y
sudo apt install ros-melodic-joint-state-publisher* -y
# 関係ない
sudo apt install ros-melodic-pcl-*
sudo apt install ros-melodic-openni2-*
