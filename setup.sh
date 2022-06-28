#!/bin/bash


echo "╔══╣ Install: Sobit Common (STARTING) ╠══╗"


sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-kobuki-* \
    ros-${ROS_DISTRO}-ecl-streams \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joint-state-publisher*

sudo cp ~/catkin_ws/src/sobit_common/turtlebot2/turtlebot_simulator/turtlebot_gazebo/libgazebo_ros_kobuki.so /opt/ros/${ROS_DISTRO}/lib

# 関係ない
sudo apt-get install -y \
    ros-${ROS_DISTRO}-pcl-* \
    ros-${ROS_DISTRO}-openni2-*


echo "╚══╣ Install: Sobit Common (FINISHED) ╠══╝"
