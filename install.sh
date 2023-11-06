#!/bin/bash

# Select mobile base version
MOBILE_BASE_LIST=("kobuki" "custom")
MOBILE_BASE=$1
MOBILE_BASE=${MOBILE_BASE//-}


# Check if the mobile base is selected
if [[ "${MOBILE_BASE_LIST[*]}" =~ (^|[[:space:]])"${MOBILE_BASE}"($|[[:space:]]) ]]; then
    echo "╔══╣ Install: SOBITS Common with $MOBILE_BASE mobile base (STARTING) ╠══╗"

else
    echo "Please select one of the following mobile bases:"
    echo ${MOBILE_BASE_LIST[*]}
    echo "For example: bash install.sh --kobuki"
    exit
fi


# Download and install dependencies
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-joint-limits-interface

# Install DynamixelSDK
cd ../
sudo rm -r DynamixelSDK
git clone -b noetic-devel https://github.com/TeamSOBITS/DynamixelSDK

if [[ "${MOBILE_BASE}" == "kobuki" ]]; then
    # Install TurtleBot2
    sudo rm -r turtlebot2_on_noetic
    git clone https://github.com/TeamSOBITS/turtlebot2_on_noetic
    cd turtlebot2_on_noetic
    bash install.sh
    cd ../

    # Link Kobuki library with gazebo (not necessary?)
    # sudo cp ~/catkin_ws/src/sobit_common/turtlebot2_on_noetic/turtlebot_simulator/turtlebot_gazebo/libgazebo_ros_kobuki.so /opt/ros/${ROS_DISTRO}/lib
fi

cd sobits_common/


echo "╚══╣ Install: SOBITS Common (FINISHED) ╠══╝"
