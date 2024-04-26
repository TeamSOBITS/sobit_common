#!/bin/bash

echo "╔══╣ Install: SOBITS Common with $MOBILE_BASE mobile base (STARTING) ╠══╗"


# Keep track of the current directory
CURRENT_DIR=`pwd`
cd ..

# Dowload required packages for SOBIT PRO
ros_packages=(
    "DynamixelSDK" \
    "sobits_msgs"
)

# Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}

# Go back to previous directory
cd ${CURRENT_DIR}

# Download ROS dependencies
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-limits-interface \
    ros-${ROS_DISTRO}-hardware-interface


echo "╚══╣ Install: SOBITS Common (FINISHED) ╠══╝"
