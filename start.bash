#!/bin/bash

# Welcome to ROS Setup!
# Run this file by navigating to the ROS_Setup repository
# and running "sudo bash setup_ros.bash"
# Cheers!
# Author: Lance Ward

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Creating Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Detect Ubuntu version
ubuntu_version=$(lsb_release -rs)

# Sets GAZEBO_VERSION to version supported by Ubuntu distro in use
set_gazebo_version() {
    case $ubuntu_version in
        18.04)
            GAZEBO_VERSION="gazebo-9"
            ;;
        20.04)
            GAZEBO_VERSION="gazebo-11"
            ;;
        *)
            echo "Unsupported Ubuntu version for Gazebo installation. Defaulting to no particular gazebo."
            GAZEBO_VERSION="gazebo"
            ;;
    esac
}

# ~~~~~~~~~~~~~~~~~~~~~ Functions ~~~~~~~~~~~~~~~~~~~~~

# Generic ROS Installer
install_ros() {
echo "setting keys for ROS"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
echo "Installing ROS $ROS_DISTRO Full"
    sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

# Important ROS Packages
install_additional_packages() {
echo "About to run install_additional_packages()"
    sudo apt install -y ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-rqt-robot-steering ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-roslint ros-$ROS_DISTRO-joy
    sudo apt-get install python-catkin-tools -y
echo "Upgrading..."
    sudo apt update
    sudo apt upgrade -y
}

# Install AgileX Robots
install_ugv() {
echo "About to run install_ugv()"
    if [ -d "ugv_ws" ]; then
echo "ugv_ws already exists, let me remove it for you"
sleep 1
        sudo rm -rf ugv_ws
    fi
echo "Creating new ugv_ws"
    mkdir ugv_ws
    cd ugv_ws
    mkdir src
    cd src
echo "catkin_init_workspace"
    /opt/ros/$ROS_DISTRO/bin/catkin_init_workspace
echo "clone ugv_sim"
    git clone https://github.com/agilexrobotics/ugv_sim.git
    cd ..
    LINE_TO_ADD="source devel/setup.bash"
    FILE=~/.bashrc
        if ! grep -Fxq "$LINE_TO_ADD" $FILE; then
            echo "$LINE_TO_ADD" >> $FILE
echo "source devel/setup.bash added to to $FILE."
        else
echo "source devel/setup.bash already exists in $FILE."
        fi
    source ~/.bashrc
echo "Rosdep installing"
    rosdep install --from-paths src --ignore-src -r -y
echo "Running catkin_make"
    /opt/ros/$ROS_DISTRO/bin/catkin_make
}

# A great repository of Gazebo models
get_models() {
    echo "About to download Gazebo models"
    cd /
    cd usr/share/$GAZEBO_VERSION/models
    sudo rm -r *
    sudo git init
    sudo git remote add origin https://github.com/osrf/gazebo_models.git
    sudo git pull origin master
    cd ~
}

# Head function 1.) ROS Melodic distro
install_ros_melodic() {
    export ROS_DISTRO=melodic
    install_ros
echo "About to install various python packages"
    sudo apt install -y python python3
    sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo apt install python-rosdep
    sudo rosdep init
    rosdep update
    install_additional_packages
    install_ugv
}

# Head function 2.) ROS Noetic distro
install_ros_noetic() {
    export ROS_DISTRO=noetic
    install_ros
echo "About to install various python packages"
    sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install -y python3-rosdep
    sudo rosdep init
    rosdep update
    install_additional_packages
    install_ugv
}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ The Script ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# A good waste of 10 minutes is what not including this was
cd ~

sudo apt update
sudo apt upgrade -y
sudo apt install git -y
# These are some optional software to aid in ROS development
snap install code --classic
sudo snap install foxglove-studio -y
sudo snap install qtcreator-ros --classic -y
sudo snap install cmake --classic -y
#imager for raspberry pi
sudo snap install rpi-imager -y
# Open-source 3D model editor
sudo apt install blender -y
# Turtle bot!
sudo snap install turtlebot3c -y
# System resource monitor
sudo apt install htop

# Set Gazebo Version...
set_gazebo_version

# Call ROS install function based on Ubuntu version
case $ubuntu_version in
    18.04)
        install_ros_melodic
        ;;
    20.04)
        install_ros_noetic
        ;;
    22.04)
        echo "ROS2 Humble install to Ubuntu 22.04 not yet supported."
        echo "Try again later."
        #install_ros2_humble
	;;
esac

# Get Gazebo models
#get_models

echo "."
echo "."
echo "UMES ROS Workstation Setup Complete!"
echo "."
echo "."
