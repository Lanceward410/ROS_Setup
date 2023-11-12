#!/bin/bash

cd ~ # -_-

# Welcome to ROS Setup!
# Run this file by navigating to the ROS_Setup repository
# and running "sudo sh setup_ros.sh"
# Cheers!
# Author: Lance Ward

# Detect Ubuntu version
ubuntu_version=$(lsb_release -rs)

# Sets variable $GAZEBO_VERSION based on Ubuntu version, Ubuntu 18.04 = Gazebo-9, Ubuntu 20.04 = Gazebo-11
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

set_gazebo_version

# Update
sudo apt update -y
sudo apt upgrade -y
# Visual Studio Code
echo "VS Code"
sudo snap install code --classic
# System Resource Monitor
echo "htop"
sudo apt install htop -y
# Git!
echo "Git"
sudo apt install git -y
# Python 3 from popular deadsnakes repository
echo "repository ppa:deadsnakes/ppa"
sudo apt-repository ppa:deadsnakes/ppa
echo "python3"
sudo apt install python3 -y
# Autoremove

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Misc Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Additional ROS packages install function
# Debug Additional ROS Packages
install_additional_packages() {
    echo "installing ros packages..."
    sudo apt install -y ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-rqt-robot-steering ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-joint-state-publisher-gui rviz_visual_tools ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-joy
}

# Function to add sources
sourcing() {
    cd ~/limo_ws
    source /opt/ros/melodic/setup.bash
    source devel/setup.bash
    cd ~
}

# Limo_ws install/make function; Version Ubuntu 18.04
install_limo() {
    mkdir limo_ws
    cd limo_ws
    mkdir src
    cd src
    /opt/ros/melodic/bin/catkin_init_workspace
    echo "clone ugv_sim:"
    git clone https://github.com/agilexrobotics/ugv_sim.git
    cd ~/limo_ws
    echo "rosdep installer:"
    rosdep install --from-paths src --ignore-src -r -y
    /opt/ros/melodic/bin/catkin_make
}

# ROS1 Melodic install function
install_ros_melodic() {
    echo "Begin Melodic Install"
    export ROS_DISTRO=melodic
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update -y
    sudo apt install -y ros-melodic-desktop-full
    source /opt/ros/melodic/setup.bash
    echo 'export PATH=$PATH:/opt/ros/melodic/bin' >> ~/.bashrc
    sudo apt install -y python-rosdep python-rosinstall python-roslaunch python-rosinstall-generator python-wstool build-essential
    sudo apt install python-rosdep
    sudo rosdep init
    rosdep update
    install_additional_packages
# Setting up Catkin Tools keys
    cd ~
    sudo sh \
        -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get install python3-catkin-tools -y
    echo "install_limo()"
    install_limo
    echo "sourcing()"
    sourcing
}

# ROS1 Noetic install function
install_ros_noetic() {
    export ROS_DISTRO=noetic
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install -y curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install -y ros-noetic-desktop-full
    cd ~
    source /opt/ros/noetic/setup.bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install -y python3-rosdep
    sudo rosdep init
    rosdep update

# Setting up Catkin Tools keys
cd ~
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get install python3-catkin-tools -y
    install_additional_packages
}

# ROS2 Humble install function
install_ros2_humble() {
    export ROS_DISTRO=humble
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros2.asc | sudo apt-key add -
    sudo apt update -y
    sudo apt install ros-humble-desktop -y
#    install_additional_packages
}

# Call ROS install function based on Ubuntu version
case $ubuntu_version in
    18.04)
        install_ros_melodic
        ;;
    20.04)
        install_ros_noetic
        ;;
    22.04)
        install_ros2_humble
	;;
esac

# Have to make sure we get back to the root directory
# Then we navigate to our Gazebo models folder
# Erase current contents, then add all gazebo models from OSRF
cd /
cd usr/share/$GAZEBO_VERSION/models
sudo rm -r *
git init
git remote add origin https://github.com/osrf/gazebo_models.git
git pull origin master
# Back to Home
cd ~

# Why not
sudo apt update -y
sudo apt upgrade -y

echo "UMES ROS Workstation Setup Complete"
