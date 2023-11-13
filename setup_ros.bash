#!/bin/bash

cd ~ # Because I'll never make this mistake again

# Welcome to ROS Setup!
# Run this file by navigating to the ROS_Setup repository
# and running "sudo bash setup_ros.bash"
# Cheers!
# Author: Lance Ward

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Creating Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Start Software Packages ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
echo "Update + Upgrade!"
# Update
sudo apt update -y
sudo apt upgrade -y

# Visual Studio Code
sudo snap install code --classic
# System Resource Monitor
sudo apt install htop
# Git!
sudo apt install git
# Python 3
sudo apt install python python3 -y
# Autoremove
# echo "sudo apt autoremove -y"
# sudo apt autoremove -y
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Misc Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Additional ROS packages install function
install_additional_packages() {
echo "About to run install_additional_packages()"
    sudo apt install -y ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-rqt-robot-steering ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-joy
    sudo apt-get install ros-$ROS_DISTRO-lms1xx
}

# Function to add sources
sourcing() {
echo "sourcing():"
    cd ~/ugv_ws
    source /opt/ros/$ROS_DISTRO/setup.bash
    source devel/setup.bash
    cd ~
}
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Start Debug install_ugv() ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ugv_ws install/make function; Version Ubuntu 18.04
install_ugv() {
echo "install_ugv():"
    mkdir ugv_ws
    cd ugv_ws
    mkdir src
    cd src
echo "catkin_init_workspace"
    /opt/ros/$ROS_DISTRO/bin/catkin_init_workspace
echo "clone ugv_sim"
    git clone https://github.com/agilexrobotics/ugv_sim.git
    cd ..
echo "Rosdep installing"
    rosdep install --from-paths src --ignore-src -r -y
echo "Running catkin_make"
    /opt/ros/$ROS_DISTRO/bin/catkin_make
}
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~ Install ROS functions, for Melodic, Noetic, and Humble ~~~~~~~~~~~~~~~~
# ROS1 Melodic install function
install_ros_melodic() {
    export ROS_DISTRO=melodic
echo "setting keys for ROS"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F42ED6FBAB17C654
    sudo apt update -y
echo "Installing ROS Melodic Full"
    sudo apt install -y ros-$DISTRO-desktop-full
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    source ~/.bashrc
echo "About to install various python packages"
    sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo apt install python-rosdep
    sudo rosdep init
    rosdep update
    install_additional_packages
echo "About to install python3-catkin-tools"
    sudo apt-get install python-catkin-tools python3-catkin-tools -y
    install_ugv
    sourcing
}

# ROS1 Noetic install function
install_ros_noetic() {
    export ROS_DISTRO=noetic
echo "setting keys for ROS"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update -y
echo "Installing ROS Noetic Full"
    sudo apt install -y ros-noetic-desktop-full
    source /opt/ros/noetic/setup.bash
echo "About to install various python packages"
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update
    install_additional_packages
echo "About to install python3-catkin-tools"
    sudo apt-get install python3-catkin-tools -y
    install_ugv
    sourcing
}

# ROS2 Humble install function
install_ros2_humble() {
    export ROS_DISTRO=humble
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros2.asc | sudo apt-key add -
    sudo apt update -y
echo "About to install ROS2 Humble..."
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
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Have to make sure we get back to the root directory
# Then we navigate to our Gazebo models folder
# Erase current contents, then add all gazebo models from OSRF
# Debug Gazebo models download
echo "About to download gazebo models to $GAZEBO_VERSION/models directory"
    cd /
    cd usr/share/$GAZEBO_VERSION/models
    sudo rm -r *
    sudo git init
    sudo git remote add origin https://github.com/osrf/gazebo_models.git
    sudo git pull origin master
# Back to Home
    cd ~

# Why not
echo "update + upgrade..."

    sudo apt update -y
    sudo apt upgrade -y

echo "redirecting to ugv_ws and sourcing devel/setup.bash"
    cd ~/ugv_ws
    source devel/setup.bash

echo "UMES ROS Workstation Setup Complete!"
echo "."
echo "."