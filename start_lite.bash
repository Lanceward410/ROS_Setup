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
    sudo apt install -y ros-$ROS_DISTRO-desktop-full
echo "source /opt/ros/$ROS_DISTRO/setup.bash"
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

# Important ROS Packages
install_additional_packages() {
echo "About to run install_additional_packages()"
    sudo apt install -y ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-rqt-robot-steering ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-roslint ros-$ROS_DISTRO-joy
    sudo apt-get install python3-catkin-tools -y
echo "Upgrading..."
    sudo apt update
    sudo apt upgrade -y
}

# Install AgileX Robots
install_ugv() {
echo "About to run install_ugv()"
#echo "ugv_ws already exists, let me remove it for you"
#sleep 1
#        sudo rm -rf ugv_ws
echo "Creating new ugv_ws"
    mkdir ugv_ws
    cd ugv_ws
    mkdir src
    cd src
echo "catkin_init_workspace"
    /opt/ros/$ROS_DISTRO/bin/catkin_init_workspace
echo "clone UMES_Limo"
    git clone https://github.com/Lanceward410/UMES_Limo.git
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
    echo "source devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

# A great repository of Gazebo models
get_models() {
    cd ~/ROS_Setup # To protect from any failure in the lines following
    echo "About to download Gazebo models"
    cd /usr/share/$GAZEBO_VERSION/models && sudo rm -r *
    sudo git init
    sudo git remote add origin https://github.com/osrf/gazebo_models.git
    sudo git pull origin master
    cd ~
}

# Head function 1.) ROS Melodic distrogit
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

# Automatic update from Github
cd ~/ROS_Setup
git pull

# A good waste of 10 minutes is what not including this was
cd ~

sudo apt update
sudo apt upgrade -y
sudo apt install git -y
sudo apt install git-lfs
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
# Acquire and install Docker
sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common -y
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io -y

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
get_models

echo "."
echo "."
echo "UMES ROS Workstation Setup Complete!"
echo "."
echo "."
