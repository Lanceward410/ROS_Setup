#!/bin/bash

ubuntu_version=$(lsb_release -rs)

cd ~

sudo apt update
sudo apt upgrade -y
snap install code --classic
sudo apt install htop
sudo apt install git

install_additional_packages() {
echo "About to run install_additional_packages()"
    sudo apt install -y ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-rqt-robot-steering ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-joy
}

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

install_ros_noetic() {
    export ROS_DISTRO=noetic
echo "setting keys for ROS"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
echo "Installing ROS $ROS_DISTRO Full"
    sudo apt install -y ros-noetic-desktop-full
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
echo "About to install various python packages"
    sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install -y python3-rosdep
    sudo rosdep init
    rosdep update
    install_additional_packages
echo "About to install python3-catkin-tools"
    sudo apt-get install python3-catkin-tools -y
    install_ugv
    #sourcing
}
install_ros_noetic