#!/bin/bash

sudo apt update -y
sudo apt upgrade -y
sudo snap install code --classic
sudo apt install htop
sudo apt install git
sudo apt-repository ppa:deadsnakes/ppa
sudo apt install python3

# Common function to install additional ROS packages
install_additional_packages() {
    sudo apt install -y gazebo_ros_pkgs ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-rqt-robot-steering ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-rqt-common-plugins rviz_visual_tools robot_localization ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-image-pipeline ros-$ROS_DISTRO-depthimage-to-laserscan ros-$ROS_DISTRO-octomap ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-joy
}
install_limo() {
    mkdir limo_ws
    cd limo_ws
    mkdir src
    cd src
    catkin_init_workspace
    git clone https://github.com/agilexrobotics/ugv_sim.git
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    source devel/setup.bash
}

# Function to install ROS Melodic for Ubuntu 18.04
install_ros_melodic() {
    export ROS_DISTRO=melodic
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-melodic-desktop-full -y
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo apt install python-rosdep
    sudo rosdep init
    rosdep update
    install_additional_packages
    install_limo
}

# Function to install ROS Noetic for Ubuntu 20.04
install_ros_noetic() {
    export ROS_DISTRO=noetic
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full -y
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update
    install_additional_packages
}

# Function to install ROS2 Humble Hawksbill for Ubuntu 22.04
install_ros2_humble() {
    export ROS_DISTRO=humble
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros2.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-humble-desktop -y
    install_additional_packages
}

# Detect Ubuntu version
ubuntu_version=$(lsb_release -rs)

# Install the appropriate ROS version
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
    *)
        echo "Unsupported Ubuntu version for this script. Exiting."
        exit 1
        ;;
esac

sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get install python3-catkin-tools


# Source ROS environment in bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "UMES ROS Workstation Setup Complete"
