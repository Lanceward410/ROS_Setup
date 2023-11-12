#!/bin/bash

cd ~ # Because I'll never make this mistake again

# Welcome to ROS Setup!
# Run this file by navigating to the ROS_Setup repository
# and running "sudo sh setup_ros.sh"
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
# Debug Update
echo "Prepare to Update + Upgrade!"
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing with update"
# Update
echo "sudo apt update -y"
sudo apt update -y
echo "sudo apt upgrade -y"
sudo apt upgrade -y

# Debug Software Packages
echo "Just finished # Update! About to install software packages..."
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing with the next commands, Visual Studio Code, etc"
# Visual Studio Code
sudo snap install code --classic
# System Resource Monitor
sudo apt install htop -y
# Git!
sudo apt install git -y
# Python 3 from popular deadsnakes repository
echo "repository ppa:deadsnakes/ppa"
sudo apt-repository ppa:deadsnakes/ppa
echo "python3"
sudo apt install python3 -y
# Autoremove
# echo "sudo apt autoremove -y"
# sudo apt autoremove -y
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Misc Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Additional ROS packages install function
# Debug Additional ROS Packages
install_additional_packages() {
echo "About to run install_additional_packages()..."
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "sudo apt install -y ros-$ROS_DISTRO-packages"
    sudo apt install -y ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-rqt-robot-steering ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-joint-state-publisher-gui rviz_visual_tools ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-joy
}

# Debug sourcing()
echo "About to call function sourcing()"
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing with sourcing()"
# Function to add sources
sourcing() {
    echo "cd ~/limo_ws"
    cd ~/limo_ws
    echo "source /opt/ros/melodic/setup.bash"
    source /opt/ros/melodic/setup.bash
    echo "source devel/setup.bash"
    source devel/setup.bash
    echo "cd ~"
    cd ~
}
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Start Debug install_limo() ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Limo_ws install/make function; Version Ubuntu 18.04
install_limo() {
echo "About to run install_limo()"
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing @ mkdir limo_ws"
    mkdir limo_ws
    echo "cd limo_ws"
    cd limo_ws

# Debug Sourcing
#   Source ROS environment in bashrc
    echo "source /opt/ros/melodic/setup.bash"
    source /opt/ros/melodic/setup.bash

# Debug catkin_init_workspace
    echo "About to run catkin_init_workspace"
    for i in {5..1}; do
        echo "Countdown: $i"
        sleep 1
    done
    echo "mkdir src"
    mkdir src
    echo "cd src"
    cd src
    echo "catkin_init_workspace"
    /opt/ros/melodic/bin/catkin_init_workspace
    echo "clone ugv_sim"
    git clone https://github.com/agilexrobotics/ugv_sim.git
    cd ..
    echo "rosdep installer"
    rosdep install --from-paths src --ignore-src -r -y

# Debug catkin_make
echo "About to run catkin_make"
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing"
#    echo "cd .."
#    cd ..
    /opt/ros/melodic/bin/catkin_make
}
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~ Install ROS functions, for Melodic, Noetic, and Humble ~~~~~~~~~~~~~~~~
# ROS1 Melodic install function
install_ros_melodic() {
    export ROS_DISTRO=melodic
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update -y

# Debug Install Melodic
echo "About to install ROS Melodic Full"
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Installing ROS Melodic Full"
    sudo apt install -y ros-melodic-desktop-full
    source /opt/ros/melodic/setup.bash
    echo 'export PATH=$PATH:/opt/ros/melodic/bin' >> ~/.bashrc


# Debug Python packages
echo "About to install various python packages..."
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing with python packages"
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

# Debug Catkin Tools
echo "About to install python3-catkin-tools"
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing with catkin-tools"
sudo apt-get install python3-catkin-tools -y
    install_limo
    sourcing
}

# ROS1 Noetic install function
install_ros_noetic() {
    export ROS_DISTRO=noetic
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install -y curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update

# Debug Install Noetic
echo "About to install ROS Noetic Full"
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Installing ROS Noetic Full"
    sudo apt install -y ros-noetic-desktop-full
    cd ~
    source /opt/ros/noetic/setup.bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

# Debug Python Packages
echo "About to install various python packages..."
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Installing python packages"
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

# Debug Catkin Tools
echo "About to install python3-catkin-tools..."
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing with catkin-tools"
sudo apt-get install python3-catkin-tools -y
    install_additional_packages
}

# ROS2 Humble install function
install_ros2_humble() {
    export ROS_DISTRO=humble
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros2.asc | sudo apt-key add -
    sudo apt update -y
# Debug
echo "About to install ROS2 Humble..."
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Installing ROS2 Humble"
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
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing"
cd /
cd usr/share/$GAZEBO_VERSION/models
sudo rm -r *
git init
git remote add origin https://github.com/osrf/gazebo_models.git
git pull origin master
# Back to Home
cd ~

# Why not
# Debug
echo "About to update + upgrade..."
for i in {5..1}; do
    echo "Countdown: $i"
    sleep 1
done
echo "Continuing"
sudo apt update -y
sudo apt upgrade -y

echo "UMES ROS Workstation Setup Complete"
