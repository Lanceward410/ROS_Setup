#!/bin/bash

# Welcome to ROS Setup!
# Run this file by navigating to the ROS_Setup repository
# and running "sudo bash setup_ros.bash"
# Cheers!
# Author: Lance Ward

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Creating Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Detect Ubuntu version
ubuntu_version=$(lsb_release -rs)
flag=""

# Sets GAZEBO_VERSION to version supported by Ubuntu distro in use
# Sets GAZEBO_VERSION and ROS_DISTRO based on Ubuntu version and flag
set_gazebo_and_ros_versions() {
    local ubuntu_version=$1
    local flag=$2

    case $flag in
        -1)
            case $ubuntu_version in
                18.04)
                    ROS_DISTRO="melodic"
                    GAZEBO_VERSION="gazebo-9"
                    ;;
                20.04)
                    ROS_DISTRO="noetic"
                    GAZEBO_VERSION="gazebo-11"
                    ;;
                14.04|16.04)
                    echo "Unsupported Ubuntu version for ROS 1."
                    GAZEBO_VERSION="gazebo-9"
                    exit 1
                    ;;
                22.04|23.04)
                    echo "Unsupported Ubuntu version for ROS 1."
                    GAZEBO_VERSION="gazebo-11"
                    exit 1
                *)
                    echo "Unsupported Ubuntu version for our projects."
                    exit 1
                    ;;
            esac
            ;;
        -2)
            case $ubuntu_version in
                14.04|16.04|18.04)
                    echo "Unsupported Ubuntu version for ROS 2."
                    GAZEBO_VERSION="gazebo-9"
                    exit 1
                    ;;
                20.04)
                    ROS_DISTRO="foxy"
                    GAZEBO_VERSION="gazebo-11"
                    ;;
                22.04)
                    ROS_DISTRO="humble"
                    GAZEBO_VERSION="gazebo-11"
                    ;;
                23.04)
                    echo "Unsupported Ubuntu version for ROS 1."
                    GAZEBO_VERSION="gazebo-11"
                    exit 1
                    ;;
                *)
                    echo "Unsupported Ubuntu version for our projects."
                    exit 1
                    ;;
            esac
            ;;
        *)
            echo "Invalid flag."
            exit 1
            ;;
    esac
}

# ~~~~~~~~~~~~~~~~~~~~~ Functions ~~~~~~~~~~~~~~~~~~~~~

    case $flag in
        -1)
            # Generic ROS1 Installer
            install_ros() {
            echo "setting keys for ROS"
                sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
                sudo apt install curl -y
            echo "Updating/Upgrading with new keys"    
                curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
                sudo apt update
                sudo apt upgrade -y
            echo "Installing ROS $ROS_DISTRO Full"
                sudo apt install -y ros-$ROS_DISTRO-desktop-full
            echo "source /opt/ros/$ROS_DISTRO/setup.bash"
                source /opt/ros/$ROS_DISTRO/setup.bash
                echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
                source ~/.bashrc
            }
            # Important ROS1 Packages
            install_additional_packages() {
            echo "About to run install_additional_packages()"
                sudo apt install -y ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-rqt-robot-steering ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-roslint ros-$ROS_DISTRO-joy
                sudo apt-get install python3-catkin-tools -y
            echo "Upgrading..."
                sudo apt update
                sudo apt upgrade -y
            }
            ;;
        -2)
            # Generic ROS2 Installer
            install_ros() {
                echo "Enabling Universe repository..."
                    sudo apt install software-properties-common
                    sudo add-apt-repository universe
                echo "Setting up keys..."
                    sudo apt install curl -y
                    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
                    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main"
                    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
                echo "Updating/Upgrading with new keys"
                    sudo apt update
                    sudo apt upgrade -y
                echo "Installing ROS $ROS_DISTRO..."
                    sudo apt install ros-$ROS_DISTRO-desktop -y
                    sudo apt install ros-dev-tools -y
                echo "source /opt/ros/$ROS_DISTRO/setup.bash"
                    source /opt/ros/$ROS_DISTRO/setup.bash
                    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
                    source ~/.bashrc
                case $ROS_DISTRO in
                    foxy)
                    sudo apt install python3-argcomplete -y
                        ;;
                    *)
                        ;;
                esac
                ;;
            }
            # Important ROS2 Packages
            # There are none at this time, but the framework to add packages
            # Is provided to add custom packages in the future.
            install_additional_packages() {
                case $ROS_DISTRO in 
                    foxy)
                        echo "Foxy does not use install_additional_packages() yet."
                        ;;
                    humble)
                        echo "Humble does not use install_additional_packages() yet."
                        ;;
                    *)
                        echo "What are thooooose!"
                        ;;
                esac
                ;;
            }
        *)
            echo "Problem while executing generic installer."
            ;;
    esac

# Install AgileX Robots
install_ugv() {
echo "About to run install_ugv()"
# Check if an SSH key already exists
if [ -f ~/ugv_ws ]; then
  echo "Workspace named ugv_ws already exists,"
  read -p "Would you like to factory reset it? (y/n): " choice
  if [[ "$choice" = "y" ]]; then
    echo "Removing ugv_ws and attempting to rebuild..."
    cd ~/ugv_ws && sudo rm -rf *
    cd ~ && sudo rmdir ~/ugv_ws
    cd ~
  fi
  return 2
fi
echo "Creating new ugv_ws"
    mkdir ugv_ws
    cd ugv_ws
    mkdir src
    cd src
echo "catkin_init_workspace"
    /opt/ros/$ROS_DISTRO/bin/catkin_init_workspace
echo "clone UMES_Limo"
    git clone github.com/Lanceward410/UMES_Limo.git
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
# A repository of custom models that I created and will make available via GitHub once I figure out how to do it for free
get_space_models() {
    echo "get_space_models() has been executed, but nothing happened."
}
# A repository of custom models that I created and will make available via GitHub once I figure out how to do it for free
get_moon_models() {
    echo "get_moon_models() has been executed, but nothing happened."
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

# Head function 2.) ROS Noetic distro
install_ros2_foxy() {
    export ROS_DISTRO=foxy
    install_ros
echo "About to install various python packages"
    sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install -y python3-rosdep
    sudo rosdep init
    rosdep update
    install_additional_packages
    install_ugv
}

# Head function 2.) ROS Noetic distro
install_ros2_humble() {
    export ROS_DISTRO=humble
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

# Check if a flag was provided
if [[ $# -eq 0 ]]; then
    echo "No flag provided. Please specify either -1 for ROS 1 or -2 for ROS 2." >&2
    echo "example:   linuxuser:~$   bash start.bash -1"
    exit 1
fi

sudo apt update
sudo apt upgrade -y
# Ensuring correct permissions to execute other_scripts from ROS_Setup
chmod +x ~/ROS_Setup/other_scripts/nvidia_setup.bash
chmod +x ~/ROS_Setup/other_scripts/sound.bash
chmod +x ~/ROS_Setup/other_scripts/git2.bash
chmod +x ~/ROS_Setup/other_scripts/git.bash
# Setting up Nvidia-Prime with NO flags (-n for nvidia, -i for intel, -o for on-demand)
bash ~/ROS_Setup/other_scripts/nvidia_setup.bash
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
sudo apt-get install docker-ce docker-ce-cli containerd.io -

# Call the function to set GAZEBO_VERSION and ROS_DISTRO
# If start.bash is called with flag -1, then ROS1; -2, then ROS2
set_gazebo_and_ros_versions "$ubuntu_version" "$flag"

# Call ROS install function based on Ubuntu version
case $ubuntu_version in
    18.04)
        install_ros_melodic
        ;;
    20.04)
        case $flag in
            -1)
            install_ros_noetic
            ;;
            -2)
            install_ros2_foxy
            ;;
            *)
            ;;
        esac
        ;;
    22.04)
        install_ros2_humble
	;;
esac

# Get Gazebo models
get_models
get_space_models
get_moon_models

# In the future, various aliases will be determined and set here
# Example:$ echo "alias myalias='/path/to/your/function.sh'" >> ~/.bashrc
echo "alias soundbash='~/ROS_Setup/other_scripts/sound.bash'" >> ~/.bashrc
echo "alias gitbash='~/ROS_Setup/other_scripts/git2.bash'" >> ~/.bashrc
echo "alias nvidiabash='~/ROS_Setup/other_scripts/nvidia_setup.bash'" >> ~/.bashrc

# Closing statement lets user know without confusion that the shell script has completed. 
echo "."
echo "."
echo "UMES ROS Workstation Setup Complete!"
echo "Please report bugs to Lward@umes.edu"
echo "."
echo "."
