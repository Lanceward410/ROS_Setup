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