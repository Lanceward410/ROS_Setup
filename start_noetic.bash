#!/bin/bash


# Welcome to ROS Setup!
# Run this file by navigating to the ROS_Setup repository
# and running "sudo bash setup_ros.bash"
# Cheers!
# Author: Lance Ward


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Creating Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Detect Ubuntu version

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

cd ~
sudo apt update
sudo apt upgrade -y
snap install code --classic
sudo apt install htop
sudo apt install git


username="$(whoami)"
#echo "chmod +x ~/ROS_Setup/setup_ros.bash"
#chmod +x ~/ROS_Setup/setup_ros.bash

echo "Running setup_ros as $username"
sudo -u $username bash ~/ROS_Setup/merge.bash