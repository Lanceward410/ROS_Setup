#!/bin/bash

# Change to the ROS_Setup directory
cd ~/ROS_Setup

# Check if python3-tk is installed
if ! dpkg-query -W -f='${Status}' python3-tk 2>/dev/null | grep -q "ok installed"; then
    echo "python3-tk is not installed. Installing now..."
    sudo apt-get install python3-tk -y
else
    echo "Python3 found! Opening ROS Setup GUI."
fi
# Run the GUI script
python3 ~/ROS_Setup/rosgui.py
