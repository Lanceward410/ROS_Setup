#!/bin/bash

username="$(whoami)"
#echo "chmod +x ~/ROS_Setup/setup_ros.bash"
#chmod +x ~/ROS_Setup/setup_ros.bash

echo "Running setup_ros as $username"
sudo -u $username bash ~/ROS_Setup/setup_ros.bash
