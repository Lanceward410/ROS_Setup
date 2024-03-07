    cd ~/ROS_Setup
    git pull
    cd ~/ugv_ws
    gnome-terminal -- bash -c limostart.bash
    gnome-terminal -- bash -c limomap.bash
    gnome-terminal -- bash -c limocontrol.bash


    cd ~/ugv_ws
    source ~/.bashrc
    roslaunch limo_gazebo_sim limo_four_diff.launch

    cd ~/ugv_ws
    source ~/.bashrc
    rosparam set use_sim_time true
    rosrun gmapping slam_gmapping scan:=/limo/scan

    cd ~/ugv_ws
    source ~/.bashrc
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
