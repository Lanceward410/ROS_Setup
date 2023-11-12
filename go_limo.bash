    cd ~/limo_ws

    source /opt/ros/melodic/setup.bash
    source devel/setup.bash
    rosparam set use_sim_time true
    rosrun gmapping slam_gmapping scan:=/limo/scan

    source /opt/ros/melodic/setup.bash
    source devel/setup.bash
    rosparam set use_sim_time true
    roslaunch limo_gazebo_sim limo_ackerman.launch

    source /opt/ros/melodic/setup.bash
    source devel/setup.bash
    rosparam set use_sim_time true
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
