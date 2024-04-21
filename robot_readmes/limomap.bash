    cd ~/ugv_ws
    source ~/.bashrc
    rosparam set use_sim_time true
    rosrun gmapping slam_gmapping scan:=/limo/scan
