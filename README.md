# Differential-Drive Robot: Wheel Odometry + IMU Sensor Fusion (EKF)

ROS 2 Humble + Gazebo Classic project fusing wheel odometry and IMU data
with an Extended Kalman Filter (robot_localization), evaluated against
ground truth. Work in progress.

## Build
    cd ~/ekf_ws
    colcon build --symlink-install
    source install/setup.bash

## Run the simulation
    ros2 launch diffbot_ekf sim.launch.py
