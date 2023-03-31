#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/ubuntu/P10/install/setup.bash

export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=rplidar
export ROBOT_NAMESPACE=polybot0x
export ROS_DOMAIN_ID=0

ros2 launch linorobot2_bringup namespace_bringup.launch.py
