#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash

ros2 topic pub -t 3 /anafi/battery std_msgs/msg/Float64 '{data: "34"}' 
