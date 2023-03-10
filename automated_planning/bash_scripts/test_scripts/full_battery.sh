#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash

ros2 topic pub -t 3 /anafi/battery std_msgs/msg/UInt8 '{data: "100"}' # Obs! This will cause the battery-message to be read as the letter 'd', since 100 = 0x64 = d
