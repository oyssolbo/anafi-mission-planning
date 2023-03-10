#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash

ros2 topic pub -t 3 /anafi/state std_msgs/msg/String "{data: 'FS_LANDED'}"