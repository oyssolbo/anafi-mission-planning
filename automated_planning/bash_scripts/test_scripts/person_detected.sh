#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash

ros2 topic pub /estimate/person_detected anafi_uav_interfaces/msg/DetectedPerson "{}"