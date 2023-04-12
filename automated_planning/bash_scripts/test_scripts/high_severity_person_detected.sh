#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash

# Currently hardcoded for detection at location A2
ros2 topic pub /estimate/person_detected anafi_uav_interfaces/msg/DetectedPerson '{id: 0, severity: 2, position: {x: 20.0, y: 20.0, z: 0.0}}'