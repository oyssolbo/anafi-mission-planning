#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash

# Currently hardcoded in location for A0
ros2 topic pub --once /estimate/person_detected anafi_uav_interfaces/msg/DetectedPerson '{id: 69, severity: 2, position: {x: 20.0, y: 0.0, z: 0.0}}'