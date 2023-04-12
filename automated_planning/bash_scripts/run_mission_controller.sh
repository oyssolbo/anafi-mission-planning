#!/bin/bash

source ~/colcon_ws/install/setup.bash

ros2 run automated_planning mission_controller_node --ros-args --params-file /home/killah/colcon_ws/install/automated_planning/share/automated_planning/config/mission_parameters.yaml --params-file /home/killah/colcon_ws/install/automated_planning/share/automated_planning/config/config.yaml