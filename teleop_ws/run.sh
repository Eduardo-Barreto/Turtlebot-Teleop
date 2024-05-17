#!/bin/bash

source /opt/ros/humble/setup.sh

colcon build

source install/setup.bash

ros2 run turtlebot_teleop teleop_node
