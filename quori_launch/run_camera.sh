#!/usr/bin/env bash

. /workspaces/isaac_ros-dev/install/setup.bash

ros2 launch quori_launch quori_launch.py camera_type:=$1


