#!/bin/bash
# This is an entrypoint for the Docker container.
# DO NOT RUN MANUALLY!

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "Starting ROS2 node ..."
# ROS2- und Node-Environment laden
. /opt/ros/$ROS_DISTRO/setup.bash
. /root/ros2_workspace/src/$PKG_NAME/install/setup.bash

# Node starten
ros2 run $PKG_NAME $NODE_NAME
