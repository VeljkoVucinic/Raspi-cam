#!/bin/bash
# This is an entrypoint for the Docker container.
# DO NOT RUN MANUALLY!

rosdep update
apt update
apt install -y ros-foxy-rmw-cyclonedds-cpp libserial-dev

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ROS2-Environment laden
. /opt/ros/$ROS_DISTRO/setup.bash

echo "Executing first-run routine ..."
# Hier können Befehle eingefügt werden, welche den Container einrichten und nur einmal ausgeführt werden müssen
# Beispielsweise: Pakete installieren, kompilieren, Zusatzinhalte runterladen, ...

echo "Building node ..."
# NemaTode bauen
cd /root/ros2_workspace/src/$PKG_NAME/NemaTode

cmake .
make install

# Node-Environment laden
cd /root/ros2_workspace/src/$PKG_NAME/nodetemplate

# Node kompilieren
colcon build
