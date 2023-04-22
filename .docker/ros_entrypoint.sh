#!/bin/bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash
echo "Sourced ROS 2 Humble"

# Source the base workspace, if built
if [ -f /root/c3pzero_ws/install/setup.bash ]
then
  source /root/c3pzero_ws/install/setup.bash
  echo "Sourced the robots workspace"
else
  echo "Please build the robots workspace with:"
  echo "colcon build"
fi

# Execute the command passed into this entrypoint
exec "$@"
