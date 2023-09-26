#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash";
if [ -f "/catkin_ws/devel/setup.bash" ]; then
	source "/catkin_ws/devel/setup.bash"
fi
exec "$@"
