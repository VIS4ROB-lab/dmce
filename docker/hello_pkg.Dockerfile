FROM osrf/ros:noetic-desktop-full

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install Git
RUN apt-get update && apt-get install -y git

RUN source /opt/ros/noetic/setup.bash && \
	mkdir -p /catkin_ws/src && \
	cd /catkin_ws/src && \
	catkin_init_workspace && \
	cd .. && \
	catkin_make && \
	echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /catkin_ws
