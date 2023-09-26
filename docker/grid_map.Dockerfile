FROM osrf/ros:noetic-desktop-full

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install dependencies
# python3wstool
RUN apt-get update && apt-get install -y git libeigen3-dev ros-noetic-navigation ros-noetic-octomap-msgs ros-noetic-grid-map ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins ros-noetic-ompl ros-noetic-move-base ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-costmap-2d ros-noetic-teb-local-planner ros-noetic-robot-self-filter ros-noetic-pointcloud-to-laserscan ros-noetic-ros-numpy ros-noetic-octomap-server ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-grid-map-costmap-2d ros-noetic-grid-map-ros ros-noetic-map-server ros-noetic-global-planner ros-noetic-grid-map-filters ros-noetic-grid-map-visualization

RUN source /opt/ros/noetic/setup.bash && \
	# mkdir -p gridmap_ws/src && \
	# cd gridmap_ws/src/ && \
	# git clone https://github.com/anybotics/grid_map.git && \
	# cd ../ && \
	# catkin_make -DCMAKE_BUILD_TYPE=Release install && \
	# echo "source /gridmap_ws/devel/setup.bash" >> ~/.bashrc && \
	cd / && \
	mkdir -p /catkin_ws/src && \
	cd /catkin_ws/src && \
	catkin_init_workspace && \
	cd .. && \
	catkin_make && \
	# git clone https://github.com/anybotics/grid_map.git && \
	echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
	# sudo apt-get install -y ros-noetic-grid-map

WORKDIR /catkin_ws

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# ENTRYPOINT /bin/bash -i
#CMD source /gridmap_ws/devel/setup.bash && echo "hi there"
# CMD echo "hi there"
