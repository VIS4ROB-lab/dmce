cmake_minimum_required(VERSION 3.0.2)
project(dmce)
find_package(catkin REQUIRED)
catkin_metapackage()

add_custom_target(dmce)
add_dependencies(dmce
	dmce_core
	dmce_mcplanner
	dmce_nodes
	dmce_sim
	rrt_node
	mmpf_node
)
