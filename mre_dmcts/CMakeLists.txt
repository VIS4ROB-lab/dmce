cmake_minimum_required(VERSION 3.0.2)
project(mre_dmcts)
find_package(catkin REQUIRED)
catkin_metapackage()

add_custom_target(mre_dmcts)
add_dependencies(mre_dmcts
	mre_dmcts_core
	mre_dmcts_mcplanner
	mre_dmcts_nodes
	mre_dmcts_sim
	rrt_node
	mmpf_node
)
