cmake_minimum_required(VERSION 3.0.2)
project(dmce_mcplanner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(required_packages
  grid_map_core
  grid_map_ros
  grid_map_cv
  # grid_map_filters
  # grid_map_loader
  # grid_map_msgs
  # grid_map_octomap
  # grid_map_rviz_plugin
  # grid_map_visualization

  dmce_msgs
  dmce_core
)
find_package(catkin REQUIRED COMPONENTS
  ${required_packages}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${required_packages}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/MCAction.cpp
  src/DisplacementAction.cpp
  src/FrontierClusterAction.cpp
  src/MCActionGenerator.cpp
  src/MCTree.cpp
  src/MCTreeNode.cpp
  src/MCTSPlanner.cpp
  src/DMCTSPlanner.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

