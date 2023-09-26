cmake_minimum_required(VERSION 3.0.2)
project(dmce_nodes)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_compile_options(-Wno-format-security)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ompl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  costmap_2d
  smb_ompl_planner

  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization

  dmce_core
  dmce_mcplanner
  dmce_sim
  dmce_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


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
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
  roscpp
  std_msgs
  rosgraph_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  message_runtime
  costmap_2d
  smb_ompl_planner

  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization

  dmce_core
  dmce_mcplanner
  dmce_sim
  dmce_msgs
#  DEPENDS system_lib
)

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

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
add_library(${PROJECT_NAME}_lib
  src/GroundTruthMapServer.cpp
  src/SensorEmulatorServer.cpp
  src/NavigationServer.cpp
  src/RobotMapServer.cpp
  src/GlobalMapServer.cpp
  src/PlannerServer.cpp
  src/LoggingServer.cpp
  src/NodeServer.cpp
  src/TransformServer.cpp
  src/RobotMarkerServer.cpp
  src/ConnectivityServer.cpp
  src/FrontierClusterServer.cpp
  src/PathfindingServer.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_nodeMain src/nodeMain.cpp)
add_executable(${PROJECT_NAME}_clock src/clock.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_nodeMain PROPERTIES OUTPUT_NAME nodeMain PREFIX "")
set_target_properties(${PROJECT_NAME}_clock PROPERTIES OUTPUT_NAME clock PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_nodeMain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_clock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_nodeMain ${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_clock ${PROJECT_NAME}_lib ${catkin_LIBRARIES})


add_custom_target(${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_clock
  ${PROJECT_NAME}_nodeMain
)

