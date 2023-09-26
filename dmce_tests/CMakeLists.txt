cmake_minimum_required(VERSION 3.0.2)
project(dmce_tests)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(ompl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dmce_core
  dmce_mcplanner
  dmce_sim
  dmce_nodes
  dmce_msgs
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
  CATKIN_DEPENDS
  dmce_core
  dmce_mcplanner
  dmce_sim
  dmce_nodes
  dmce_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

#############
## Testing ##
#############

# Add gtest unit tests as a single target.
# test_name is used to generate the target name.
# Subsequent arguments are taken as source files containing gtest unit tests.
# Note that these are linked against test/unit/main.cpp, so they must not contain
# a main function.
# The source files are expected to be at test/unit/${file}.cpp
function (dmce_add_unit_tests test_name)
  set(source_files "")
  foreach(file ${ARGN})
    set(source_files ${source_files} test/unit/${file}.cpp)
  endforeach()
  catkin_add_gtest(${PROJECT_NAME}_${test_name} test/unit/main.cpp ${source_files})
  if(TARGET ${PROJECT_NAME}_${test_name})
    target_link_libraries(
      ${PROJECT_NAME}_${test_name} ${catkin_LIBRARIES}
    )
  endif()
endfunction()


# Add node-level tests to be run with rostest.
# launch_file is the name of the launch file to be used for the rostest.
# Subsequent arguments are built as self-contained gtest files and added
#  as dependencies of the rostest.
# For each such argument, a cpp file is expected at test/node/${file}.cpp
function (dmce_add_ntest launch_file)
  find_package(rostest REQUIRED)
  set(all_gtest_targets "")

  # Iterate over additional arguments and build them as gtests
  foreach(file ${ARGN})
    set(gtest_target ${PROJECT_NAME}_${file})
    set(all_gtest_targets ${all_gtest_targets} ${gtest_target})

    add_executable(${gtest_target} EXCLUDE_FROM_ALL test/node/${file}.cpp)
    target_link_libraries(
      ${gtest_target} ${GTEST_LIBRARIES} ${catkin_LIBRARIES}
    )
    if(TARGET tests)
      add_dependencies(tests ${gtest_target})
    endif()
  endforeach()

  # Add a single rostest dependent on each gtest
  add_rostest(${launch_file} DEPENDENCIES ${all_gtest_targets})
endfunction()




## Register unit tests
if (CATKIN_ENABLE_TESTING)
  dmce_add_unit_tests(unitTests
    TestOccupancyMap
    TestRobotMap
    TestSensorEmulator
    TestGroundTruthMap
    TestPathPlanner
    TestCollisionHandler
    TestPlanner
    TestDisplacementAction
    TestMCTreeNode
    TestMCActionGenerator
    TestMCTree
    TestFiniteLog
    TestRobotConnectivity
    TestFrontierClustering
    TestFrontierClusterAction
    TestNavigation
  )
endif()

## Register integration tests
if (CATKIN_ENABLE_TESTING)
  dmce_add_ntest(launch/node.test
    TestGlobalMapServer
    TestRobotMapServer
    TestGroundTruthMapServer
    TestSensorEmulatorServer
    TestPlannerServer
    TestPathfindingServer
    TestOmplPathfinding
    TestNavigationServer
  )

  add_rostest(launch/integration.test DEPENDENCIES dmce_nodes_nodeMain)
endif()

