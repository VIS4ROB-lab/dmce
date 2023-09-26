# Makefile for building and launching docker images

### Variables

# Arguments to be passed to roslaunch
ARGS ?=

# Set to '' to prevent interactive shell
DOCKER_IT ?= -it

# Catkin workspace on the local machine where build artifacts will be stored
CATKIN_WS ?= "${HOME}/catkin_ws"

# Location of the git repository on the local machine (mounted for source)
GIT ?= "${HOME}/ros_repos/"

#DOCKER_IMAGE = osrf/ros:noetic-desktop-full
DOCKER_IMAGE ?= grid_map

DOCKER_FILE ?= "${GIT}/dmce/docker/${DOCKER_IMAGE}.Dockerfile"

DOCKER_VOLUMES = \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="${GIT}":"/catkin_ws/src/ros_repos":rw \
	--volume="${CATKIN_WS}/build":"/catkin_ws/build":rw \
	--volume="${CATKIN_WS}/devel":"/catkin_ws/devel":rw \
	--volume="${CATKIN_WS}/logs":"/catkin_ws/logs":rw

DOCKER_ENV = \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1"

DOCKER_ARGS = ${DOCKER_IT} --net=host ${DOCKER_VOLUMES} ${DOCKER_ENV}

DOCKER_RUN = docker run ${DOCKER_ARGS} ${DOCKER_IMAGE}

CATKIN_MAKE = catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1


### Builders

.PHONY: docker-build
docker-build:
	docker build -f ${DOCKER_FILE} -t ${DOCKER_IMAGE} .

.PHONY: build-pkg
build-pkg:
	${DOCKER_RUN} ${CATKIN_MAKE} mre_dmcts

.PHONY: build-pkg-local
build-pkg-local:
	${CATKIN_MAKE} all


### Launchers

.PHONY: bash
bash: xhost-open
	${DOCKER_RUN} bash

# Run simulation with RViz display
.PHONY: demo
demo: build-pkg # xhost-open
	# ${DOCKER_RUN} roslaunch mre_dmcts_sim demo.launch
	roslaunch mre_dmcts_sim demo.launch ${ARGS}

# Run the headless simulation
.PHONY: sim
sim: build-pkg
	${DOCKER_RUN} roslaunch mre_dmcts_sim sim.launch ${ARGS}

.PHONY: rviz
rviz: xhost-open
	${DOCKER_RUN} rosrun rviz rviz

.PHONY: roscore
roscore:
	${DOCKER_RUN} roscore

.PHONY: test
test:
	${DOCKER_RUN} ${CATKIN_MAKE} run_tests_mre_dmcts_tests

#${DOCKER_RUN} catkin_test_results

.PHONY: utest
utest: # gTest-based unit tests
	${DOCKER_RUN} ${CATKIN_MAKE} run_tests_mre_dmcts_tests_gtest

.PHONY: ntest
ntest: # rostest-based node-level tests
	${DOCKER_RUN} ${CATKIN_MAKE} run_tests_mre_dmcts_tests_rostest_launch_node.test

.PHONY: itest
itest: # rostest-based integration tests
	${DOCKER_RUN} ${CATKIN_MAKE} run_tests_mre_dmcts_tests_rostest_launch_integration.test


### Other

.PHONY: perms-logs
perms-logs:
	${DOCKER_RUN} chmod -R 777 src/ros_repos/dmce/mre_dmcts_sim/logs/

.PHONY: xhost-open
xhost-open:
	xhost +

.PHONY: xhost-close
xhost-close:
	xhost -

.PHONY: clean
clean:
	sudo rm -rf ${CATKIN_WS}/build
	sudo rm -rf ${CATKIN_WS}/devel
	sudo rm -rf ${CATKIN_WS}/logs

.PHONY: logclean
logclean: perms-logs
	rm -rf mre_dmcts_sim/logs/*

