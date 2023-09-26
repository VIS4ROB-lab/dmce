# DMCE: Decentralised Monte Carlo Exploration

This project is an implementation of the multi-robot planning strategy proposed in **"Decentralised Multi-Robot Exploration using Monte Carlo Tree Search"**, published at the IROS 2023 conference. It is implemented in C++ with the framework provided by ROS Noetic. See the [demo video](https://youtu.be/3RfDq8wlnPI).

<!--[![IMAGE ALT TEXT](http://img.youtube.com/vi/3RfDq8wlnPI/0.jpg)](http://www.youtube.com/watch?v=3RfDq8wlnPI "DMCE Video Demo")-->

  - [Publications](#publications)
  - [Quick start](#quick-start)
  - [Utility Scripts](#utility-scripts)
  - [Code Overview](#code-overview)
  - [Adding Scenarios and Maps](#adding-scenarios-and-maps)
  - [Questions and bug reports](#questions-and-bug-reports)

## Publications

If you use this work in an academic context, please cite the following publication:

> S. Bone, L. Bartolomei, F. Kennel-Maushart and M. Chli, **"Decentralised Multi-Robot Exploration using Monte Carlo Tree Search"**. In IEEE Int. Conf. on Intelligent Robots and Systems (IROS), 2023. DOI: [10.3929/ethz-b-000624905](https://doi.org/10.3929/ethz-b-000624905)

```
@inproceedings{bone_decentralised_2023,
	title = {Decentralised Multi-Robot Exploration using Monte Carlo Tree Search},
	url = {https://doi.org/10.3929/ethz-b-000624905},
	doi = {10.3929/ethz-b-000624905},
	eventtitle = {36th {IEEE}/{RSJ} International Conference on Intelligent Robots and Systems ({IROS} 2023)},
	author = {Bone, Sean and Bartolomei, Luca and Kennel-Maushart, Florian and Chli, Margarita},
	year = {2023},
}
```

## Quick Start

To compile and run the software, you can either build the Docker container, or install all the dependencies locally.

### Running with Docker

1. Install the docker engine as described in [this guide](https://docs.docker.com/engine/install/ubuntu/).
2. Recommended: follow the [post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/) to be able to run docker without `sudo`.
3. Create a folder for the catkin build artifacts and clone this repository:
```
mkdir ~/catkin_ws
mkdir ~/ros_repos
cd ~/ros_repos
git clone git@github.com:ETHZ-RobotX/smb_path_planner.git
git clone git@github.com:VIS4ROB-lab/dmce.git
cd dmce
```
4. If you used different locations for the Catkin workspace or the repository folder, make sure to adjust the `CATKIN_WS` and `GIT` variables in `Makefile`.
5. Run the tests with `make docker-build test`
6. Run the simulation in headless mode with `make sim`. If you want graphical output, use `make demo` instead.

#### GUI forwarding from within Docker

To run GUI applications such as Rviz from within the Docker container, `Makefile` sets up X11 port forwarding. This should work on Ubuntu 20.04 (Focal) without extra steps.

**Note on security:** this works by running `xhost +`, which allows clients to connect to the X server from any host. Malicious applications could draw on the screen or read inputs. Use with caution.

### Running locally

Alternatively, if you're running Ubuntu 20.04 (Focal), you can run everything locally.

1. [Install ROS Noetic](http://wiki.ros.org/noetic/Installation) on your system.
2. Read [docker/grid_map.Dockerfile](docker/grid_map.Dockerfile) and replicate the installation steps.


## Utility scripts

There are a number of utility scripts for automated execution of multiple runs as well as post-processing tasks.

### `scripts/run_batch.py`
This scripts performs the following steps locally:

1. Erase the current logs in `dmce_sim/logs/`
2. Build the package with `make build-pkg`
3. Run the demo (with RViz window) for the specified duration multiple times, for each specified configuration.

In order to customise the run configuration, edit the following variables in the script:

- `nruns`: number of runs for each config
- `run_duration`: duration in seconds (wall-clock, not simulation) of each run. Note that this includes startup time, so you may want to add a few seconds (e.g., to have a full 600s of simulation, set `run_duration` to 605s.) 
- `plannerTypes`: the planner types to be evaluated. Each planner will get `nruns` executions lasting `run_duration` seconds. Note that these values are simply appended to the roslaunch arguments, so you can also specify other arguments after the planner name. For instance `plannerTypes = ['dmcts nRobots:=1', 'dmcts nRobots:=2']` will run DMCTS for 1 robot and 2 robots.
- `cmd`: this is the main `roslaunch` command to which the values of `plannerTypes` are appended. You can use this to specify arguments that should be the same for all runs.

See [Launchfiles and Arguments](#launchfiles-and-arguments) for an exhaustive list of valid roslaunch arguments.

### `scripts/plot.py`

Example usage: 
```
./scripts/plot.py -n2 -l"DMCTS,Cluster" dmce_sim/logs/2robots/dmcts/ dmce_sim/logs/2robots/cluster/
```

Plots will be written to `<CWD>/plots/*.png`. See `./scripts/plot.py -h` for more options.

### `scripts/stats.py`

Example usage: 
```
./scripts/stats.py dmce_sim/logs/2robots/dmcts/ dmce_sim/logs/2robots/cluster/
```

See `./scripts/stats.py -h` for more details.


## Code Overview

### `Makefile` Targets
The top-level `Makefile` has several targets, particularly useful to run the simulation in a Docker container. Before use, make sure Docker is installed and the following variables match your setup:
```
# Catkin workspace on the local machine where build artifacts will be stored
CATKIN_WS ?= "${HOME}/catkin_ws"

# Location of the git repository on the local machine (mounted for source)
GIT ?= "${HOME}/ros_repos/"
```

- `docker-build`: build the Docker container
- `build-pkg`: build the required ROS packages
- `sim`: run the headless sim in the Docker container
- `demo`: run the sim with GUI inside the Docker container, forwarding the RViz window
- `test`: run all automated tests
- `utest`: run unit tests
- `ntest`: run node-level tests
- `itest`: run integration tests
- `clean`: remove build artifacts
- `logclean`: remove run logs from `dmce_sim/logs/`

With `make sim` and `make demo` you can set roslaunch arguments with the `ARGS` variable. For instance:
```
make demo ARGS="scenario:=urban1 plannerType:=cluster nRobots:=2"
```
### Launchfiles and Arguments
The main entry point for the simulation is the `dmce_sim/sim.launch` launchfile, which launches the simulation in headless mode. `demo.launch` is the same but additionally opens an RViz window. The following roslaunch arguments are recognized:

- `plannerType`: planner to run. Possible values: random, frontier, cluster, mcts, **dmcts**, rrt, mmpf
- `nRobots`: number of robots to simulate. Maximum 5.
- `scenario`: scenario to use. Will include the `dmce_sim/config/scenarios/<scenario>.yaml` parameter file, which determines the map and starting conditions. Possible values: **tunnels**, tunnels_45, urban1, urban2, urban_full.
- `restrictComms`: boolean, default **true**. If true, LoS restrictrions will be applied to communications between robots.
- `timeMultiplier`: scaling applied to simulation time VS wall-clock time. E.g. with `timeMultiplier=.5`, every second of real time is half a second of simulated time.
- `gamma`: iteration discounting factor, (0;1].
- `minRollouts`: minimum number of MCTS rollouts before proposing a plan.
- `rolloutDepth`: depth for rollouts in MCTS and DMCTS planners.
- `explorationFactor`: exploration factor in MCTS and DMCTS planners.

### Automated tests
This project uses automated tests, divided into three levels:
- **Unit tests** verify the business logic aspects of the code. These are independent of ROS communication and therefore do not require a ROS Master to be running during the tests. This makes them fast to run. Execute with `make utest`.
- **Node-level tests** verify a single ROS node, which usually handles inter-process communication but little business logic. These are launched with `rostest` and are therefore slower. Execute with `make ntest`.
- **Integration tests** verify the full simulation stack running in unison, checking that all required topics and services are published. Execute with `make itest`.


## Adding Scenarios and maps
In order to add a new map to simulate:

1. Create a black-and-white PNG file as the ground-truth map. Note that the colours are inverted w.r.t. what you see in RViz: black means open space (occupancy probability 0) and white means occupied space (occupancy probability 1). Make sure you have no grey pixels, only pure white or black (in GIMP, use Colors > Threshold).
2. Save the PNG map in `dmce_sim/maps/`.
3. Create a scenario config file in `dmce_sim/config/scenarios/<scenario-name>.yaml`. Here is a minimal example:
```
globalMap:
  scenarioName: "<scenario-name>"
  resolution: 0.16  # Resolution of the map in meters/pixel
  groundTruthImage: "$(find dmce_sim)/maps/<my-scenario-map>.png"

robot:
  startingArea: # Must be in a valid (traversable) position for your map
    position_x: 23 # [m]
    position_y: 23 # [m]
```
4. You should now be able to run the scenario with `make demo ARGS="scenario:=<scenario-name>"`.

Note that the scenario's YAML file can set or override any number of parameters, as it is included after the main parameter file `dmce_sim/config/sim.yaml`. See for instance [urban_full.yaml](dmce_sim/config/scenarios/urban_full.yaml).

When determining the starting position, remember that coordinates `(0,0)` are at the center of the map.


## Questions and bug reports

For any questions or bug reports please use the issue tracker on the [Github repository](https://github.com/VIS4ROB-lab/dmce).
