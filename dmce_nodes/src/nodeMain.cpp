#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <thread>

#include <ros/ros.h>

#include "dmce_core/utils.hpp"
#include "dmce_core/TypeDefs.hpp"
#include "dmce_nodes/NodeServer.hpp"
#include "dmce_nodes/SensorEmulatorServer.hpp"
#include "dmce_nodes/GroundTruthMapServer.hpp"
#include "dmce_nodes/RobotMapServer.hpp"
#include "dmce_nodes/GlobalMapServer.hpp"
#include "dmce_nodes/NavigationServer.hpp"
#include "dmce_nodes/RobotMarkerServer.hpp"
#include "dmce_nodes/PlannerServer.hpp"
#include "dmce_nodes/LoggingServer.hpp"
#include "dmce_nodes/TransformServer.hpp"
#include "dmce_nodes/ConnectivityServer.hpp"
#include "dmce_nodes/FrontierClusterServer.hpp"
#include "dmce_nodes/PathfindingServer.hpp"

template<typename T> 
std::unique_ptr<dmce::NodeServer> createInstance(ros::NodeHandle& nh, unsigned int robotId) {
    return std::make_unique<T>(nh, robotId);
}

typedef std::map<std::string, std::unique_ptr<dmce::NodeServer>(*)(ros::NodeHandle&, unsigned int)> map_type;

int main(int argc, char **argv) {
    ros::init(argc, argv, "nodeMain");
    ros::NodeHandle nodeHandle;
    auto nodeName = ros::this_node::getName();

    if (argc < 3)
        throw std::runtime_error("[nodeMain] Insufficient arguments provided! Usage: nodeMain [robotId] [NodeServerType]");

    // First argument is the robot ID
    unsigned int robotId;
    {
        int tmp = std::atoi(argv[1]);
        robotId = (tmp < 0) ? 0 : tmp;
    }

    // Seed the random number generator
    if (ros::param::has("/randomSeed")) {
        int seed = 0;
        ros::param::get("/randomSeed", seed);
        if (seed != 0) {
            dmce::utils::RNG::init(seed + robotId);
        }
    }

	map_type serverMap = {
     {"GlobalMapServer", &createInstance<dmce::GlobalMapServer>},
     {"RobotMapServer", &createInstance<dmce::RobotMapServer>},
	 {"GroundTruthMapServer", &createInstance<dmce::GroundTruthMapServer>},
	 {"SensorEmulatorServer", &createInstance<dmce::SensorEmulatorServer>},
	 {"NavigationServer", &createInstance<dmce::NavigationServer>},
	 {"RobotMarkerServer", &createInstance<dmce::RobotMarkerServer>},
	 {"PlannerServer", &createInstance<dmce::PlannerServer>},
	 {"LoggingServer", &createInstance<dmce::LoggingServer>},
	 {"TransformServer", &createInstance<dmce::TransformServer>},
	 {"ConnectivityServer", &createInstance<dmce::ConnectivityServer>},
	 {"FrontierClusterServer", &createInstance<dmce::FrontierClusterServer>},
	 {"PathfindingServer", &createInstance<dmce::PathfindingServer>}
    };

    std::unique_ptr<dmce::NodeServer> instance;

    std::string key = argv[2];
    if (serverMap.find(key) != serverMap.end()) {
        ROS_INFO("[%u:%s] Starting server: %s...", robotId, nodeName.c_str(), key.c_str());
        instance = serverMap[key](nodeHandle, robotId);
        ROS_INFO("[%u:%s] %s started.", robotId, nodeName.c_str(), key.c_str());
    } else {
        std::stringstream ss;
        ss << "[" << robotId << ":" << nodeName << "] Invalid NodeServer type: '" << key << "'.";
        throw std::invalid_argument(ss.str());
    }

    unsigned int preferredRate = instance->getPreferredRate();
    ros::Rate rate(preferredRate > 0 ? preferredRate : 1);
    ros::Time prevTime = ros::Time::now();

    while (ros::ok()) {
        ros::Time newTime = ros::Time::now();
        ros::Duration timeStep = newTime - prevTime;
        prevTime = newTime;

        instance->update(timeStep);
        ros::spinOnce();

        if (preferredRate > 0)
            rate.sleep();
    }

    printf("[%u:%s] shut down.\n", robotId, nodeName.c_str());

    return 0;
}

