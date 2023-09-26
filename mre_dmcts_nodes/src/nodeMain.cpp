#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <thread>

#include <ros/ros.h>

#include "mre_dmcts_core/utils.hpp"
#include "mre_dmcts_core/TypeDefs.hpp"
#include "mre_dmcts_nodes/NodeServer.hpp"
#include "mre_dmcts_nodes/SensorEmulatorServer.hpp"
#include "mre_dmcts_nodes/GroundTruthMapServer.hpp"
#include "mre_dmcts_nodes/RobotMapServer.hpp"
#include "mre_dmcts_nodes/GlobalMapServer.hpp"
#include "mre_dmcts_nodes/NavigationServer.hpp"
#include "mre_dmcts_nodes/RobotMarkerServer.hpp"
#include "mre_dmcts_nodes/PlannerServer.hpp"
#include "mre_dmcts_nodes/LoggingServer.hpp"
#include "mre_dmcts_nodes/TransformServer.hpp"
#include "mre_dmcts_nodes/ConnectivityServer.hpp"
#include "mre_dmcts_nodes/FrontierClusterServer.hpp"
#include "mre_dmcts_nodes/PathfindingServer.hpp"

template<typename T> 
std::unique_ptr<mre_dmcts::NodeServer> createInstance(ros::NodeHandle& nh, unsigned int robotId) {
    return std::make_unique<T>(nh, robotId);
}

typedef std::map<std::string, std::unique_ptr<mre_dmcts::NodeServer>(*)(ros::NodeHandle&, unsigned int)> map_type;

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
            mre_dmcts::utils::RNG::init(seed + robotId);
        }
    }

	map_type serverMap = {
     {"GlobalMapServer", &createInstance<mre_dmcts::GlobalMapServer>},
     {"RobotMapServer", &createInstance<mre_dmcts::RobotMapServer>},
	 {"GroundTruthMapServer", &createInstance<mre_dmcts::GroundTruthMapServer>},
	 {"SensorEmulatorServer", &createInstance<mre_dmcts::SensorEmulatorServer>},
	 {"NavigationServer", &createInstance<mre_dmcts::NavigationServer>},
	 {"RobotMarkerServer", &createInstance<mre_dmcts::RobotMarkerServer>},
	 {"PlannerServer", &createInstance<mre_dmcts::PlannerServer>},
	 {"LoggingServer", &createInstance<mre_dmcts::LoggingServer>},
	 {"TransformServer", &createInstance<mre_dmcts::TransformServer>},
	 {"ConnectivityServer", &createInstance<mre_dmcts::ConnectivityServer>},
	 {"FrontierClusterServer", &createInstance<mre_dmcts::FrontierClusterServer>},
	 {"PathfindingServer", &createInstance<mre_dmcts::PathfindingServer>}
    };

    std::unique_ptr<mre_dmcts::NodeServer> instance;

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

