#include "dmce_nodes/GlobalMapServer.hpp"

namespace dmce {

	GlobalMapServer::GlobalMapServer(ros::NodeHandle nh, unsigned int robotId, double timeout)
		: NodeServer(nh, robotId)
	{
		double r;
		getRequiredParam("/globalMap/resolution", r);
		auto truthMap = fetchGroundTruthMap_(timeout);

		map_ = RobotMap(truthMap.getLength(), r, truthMap.getPosition(), "map");

		mapPublisher_ =
			nodeHandle_.advertise<grid_map_msgs::GridMap>(
				"GlobalMap", 1, true
			);

		publishMap_();

		updateSubscribers_ = subscribeForEachRobot(
			"RobotMapUpdates", 1,
			&GlobalMapServer::updateCallback_, this
		);
	}

	void GlobalMapServer::publishMap_() {
		auto message = map_.getMessage();
		mapPublisher_.publish(message);
	}

	void GlobalMapServer::updateCallback_(const dmce_msgs::RobotMapUpdate& msg) {
		map_.setOccupancy(msg.x_positions, msg.y_positions, msg.values, msg.length);
		publishMap_();
	}
};
