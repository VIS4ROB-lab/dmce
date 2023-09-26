#include "dmce_nodes/RobotMapServer.hpp"

namespace dmce {

	RobotMapServer::RobotMapServer(ros::NodeHandle nh, unsigned int robotId, double timeout)
		: NodeServer(nh, robotId)
	{
		getRequiredParam("/robot/diameter", robotDiameter_);
		initialiseMap_(timeout);

		mapUpdateSubscriber_ = nodeHandle_.subscribe(
			"RobotMapUpdates", 1,
			&RobotMapServer::mapUpdateCallback_, this
		);

		mapSubscribers_ = subscribeForEachOtherRobot(
			"map", 1,
			&RobotMapServer::mapCallback_, this,
			true
		);

		mapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("map", 1, true);
		inflatedMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("inflatedMap", 1, true);
	}

	
	void RobotMapServer::update_(ros::Duration timeStep) {
		publishMap_(mapPublisher_, map_);

		RobotMap inflatedMap;
		inflatedMap.fromImage(map_.inflateObstacles(map_.toImage(), (robotDiameter_ * 0.75)/map_.getResolution()), map_.getResolution());
		publishMap_(inflatedMapPublisher_, inflatedMap);
	}


	void RobotMapServer::mapUpdateCallback_(const dmce_msgs::RobotMapUpdate& msg) {
		map_.setOccupancyConservative(
				msg.x_positions,
				msg.y_positions,
				msg.values,
				msg.length
		);
	}


	void RobotMapServer::mapCallback_(const grid_map_msgs::GridMap& msg) {
		// Somewhat hacky workaround to receive robot ID
		// Not clean but saves me having to make a new message type
		// and using it across the whole sim
		unsigned int senderId = msg.info.header.stamp.toSec();
		if (canReceiveFromPeer(senderId)) {
			map_.merge(RobotMap(msg));
		}
	}


	void RobotMapServer::initialiseMap_(const double& timeout) {
		auto truthMap = fetchGroundTruthMap_(timeout);
		double res;
		getRequiredParam("/robot/map/resolution", res);
		map_ = RobotMap(truthMap.getLength(), res, truthMap.getPosition(), "map");
	}


	void RobotMapServer::publishMap_(const ros::Publisher& pub, const RobotMap& map) {
		auto msg = map.getMessage();
		// Somewhat hacky workaround to transmit robot ID
		// Not clean but saves me having to make a new message type
		// and using it across the whole sim
		msg.info.header.stamp = ros::Time(robotId_);
		pub.publish(msg);
	}
};

