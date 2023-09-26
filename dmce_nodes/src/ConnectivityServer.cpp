#include "dmce_nodes/ConnectivityServer.hpp"

namespace dmce {
	bool ConnectivityServer::lineOfSight_(const pos_t& from, const pos_t& to) const {
		auto it = truthMap_.getLineIterator(from, to);
		for ( ; !it.isPastEnd(); ++it) {
			if (truthMap_.isOccupied(*it))
				return false;
		}
		return true;
	}

	void ConnectivityServer::update_(ros::Duration timeStep) {
		auto nrobots = getRobotCount();
		RobotConnectivity connectivityHandler(getRobotCount());

		if (restrictComms_)
			for (unsigned int from = 0; from <= nrobots; from++)
				for (unsigned int to = from+1; to <= nrobots; to++)
					if (!lineOfSight_(robotPositions_[from], robotPositions_[to]))
						connectivityHandler.disconnectBoth(from, to);

		connectivityPublisher_.publish(connectivityHandler.getMessage());
	}

	void ConnectivityServer::positionCallback_(const dmce_msgs::RobotPosition& msg) {
		robotPositions_[msg.robotId] = { msg.x_position, msg.y_position };
	}

	ConnectivityServer::ConnectivityServer(ros::NodeHandle nh, unsigned int robotId)
		: NodeServer(nh, robotId), robotPositions_(getRobotCount()+1, {0, 0})
	{
		getRequiredParam("/robot/restrictComms", restrictComms_);
		robotPositions_[0] = fetchGroundStationPosition();
		positionSubscribers_ = subscribeForEachRobot(
			"RobotPosition", 1, &ConnectivityServer::positionCallback_, this
		);
		connectivityPublisher_ = nodeHandle_.advertise<RobotConnectivity::Message>(
			"RobotConnectivity", 1, true
		);

		truthMap_ = fetchGroundTruthMap_();
	}
}
