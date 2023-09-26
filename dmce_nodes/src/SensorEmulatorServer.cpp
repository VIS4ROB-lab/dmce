#include "dmce_nodes/SensorEmulatorServer.hpp"

namespace dmce {
	void SensorEmulatorServer::positionCallback_(const dmce_msgs::RobotPosition& msg) {
		grid_map::Position pos{msg.x_position, msg.y_position};
		auto mapUpdate = sensorEmulator_.getMapUpdate(pos);
		mapUpdate.robotId = robotId_;
		mapUpdatePublisher_.publish(mapUpdate);
	}

	SensorEmulatorServer::SensorEmulatorServer(ros::NodeHandle nh, unsigned int robotId, const double& timeout)
		: NodeServer(nh, robotId)
	{
		auto truthMap = fetchGroundTruthMap_(timeout);

		logInfo("", "Service called.");

		double sensorRange;
		getRequiredParam("/robot/sensorRange", sensorRange);

		int lidarRayCount;
		getRequiredParam("/robot/lidarRayCount", lidarRayCount);

		sensorEmulator_ = SensorEmulator(sensorRange, lidarRayCount, truthMap);

		positionSubscriber_ =
			nodeHandle_.subscribe(
				"RobotPosition", 1,
				&SensorEmulatorServer::positionCallback_,
				this
			);

		mapUpdatePublisher_ =
			nodeHandle_.advertise<dmce_msgs::RobotMapUpdate>("RobotMapUpdates", 10);
	}
}
