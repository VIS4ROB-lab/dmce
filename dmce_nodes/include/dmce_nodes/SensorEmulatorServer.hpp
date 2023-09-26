#pragma once

#include "dmce_nodes/NodeServer.hpp"
#include "dmce_sim/SensorEmulator.hpp"
#include "dmce_sim/GroundTruthMap.hpp"

#include "dmce_msgs/RobotPosition.h"
#include "dmce_msgs/GetMap.h"

namespace dmce {
	/**
	 * This class makes SensorEmulator available as a ROS node.
	 * It subscribes to position updates on the RobotPosition topic,
	 * and publishes RobotMapUpdate messages on the RobotMapUpdates topic.
	 * It invokes the /GroundTruthMapServer once on instantiation to fetch
	 * a copy of the ground-truth map which is used to generate the synthetic
	 * sensor readings.
	 */
	class SensorEmulatorServer : public NodeServer {
		SensorEmulator sensorEmulator_;

		ros::Subscriber positionSubscriber_;
		ros::Publisher mapUpdatePublisher_;

		void positionCallback_(const dmce_msgs::RobotPosition& msg);

	protected:
		void update_(ros::Duration timeStep) override { }

	public:
		/**
		 * Constructor.
		 * @param timeout [s] Time to wait for the GroundTruthMapServer to come online before throwing an error.
		 */
		SensorEmulatorServer(ros::NodeHandle nh, unsigned int robotId, const double& timeout = 15);
		
		virtual std::string getName() const override {
			return "SensorEmulatorServer";
		}
	};
}
