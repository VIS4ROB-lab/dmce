#pragma once

#include "mre_dmcts_nodes/NodeServer.hpp"
#include "mre_dmcts_sim/SensorEmulator.hpp"
#include "mre_dmcts_sim/GroundTruthMap.hpp"

#include "mre_dmcts_msgs/RobotPosition.h"
#include "mre_dmcts_msgs/GetMap.h"

namespace mre_dmcts {
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

		void positionCallback_(const mre_dmcts_msgs::RobotPosition& msg);

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
