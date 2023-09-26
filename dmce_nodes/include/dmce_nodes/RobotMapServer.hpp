#pragma once

#include "dmce_nodes/NodeServer.hpp"
#include "dmce_core/RobotMap.hpp"
#include "dmce_core/RobotConnectivity.hpp"

#include "dmce_msgs/RobotMapUpdate.h"
#include "dmce_msgs/RobotPosition.h"

#include <memory>

namespace dmce {
	/**
	 * This server listens for updates from this robot's SensorEmulatorServer
	 * as well as from other robots' RobotMapServers.
	 * It merges these data streams to produce a single robot map which is
	 * published on the /robotN/map topic.
	 */
	class RobotMapServer : public NodeServer {
		ros::Subscriber mapUpdateSubscriber_;
		std::vector<ros::Subscriber> mapSubscribers_;
		ros::Publisher mapPublisher_;
		ros::Publisher inflatedMapPublisher_;

		RobotMap map_;
		double robotDiameter_;

		/**
		 * Callback for the /RobotMapUpdates topic subscription of this robot.
		 * Merges the received updates with the current map.
		 */
		void mapUpdateCallback_(const dmce_msgs::RobotMapUpdate& msg);

		/**
		 * Callback for each of the /robotN/map topics of the other robots.
		 * Merges the received map with the current one.
		 */
		void mapCallback_(const grid_map_msgs::GridMap& msg);

		void initialiseMap_(const double& timeout);

		void publishMap_(const ros::Publisher& pub, const RobotMap& map);

	protected:
		/**
		 * Publishes the current robot map as well as an inflated version.
		 */
		void update_(ros::Duration timeStep) override;

	public:
		/**
		 * Constructor.
		 * Initialises the RobotMap based on the settings found on the ROS parameter server.
		 */
		RobotMapServer(ros::NodeHandle nh, unsigned int robotId, double timeout = 15);
		
		virtual std::string getName() const override {
			return "RobotMapServer";
		}
	};
};
