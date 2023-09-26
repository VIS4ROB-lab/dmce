#pragma once

#include "dmce_nodes/NodeServer.hpp"
#include "dmce_core/RobotMap.hpp"

#include "dmce_msgs/RobotMapUpdate.h"

#include <memory>

namespace dmce {

	/**
	 * This node aggregates exploration data from each robot and
	 * reconstructs the global map.
	 * It listens for map updates on each robotN/RobotMapUpdates topic,
	 * and updates the RobotMap accordingly.
	 * When the map is updated, it publishes it on the GlobalMap topic
	 * as a grid_map_msgs::GridMap message.
	 * Unlike RobotMapServer, this server ignores communication
	 * restrictions and reconstructs a full-resolution map directly
	 * from the robots's sensor streams.
	 * The GlobalMap topic should only be used for evaluation purposes (not planning).
	 */
	class GlobalMapServer : public NodeServer {
		RobotMap map_;

		ros::Publisher mapPublisher_;
		std::vector<ros::Subscriber> updateSubscribers_;

		/**
		 * Publishes the map on the RobotMap topic.
		 */
		void publishMap_();

		/**
		 * Callback for the RobotMapUpdates topic subscription.
		 * Merges the received updates with the current map and publishes
		 *  the updated map.
		 */
		void updateCallback_(const dmce_msgs::RobotMapUpdate& msg);

	protected:
		void update_(ros::Duration timeStep) override {}

	public:
		/**
		 * Constructor.
		 * Initialises the RobotMap based on the settings found on the ROS parameter server.
		 */
		GlobalMapServer(ros::NodeHandle nh, unsigned int robotId, double timeout = 15);
		
		virtual std::string getName() const override {
			return "GlobalMapServer";
		}
	};
};
