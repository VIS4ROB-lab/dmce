#pragma once

#include "visualization_msgs/Marker.h"

#include "mre_dmcts_nodes/NodeServer.hpp"
#include "mre_dmcts_core/TypeDefs.hpp"
#include "mre_dmcts_core/RobotConnectivity.hpp"
#include "mre_dmcts_sim/GroundTruthMap.hpp"
#include "mre_dmcts_msgs/GetMap.h"
#include "mre_dmcts_msgs/RobotPosition.h"

namespace mre_dmcts {
	/**
	 * This server checks which robots (including ground station) can communicate
	 * with each other and publishes connectivity updates.
	 */
	class ConnectivityServer : public NodeServer {
		std::vector<ros::Subscriber> positionSubscribers_;
		std::vector<pos_t> robotPositions_;
		ros::Publisher connectivityPublisher_;
		OccupancyMap truthMap_;
		bool restrictComms_ = false;

		/**
		 * Checks if there is a line of sight between the given points.
		 */
		bool lineOfSight_(const pos_t& from, const pos_t& to) const;

		/**
		 * Checks line of sight between each pair of robots (and the ground-station),
		 * then publishes the network connectivity.
		 */
		void update_(ros::Duration timeStep);

		/**
		 * Called when receiving a position update from a robot.
		 */
		void positionCallback_(const mre_dmcts_msgs::RobotPosition& msg);

	public:
		ConnectivityServer(ros::NodeHandle nh, unsigned int robotId);
	};
}
