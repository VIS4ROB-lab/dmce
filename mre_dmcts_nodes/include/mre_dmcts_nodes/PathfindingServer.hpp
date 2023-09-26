#pragma once

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "mre_dmcts_nodes/NodeServer.hpp"
#include "mre_dmcts_msgs/PathfindingAction.h"
#include "mre_dmcts_msgs/RobotPosition.h"

#include "mre_dmcts_sim/OmplPathfinding.hpp"

namespace mre_dmcts {
	/**
	 * This node makes an actionlib server available which
	 * computes paths from the current robot position to a given
	 * goal location.
	 * A new path is evaluated at each update, until either the
	 * action is called or the robotPosition matches the goal.
	 */
	class PathfindingServer : public NodeServer {
		double robotDiameter_;
		double robotSpeed_;
		double navigationCutoff_;

		bool receivedPosition_ = false;
		pos_t robotPosition_;
		ros::Subscriber positionSubscriber_;

		bool receivedMap_ = false;
		ros::Subscriber mapSubscriber_;

		bool receivedGoal_ = false;
		pos_t goalPosition_;

		actionlib::SimpleActionServer<mre_dmcts_msgs::PathfindingAction> actionServer_;
		OmplPathfinding pathfinder_;


		void update_(ros::Duration timeStep) override;

		void goalCallback_();

		void preemptCallback_();

		void positionCallback_(const mre_dmcts_msgs::RobotPosition& msg);

		void mapCallback_(const grid_map_msgs::GridMap& mapMsg);

	public:
		PathfindingServer(ros::NodeHandle nh, unsigned int robotId);

		virtual std::string getName() const override {
			return "PathfindingServer";
		}
	};
}

