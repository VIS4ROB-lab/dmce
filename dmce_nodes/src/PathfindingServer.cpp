#include "dmce_nodes/PathfindingServer.hpp"

namespace dmce {
	PathfindingServer::PathfindingServer(ros::NodeHandle nh, unsigned int robotId)
		: NodeServer(nh, robotId),
		  actionServer_(nh, "Pathfinding", false),
		  pathfinder_(OccupancyMap{})
	{
		preferredRate_ = 500;

		getRequiredParam("/robot/diameter", robotDiameter_);
		getRequiredParam("/robot/speed", robotSpeed_);
		getRequiredParam("/robot/navigationCutoff", navigationCutoff_);

		positionSubscriber_ = nodeHandle_.subscribe(
			"RobotPosition", 1, &PathfindingServer::positionCallback_, this);

		mapSubscriber_ = nodeHandle_.subscribe(
			"inflatedMap", 1, &PathfindingServer::mapCallback_, this);

		actionServer_.registerGoalCallback(
				boost::bind(&PathfindingServer::goalCallback_, this));
		actionServer_.registerPreemptCallback(
				boost::bind(&PathfindingServer::preemptCallback_, this));

		actionServer_.start();
	}

	void PathfindingServer::update_(ros::Duration timeStep) {
		if (!receivedGoal_ || !receivedPosition_ || !receivedMap_)
			return;

		if (!actionServer_.isActive())
			return;

		double distanceToGoal = (robotPosition_ - goalPosition_).norm();
		if (distanceToGoal < navigationCutoff_) {
			dmce_msgs::PathfindingResult result;
			actionServer_.setSucceeded(result);
			return;
		}

		dmce_msgs::PathfindingFeedback feedbackMsg;
		std::tie(feedbackMsg.success, feedbackMsg.path.poses) =
			pathfinder_.makePath(robotPosition_, goalPosition_);

		feedbackMsg.path.poses.push_back(posToPose(goalPosition_));
		actionServer_.publishFeedback(feedbackMsg);
		if (!feedbackMsg.success)
			logWarn("update_", "Failed to find path!");
	}

	void PathfindingServer::goalCallback_() {
		auto goalMsg = actionServer_.acceptNewGoal();
		goalPosition_.x() = goalMsg->x_goal;
		goalPosition_.y() = goalMsg->y_goal;
		receivedGoal_ = true;
	}

	void PathfindingServer::preemptCallback_() {
		actionServer_.setPreempted();
		receivedGoal_ = false;
	}

	void PathfindingServer::positionCallback_(const dmce_msgs::RobotPosition& msg) {
		robotPosition_.x() = msg.x_position;
		robotPosition_.y() = msg.y_position;
		receivedPosition_ = true;
	}

	void PathfindingServer::mapCallback_(const grid_map_msgs::GridMap& mapMsg) {
		pathfinder_.updateMap(OccupancyMap(mapMsg));
		receivedMap_ = true;
	}
}

