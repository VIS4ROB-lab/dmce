#include "dmce_nodes/PlannerServer.hpp"

namespace dmce {
	PlannerServer::PlannerServer(ros::NodeHandle nh, unsigned int robotId, const double& timeout)
		: NodeServer(nh, robotId), timeSinceLastMapCheck_(0)
	{
		preferredRate_ = 0;
		groundStationPosition_ = fetchGroundStationPosition();

		getRequiredParam("/robot/diameter", robotDiameter_);
		getRequiredParam("/robot/explorationCompletionThreshold", explorationCompletionThreshold_);
		std::string plannerType;
		getRequiredParam("~plannerType", plannerType);

		std::string planningMapTopic = "inflatedMap";
		if (plannerType == "random") {
			planner_ = std::make_unique<RandomPlanner>(robotDiameter_);
		} else if (plannerType == "frontier") {
			planner_ = std::make_unique<FrontierPlanner>(robotDiameter_);
		} else if (plannerType == "cluster") {
			planner_ = std::make_unique<ClusteredFrontierPlanner>(robotDiameter_);
		} else if (plannerType == "mcts") {
			planner_ = std::make_unique<MCTSPlanner>(robotDiameter_, getMCParams());
		} else if (plannerType == "dmcts") {
			planner_ = std::make_unique<DMCTSPlanner>(robotDiameter_, robotId_, getMCParams());
		} else if (plannerType == "rrt" || plannerType == "mmpf") {
			planner_ = std::make_unique<ExternalPlanner>(robotDiameter_, robotId_, plannerType);
		} else {
			logError("", "Unrecognised planner type: '%s'", plannerType.c_str());
			ros::shutdown();
		}

		mapSubscriber_ =
			nodeHandle_.subscribe(
				planningMapTopic, 1,
				&PlannerServer::mapCallback_,
				this
		);

		clusterSubscriber_ =
			nodeHandle_.subscribe(
				"FrontierClusters", 1,
				&PlannerServer::clusterCallback_,
				this
		);
		
		positionSubscriber_ =
			nodeHandle_.subscribe(
				"RobotPosition", 1,
				&PlannerServer::positionCallback_,
				this
		);

		server_ = nodeHandle_.advertiseService(
			"GlobalPlannerService",
			&PlannerServer::planServiceCallback_,
			this
		);

		planPublisher_ = nodeHandle_.advertise<dmce_msgs::RobotPlan>("SharedPlans", 10);
		otherRobotPlanSubscribers_ = subscribeForEachOtherRobot(
				"SharedPlans", 10,
				&PlannerServer::peerPlanCallback_, this
			);

		markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(
				"/viz/visualization_marker", 10
			);

		std::stringstream ss;
		ss << "robot" << robotId_ << "/planning";
		arrowMarker_ = utils::getDefaultMarker_(0.3*robotDiameter_);
		arrowMarker_.ns = ss.str();
		arrowMarker_.scale.y = robotDiameter_;
		arrowMarker_.scale.z = robotDiameter_;
		arrowMarker_.color.r = 0.2; arrowMarker_.color.g = 0.2;
		arrowMarker_.color.b = 0.7; arrowMarker_.color.a = 0.3;
		arrowMarker_.lifetime = ros::Duration(0.1);
		arrowMarker_.type = visualization_msgs::Marker::ARROW;
	};

	MCParams PlannerServer::getMCParams() {
		MCParams params;
		getRequiredParam("/mcts/reuseBranches", params.reuseBranches);
		getRequiredParam("/mcts/rolloutDepth", params.rolloutDepth);
		getRequiredParam("/mcts/minRollouts", params.minRollouts);
		getRequiredParam("/mcts/minPlanDepth", params.minPlanDepth);
		getRequiredParam("/mcts/minPlanValue", params.minPlanValue);
		getRequiredParam("/mcts/explorationFactor", params.explorationFactor);
		getRequiredParam("/mcts/iterationDiscountFactor", params.iterationDiscountFactor);
		getRequiredParam("/mcts/robotLidarRayCount", params.robotLidarRayCount);
		getRequiredParam("/mcts/robotSensorRange", params.robotSensorRange);
		getRequiredParam("/robot/speed", params.robotSpeed);
		getRequiredParam("/mcts/useActionCaching", params.useActionCaching);
		getRequiredParam("/mcts/spatialHashBinSize", params.spatialHashBinSize);
		getRequiredParam("/robot/navigationCutoff", params.navigationCutoff);
		getRequiredParam("/mcts/randomDisplacement/maxTurnAngle", params.randomDisplacementMaxTurnAngle);
		getRequiredParam("/mcts/randomDisplacement/spreadAngle", params.randomDisplacementMinSpread);
		getRequiredParam("/mcts/randomDisplacement/length", params.randomDisplacementLength);
		getRequiredParam("/mcts/randomDisplacement/branchingFactor", params.randomDisplacementBranchingFactor);
		getRequiredParam("/mcts/useLocalReward", params.useLocalReward);
		getRequiredParam("/mcts/planBufferSize", params.planBufferSize);
		getRequiredParam("/mcts/timeDiscountFactor", params.timeDiscountFactor);
		getRequiredParam("/mcts/actionBaseDuration", params.actionBaseDuration);
		getRequiredParam("/mcts/frontierClusterAction/branchingFactor", params.frontierClusterBranchingFactor);
		return params;
	}

	void PlannerServer::update_(ros::Duration timeStep) {
		timeSinceLastMapCheck_ += timeStep;

		if (timeSinceLastMapCheck_.toSec() >= 10) {
			bool wasFinished = explorationFinished_;
			explorationFinished_ = isMapFullyExplored_(map_);
			if (!wasFinished && explorationFinished_)
				logInfo("mapCallback_", "Exploration complete. Returning to ground station.");
			timeSinceLastMapCheck_ = ros::Duration(0);
		}

		if (explorationFinished_)
			return;

		planner_->updatePlan();

		dmce_msgs::RobotPlan msg;
		msg.robotId = getRobotId();
		msg.path.header.frame_id = "map";
		msg.path.header.stamp = ros::Time::now();
		bool success;
		std::tie(success, msg.path.poses) = planner_->getPlanToShare();
		if (success)
			publishPlan_(msg);
	}

	void PlannerServer::publishPlan_(const dmce_msgs::RobotPlan& planMsg) {
		planPublisher_.publish(planMsg);

		for (unsigned int i = 1; i < planMsg.path.poses.size(); i++) {
			arrowMarker_.id = i;
			arrowMarker_.header.stamp = ros::Time::now();
			arrowMarker_.points.clear();
			arrowMarker_.points.push_back(planMsg.path.poses[i-1].pose.position);
			arrowMarker_.points.push_back(planMsg.path.poses[i  ].pose.position);
			markerPublisher_.publish(arrowMarker_);
		}
	}

	void PlannerServer::positionCallback_(
		const dmce_msgs::RobotPosition& msg
	) {
		planner_->setPosition({msg.x_position, msg.y_position});
	}

	void PlannerServer::mapCallback_(const grid_map_msgs::GridMap& msg) {
		map_ = msg;
		planner_->setMap(msg);
	}

	bool PlannerServer::isMapFullyExplored_(const RobotMap& map) const {
		double relEntropy = map.getRelativeEntropy();
		if (relEntropy >= 0.99)
			return false;

		double relFrontierSize = map.getFrontier().size() / (double)map.getNCells();
		return relFrontierSize < explorationCompletionThreshold_;
	}

	void PlannerServer::peerPlanCallback_(const dmce_msgs::RobotPlan& msg) {
		if (explorationFinished_)
			return;

		if (canReceiveFromPeer(msg.robotId)) {
			planner_->peerPlanCallback(msg);
		}
	}

	bool PlannerServer::planServiceCallback_(
		dmce_msgs::GetPlan::Request& request,
		dmce_msgs::GetPlan::Response& response)
	{
		if (explorationFinished_) {
			response.plan.poses.push_back(posToPose(groundStationPosition_));
			return true;
		}

		planner_->setPosition({request.currentPosition.x_position, request.currentPosition.y_position});
		if (!request.success) {
			logWarn("planServiceCallback_", "Signalling navigation failure!");
			planner_->signalNavigationFailure();
		}
		bool success;
		std::tie(success, response.plan.poses) = planner_->getLatestPlan();
		if (success)
			logInfo("planServiceCallback_", "Plan size: %zu", response.plan.poses.size());
		return success;
	}


	void PlannerServer::clusterCallback_(const dmce_msgs::FrontierClusters& msg) {
		planner_->frontierClusterCallback(msg);
	}
};
