#include "dmce_nodes/NavigationServer.hpp"

namespace dmce {

	NavigationServer::NavigationServer(ros::NodeHandle nh, unsigned int robotId, double timeout)
		: NodeServer(nh, robotId), actionClient_("Pathfinding", false)
	{
		getRequiredParam("/robot/speed", robotSpeed_);
		getRequiredParam("/robot/navigationCutoff", navigationCutoff_);
		getRequiredParam("/robot/diameter", robotDiameter_);
		getRequiredParam("/robot/randomiseInitialPosition", randomiseInitialPosition_);
		getRequiredParam("/robot/maxPlanAge", maxPlanAge_);
		getRequiredParam("/robot/maxPathfindingFailures", maxPathfindingFailures_);

		inflatedTruthMap_ = fetchGroundTruthMap_(timeout);
		inflatedTruthMap_.inflateObstacles(robotDiameter_ * .75);

		globalPlanClient_ = nodeHandle_.serviceClient<dmce_msgs::GetPlan>(
			"GlobalPlannerService"
		);
		if (!globalPlanClient_.waitForExistence(ros::Duration(timeout))) {
			throw std::runtime_error(
				"[NavigationServer] GlobalPlannerService not found!"
			);
		}

		positionPublisher_ =
			nodeHandle_.advertise<dmce_msgs::RobotPosition>("RobotPosition", 1, true);

		pathPublisher_ = nodeHandle_.advertise<nav_msgs::Path>("path", 1);

		globalPlanPublisher_ = nodeHandle_.advertise<nav_msgs::Path>("GlobalPlan", 1);

		failurePublisher_ = nodeHandle_.advertise<dmce_msgs::NavigationFailureSignal>(
				"navigationFailure", 10
		);

		inflatedMapSubscriber_ = nodeHandle_.subscribe(
			"inflatedMap", 1,
			&NavigationServer::navigationMapCallback_,
			this
		);

		pos_t startingPosition = getStartingPosition_(fetchStartingArea_(timeout));

		navigation_ = std::make_unique<Navigation>(
			inflatedTruthMap_,
			startingPosition,
			robotSpeed_,
			navigationCutoff_
		);
	}

	void NavigationServer::update_(ros::Duration timeStep) {
		if (navigation_->hasGoal())
			checkAbortConditions_(timeStep);

		publishPlan_(navigation_->getCurrentPlan(), globalPlanPublisher_);
		publishPlan_(navigation_->getCurrentPath(), pathPublisher_);

		bool navigationSuccess = navigateToGoal_(timeStep);

		if (!navigation_->hasGoal()) {
			actionClient_.cancelAllGoals();
			tryFetchNewPlan_(navigationSuccess);
			if (navigation_->hasGoal())
				sendGoalToPathfinding_();
		}

		publishRobotPosition_();
	}

	bool NavigationServer::navigateToGoal_(const ros::Duration& timeStep) {
		if (!navigation_->isReady())
			return true;

		pos_t goalPos = navigation_->getCurrentGoal();
		try {
			navigation_->moveRobotToGoal(timeStep);
		} catch (std::runtime_error e) {
			logError("update_", "Caught runtime error: %s", e.what());
			return false;
		}

		if (!navigation_->hasGoal()) {
			signalFailure_ = !navigation_->wasLastPlanSuccessful();
			if (signalFailure_)
				logWarn("update_", "Global plan failed! Resetting.");
			return !signalFailure_;
		}

		return true;
	}

	void NavigationServer::checkAbortConditions_(const ros::Duration& timeStep) {
		currentPlanAge_ += timeStep.toSec();
		bool planTimedOut = currentPlanAge_ > maxPlanAge_;
		bool goalUnreachable = consecutivePathfindingFailures_ > maxPathfindingFailures_;

		if (planTimedOut)
			logWarn("update_", "Plan timed out after %.2fs. Resetting.", currentPlanAge_);
		else if (goalUnreachable) {
			logError("update_", "Stuck on unreachable goal! Resetting.");
			consecutivePathfindingFailures_ = 0;
			dmce_msgs::NavigationFailureSignal failureMsg;
			failureMsg.robotId = robotId_;
			failurePublisher_.publish(failureMsg);
		}

		if (planTimedOut || goalUnreachable) {
			navigation_->resetPlan();
			currentPlanAge_ = 0;
		}
	}

	void NavigationServer::navigationMapCallback_(const grid_map_msgs::GridMap& mapMsg) {
		navigation_->updateNavigationMap(OccupancyMap(mapMsg));
	}

	pos_t NavigationServer::getStartingPosition_(
		dmce_msgs::GetStartingArea::Response startingArea
	) {
		if (!randomiseInitialPosition_)
			return {startingArea.position_x, startingArea.position_y};

		double minx = startingArea.position_x - startingArea.width * .5;
		double miny = startingArea.position_y - startingArea.height * .5;
		double maxx = startingArea.position_x + startingArea.width * .5;
		double maxy = startingArea.position_y + startingArea.height * .5;

		auto gen = utils::RNG::get();
		std::uniform_real_distribution<> x_dis(minx, maxx);
		std::uniform_real_distribution<> y_dis(miny, maxy);

		auto isValid = [](const pos_t& pos, const OccupancyMap& map) -> bool {
			return map.isInside(pos) && !map.isOccupied(pos);
		};

		pos_t startingPosition{0,0};
		do {
			startingPosition = {x_dis(gen), y_dis(gen)};
		} while (!isValid(startingPosition, inflatedTruthMap_));

		return startingPosition;
	}

	void NavigationServer::publishRobotPosition_() {
		dmce_msgs::RobotPosition msg;
		pos_t robotPosition = getCurrentRobotPosition_();
		msg.robotId = robotId_;
		msg.x_position = robotPosition.x();
		msg.y_position = robotPosition.y();
		positionPublisher_.publish(msg);
	}

	pos_t NavigationServer::getCurrentRobotPosition_() const {
		return navigation_->getRobotPosition();
	}

	void NavigationServer::publishPlan_(const plan_t& plan, const ros::Publisher& publisher) {
		nav_msgs::Path gui_path;
		gui_path.poses.resize(plan.size() + 1);
		gui_path.header.frame_id = "map";
		gui_path.header.stamp = ros::Time::now();

		gui_path.poses[0] = posToPose(getCurrentRobotPosition_());
		std::copy(plan.begin(), plan.end(), gui_path.poses.begin()+1);
		
		publisher.publish(gui_path);
	}

	void NavigationServer::tryFetchNewPlan_(bool goalSuccess) {
		auto dist = [](const pos_t& from, const pos_t& to) -> double {
			return (from - to).norm();
		};
		auto sanitisePlan = [&dist](plan_t plan, const pos_t& curPos, double cutoff) -> plan_t {
			while (!plan.empty() && dist(poseToPos(plan[0]), curPos) < cutoff) {
				plan.erase(plan.begin());
			}
			return plan;
		};

		dmce_msgs::GetPlan plan_msg;
		// plan_msg.request.success = !signalFailure_;
		plan_msg.request.success = goalSuccess;
		plan_msg.request.currentPosition.robotId = robotId_;
		plan_msg.request.currentPosition.x_position = getCurrentRobotPosition_().x();
		plan_msg.request.currentPosition.y_position = getCurrentRobotPosition_().y();
		if (globalPlanClient_.call(plan_msg.request, plan_msg.response)) {
			navigation_->updatePlan(sanitisePlan(
						plan_msg.response.plan.poses,
						getCurrentRobotPosition_(),
						navigationCutoff_
					));
			if (navigation_->hasGoal()) {
				currentPlanAge_ = 0;
			} else {
				logError("tryFetchNewPlan_", "Received invalid plan!");
			}
		}

		signalFailure_ = false;
	}

	void NavigationServer::pathCallback_(
			const dmce_msgs::PathfindingFeedbackConstPtr& pathFeedback)
	{
		if (pathFeedback->success) {
			navigation_->updatePath(pathFeedback->path.poses);
			consecutivePathfindingFailures_ = 0;
		} else {
			consecutivePathfindingFailures_++;
			logWarn("pathCallback_", "Consecutive pathfinding failures: %zu", consecutivePathfindingFailures_);
		}
	}

	void NavigationServer::sendGoalToPathfinding_() {
		pos_t goalPosition;
		try {
			goalPosition = navigation_->getCurrentGoal();
		} catch (std::runtime_error e) {
			logError("sendGoalToPathfinding_", "Got exception when trying to get goal!");
			return;
		}
		dmce_msgs::PathfindingGoal goalMsg;
		goalMsg.robotId = 0;
		goalMsg.x_goal = goalPosition.x();
		goalMsg.y_goal = goalPosition.y();

		actionClient_.sendGoal(goalMsg,
			ActionClient_t::SimpleDoneCallback(),
			ActionClient_t::SimpleActiveCallback(),
			boost::bind(&NavigationServer::pathCallback_, this, _1));
	}
}

