#include "mre_dmcts_sim/Navigation.hpp"

namespace mre_dmcts {
	
	Navigation::Navigation(
			const GroundTruthMap& truthMap,
			const pos_t& initialPosition,
			double robotSpeed,
			double navigationCutoff
		) : robotPosition_(initialPosition), robotSpeed_(robotSpeed),
			navigationCutoff_(navigationCutoff), truthMap_(truthMap)
	{ }

	void Navigation::moveRobotToGoal(const ros::Duration& timeStep) {
		if (!navMapReady_)
			throw std::runtime_error("moveRobotToGoal called, but no navigation map was set!");

		if (!hasGoal())
			return;

		if (!isGoalValid_()) {
			currentPlan_.clear();
			planAborted_ = true;
			return;
		} else
			planAborted_ = false;

		if (positionCollides_(robotPosition_)) {
			ROS_ERROR("[Navigation::moveRobotToGoal] Robot is clipping! Moving to resolve.");
			moveRobotToClosestFreeSpace_();
		}

		if (!hasValidPath()) {
			currentPath_.clear();
			return;
		}

		skipPastWaypoints_(robotPosition_, currentPath_);
		moveRobotOnPath_(timeStep.toSec());

		if (positionCollides_(robotPosition_)) {
			moveRobotToClosestFreeSpace_();
			currentPlan_.clear();
			planAborted_ = true;
			return;
		}

		if (isRobotAtGoal_())
			currentPlan_.clear();
	}

	void Navigation::moveRobotToClosestFreeSpace_() {
		double closestDist = std::numeric_limits<double>::infinity();
		pos_t closestPosition = robotPosition_;
		auto it = truthMap_.getCircleIterator(robotPosition_, 10*navigationCutoff_);
		for (; !it.isPastEnd(); ++it) {
			pos_t candidatePosition;
			truthMap_.getPosition(*it, candidatePosition);
			if (!positionCollides_(candidatePosition)) {
				double dist = (candidatePosition - robotPosition_).norm();
				if (dist < closestDist) {
					closestDist = dist;
					closestPosition = candidatePosition;
				}
			}
		}
		robotPosition_ = closestPosition;
	}

	void Navigation::moveRobotOnPath_(double dt) {
		while (!currentPath_.empty() && dt > 0) {
			auto newPos_msg = currentPath_[0].pose.position;
			pos_t targetPosition{newPos_msg.x, newPos_msg.y};

			pos_t direction = targetPosition - robotPosition_;
			double distanceToTarget = direction.norm();
			if (distanceToTarget < navigationCutoff_) {
				currentPath_.erase(currentPath_.begin());
				continue;
			}

			direction /= distanceToTarget;
			double maxDistance = robotSpeed_ * dt;
			if (distanceToTarget > maxDistance) {
				robotPosition_ += direction * maxDistance;
				dt = 0;
			} else {
				robotPosition_ = targetPosition;
				dt -= distanceToTarget / robotSpeed_;
				currentPath_.erase(currentPath_.begin());
			}
		}
	}

	void Navigation::updateNavigationMap(const OccupancyMap& map) {
		navMapReady_ = true;
		navigationMap_ = map;
	}

	void Navigation::updatePath(plan_t newPath) {
		if (!hasGoal())
			return;

		skipPastWaypoints_(robotPosition_, newPath);
		double newPathCost;
		bool isNewPathValid = isPathValid_(newPath, newPathCost);

		if (!isNewPathValid)
			return;

		double oldPathCost;
		bool isOldPathValid = isPathValid_(currentPath_, oldPathCost);
		if (!isOldPathValid || newPathCost < oldPathCost)
			currentPath_ = newPath;
	}

	void Navigation::skipPastWaypoints_(const pos_t& position, plan_t& path) {
		unsigned int closestIdx = 0;
		double closestDist = std::numeric_limits<double>::infinity();
		auto dist =
			[&position] (const geometry_msgs::PoseStamped& waypoint_msg) {
				pos_t waypoint;
				waypoint.x() = waypoint_msg.pose.position.x;
				waypoint.y() = waypoint_msg.pose.position.y;
				return (waypoint - position).norm();
			};

		for (unsigned int i = 0; i < path.size(); i++) {
			double d = dist(path[i]);
			if (d < closestDist) {
				closestDist = d;
				closestIdx = i;
			}
		}

		if (closestDist < 1) {
			path.erase(path.begin(), path.begin() + closestIdx);
			while (!path.empty() && dist(path[0]) < navigationCutoff_) {
				path.erase(path.begin());
			}
		}
	}

	bool Navigation::isRobotAtGoal_() const {
		try {
			return (robotPosition_ - getCurrentGoal()).norm() < navigationCutoff_;
		} catch (std::runtime_error e) {
			std::stringstream ss;
			ss << "[Navigation::isRobotAtGoal_]" << e.what();
			throw std::runtime_error(ss.str());
		}
	}
	
	pos_t Navigation::getCurrentGoal() const {
		if (currentPlan_.empty())
			throw std::runtime_error("[Navigation::getCurrentGoal] No plan is active!");

		pos_t goal;
		goal.x() = currentPlan_[0].pose.position.x;
		goal.y() = currentPlan_[0].pose.position.y;
		return goal;
	}

	void Navigation::updatePlan(plan_t newPlan) {
		currentPlan_.clear();

		if (!navMapReady_)
			return;

		for (unsigned int i = 0; i < newPlan.size(); i++) {
			if (!navigationMap_.isInside(poseToPos(newPlan[i])))
				return;
		}
		currentPlan_ = newPlan;
	}

	bool Navigation::wasLastPlanSuccessful() const {
		return !planAborted_;
	}

	bool Navigation::isGoalValid_() const {
		try {
			return !(navigationMap_.isOccupied(getCurrentGoal()));
		} catch (std::runtime_error e) {
			std::stringstream ss;
			ss << "[Navigation::isGoalValid_]" << e.what();
			throw std::runtime_error(ss.str());
		}
	}

	bool Navigation::hasGoal() const {
		return !currentPlan_.empty();
	}

	pos_t Navigation::getRobotPosition() const {
		return robotPosition_;
	}

	bool Navigation::hasValidPath() const {
		double tmp;
		return isPathValid_(currentPath_, tmp);
	}

	bool Navigation::isStraightLineValidPath_(const pos_t& from, const pos_t& to) const {
		for (auto it = navigationMap_.getLineIterator(from, to); !it.isPastEnd(); ++it)
			if (navigationMap_.isOccupied(*it))
				return false;
		return true;
	}

	double Navigation::linearPathCost_(const pos_t& from, const pos_t& to) const {
		if (!isStraightLineValidPath_(from, to))
			return INF;

		return (to - from).norm();
	}

	double Navigation::pathCost_(const pos_t& robotPosition, const plan_t& path) const {
		if (path.empty())
			return INF;

		double cost = linearPathCost_(robotPosition, poseToPos(path[0]));
		for (unsigned int i = 0; i < path.size() - 1; i++)
			cost += linearPathCost_(poseToPos(path[i]), poseToPos(path[i+1]));
		return cost;
	}

	bool Navigation::isPathValid_(const plan_t& path, double& pathCostOut) const {
		pathCostOut = INF;

		if (path.empty())
			return false;

		pos_t endPos = poseToPos(path[path.size()-1]);
		double distToGoal;
		try {
			distToGoal = (getCurrentGoal() - endPos).norm();
		} catch (std::runtime_error e) {
			std::stringstream ss;
			ss << "[Navigation::isPathValid_]" << e.what();
			throw std::runtime_error(ss.str());
		}

		if (distToGoal > navigationCutoff_)
			return false;

		pathCostOut = pathCost_(robotPosition_, path);
		return pathCostOut < INF;
	}

	double Navigation::getCurrentPathLength() const {
		return pathCost_(robotPosition_, currentPath_);
	}

	bool Navigation::positionCollides_(const pos_t& pos) const {
		return truthMap_.isOccupied(pos) || navigationMap_.isOccupied(pos);
	}

	plan_t Navigation::getCurrentPlan() const {
		return currentPlan_;
	}

	plan_t Navigation::getCurrentPath() const {
		if (hasGoal() && hasValidPath())
			return currentPath_;
		else
			return {};
	}

	bool Navigation::isReady() const {
		return navMapReady_ && hasGoal();
	}

	void Navigation::resetPlan(bool success) {
		currentPlan_.clear();
		planAborted_ = !success;
	}
}

