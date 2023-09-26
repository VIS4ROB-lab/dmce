#pragma once

#include "dmce_sim/PathPlanner.hpp"

namespace dmce {
	/**
	 * Simplest path planner. Moves the robot along a straight line to the target.
	 */
	class LinearPathPlanner : public PathPlanner {

	public:
		using PathPlanner::PathPlanner;

		bool move(const double& timeStep, pos_t& position) override {
			auto targetPosition = getTarget();
			Eigen::Vector2d direction = targetPosition - position;
			double distance = direction.norm();
			if (distance > tolerance_) {
				direction /= distance;
				position += direction * speed_ * timeStep;
				return true;
			}
			return false;
		}

		bool makePlan(const pos_t& robotPosition) override {
			std::lock_guard<std::recursive_mutex> guard(planMutex_);
			plan_ = plan_t(2);
			plan_[0] = posToPose(robotPosition);
			plan_[1] = posToPose(getTarget());
			hasPlan_ = true;
			return true;
		}
	};
}
