#pragma once

#include "dmce_msgs/RobotPlan.h"
#include "Planner.hpp"

namespace dmce {

	/**
	 * This action planner listens for the plans published by an external planner
	 * and makes them available to the PlannerServer.
	 * It is used for the baselines (MMPF and RRT).
	 */
	class ExternalPlanner : public Planner {
		unsigned int robotId_;
		plan_t latestPlan_;
		ros::Subscriber rrtPlanSubscriber_;

	public:
		ExternalPlanner(double robotDiameter, unsigned int robotId, std::string plannerType);

		void signalNavigationFailure() override;

	protected:
		std::pair<bool, plan_t> getLatestPlan_() override;

		void updatePlan_() override;

	private:

		void planCallback_(const dmce_msgs::RobotPlan& planMsg);
	};
}
