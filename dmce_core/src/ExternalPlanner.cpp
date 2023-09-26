
#include "dmce_core/ExternalPlanner.hpp"

namespace dmce {
	ExternalPlanner::ExternalPlanner(double robotDiameter, unsigned int robotId, std::string plannerType)
		: Planner(robotDiameter), robotId_(robotId)
	{
		ros::NodeHandle nh;
		rrtPlanSubscriber_ = nh.subscribe(plannerType + "_plan", 1, &ExternalPlanner::planCallback_, this);
	}

	std::pair<bool, plan_t> ExternalPlanner::getLatestPlan_() {
		auto plan = latestPlan_;
		latestPlan_.clear();
		return std::make_pair((plan.size() > 0), plan);
	}

	void ExternalPlanner::updatePlan_() {

	}

	void ExternalPlanner::planCallback_(const dmce_msgs::RobotPlan& planMsg) {
		latestPlan_ = planMsg.path.poses;
	}

	void ExternalPlanner::signalNavigationFailure() {
		latestPlan_.clear();
	}
}
