#pragma once

#include "dmce_mcplanner/MCTSPlanner.hpp"
#include "dmce_core/utils.hpp"
#include "dmce_msgs/RobotPlan.h"
#include "nav_msgs/Path.h"

namespace dmce {
	class DMCTSPlanner : public MCTSPlanner {
		unsigned int nRobots;
		std::vector<FiniteLog<plan_t>> otherRobotPlans_;

	public:
		DMCTSPlanner(double robotDiameter, unsigned int robotId, MCParams params = MCParams{1,1,1,1,1,1});

	protected:
		virtual MCState getCurrentState_() override;

		double simulatePlan_(const plan_t& plan, MCState& state);

		virtual std::pair<bool, plan_t> getPlanToShare_() override;

		virtual void peerPlanCallback(const dmce_msgs::RobotPlan& msg) override;
	};
}
