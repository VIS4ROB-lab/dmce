#pragma once

#include "mre_dmcts_mcplanner/MCTSPlanner.hpp"
#include "mre_dmcts_core/utils.hpp"
#include "mre_dmcts_msgs/RobotPlan.h"
#include "nav_msgs/Path.h"

namespace mre_dmcts {
	class DMCTSPlanner : public MCTSPlanner {
		unsigned int nRobots;
		std::vector<FiniteLog<plan_t>> otherRobotPlans_;

	public:
		DMCTSPlanner(double robotDiameter, unsigned int robotId, MCParams params = MCParams{1,1,1,1,1,1});

	protected:
		virtual MCState getCurrentState_() override;

		double simulatePlan_(const plan_t& plan, MCState& state);

		virtual std::pair<bool, plan_t> getPlanToShare_() override;

		virtual void peerPlanCallback(const mre_dmcts_msgs::RobotPlan& msg) override;
	};
}
