#include "dmce_mcplanner/DMCTSPlanner.hpp"

namespace dmce {

	DMCTSPlanner::DMCTSPlanner(double robotDiameter, unsigned int robotId, MCParams params)
		: MCTSPlanner(robotDiameter, params)
	{
		params.useActionCaching = false;
		nRobots = utils::getRobotCount();
		for (size_t i = 0; i < nRobots; i++) {
			otherRobotPlans_.push_back(FiniteLog<plan_t>(params.planBufferSize));
		}
	}

	double DMCTSPlanner::simulatePlan_(const plan_t& plan, MCState& state) {
		if (plan.empty())
			return 0;

		double result = 0;
		double idleValue = 1 - state.map.getRelativeEntropy();
		MCActionPtr actionPtr;
		for (size_t i = 0; i < plan.size(); i++) {
			state.robot.pos.x() = plan[i].pose.position.x;
			state.robot.pos.y() = plan[i].pose.position.y;
			actionPtr = std::make_shared<DisplacementAction<NONE>>(state, params_);
			if (!actionPtr->isFeasible(state))
				return result;
			result = actionPtr->simulate(state, idleValue);
		}

		if (!tree_)
			return result;

		MCNodePtr startingNode = std::make_shared<MCTreeNode>(state, params_, actionPtr);
		tree_->rollout(state, startingNode, idleValue);
		return result;
	}

	MCState DMCTSPlanner::getCurrentState_() {
		MCState state = MCTSPlanner::getCurrentState_();
		MCRobotState robotState = state.robot;
		for (size_t i = 0; i < otherRobotPlans_.size(); i++) {
			if (!otherRobotPlans_[i].empty())
				simulatePlan_(otherRobotPlans_[i].randomItem(), state);
		}
		state.robot = robotState;
		return state;
	}

	std::pair<bool, plan_t> DMCTSPlanner::getPlanToShare_() {
		auto failure = std::make_pair(false, plan_t{});
		auto mcPlan = tree_->getCurrentBestPlan();
		if (mcPlan.empty())
			return failure;

		plan_t plan = convertMCPlanToPoses_(mcPlan);
		return std::make_pair(true, plan);
	}

	void DMCTSPlanner::peerPlanCallback(const dmce_msgs::RobotPlan& msg) {
		size_t idx = msg.robotId - 1;
		otherRobotPlans_[idx].push(msg.path.poses);
	}
}

