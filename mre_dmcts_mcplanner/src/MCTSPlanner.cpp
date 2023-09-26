#include "mre_dmcts_mcplanner/MCTSPlanner.hpp"

namespace mre_dmcts {

	std::unique_ptr<MCTree> MCTSPlanner::buildTree_(MCState state) {
		initialAction_ = std::make_shared<DisplacementAction<NONE>>(state, params_);
		MCNodePtr rootNode = std::make_shared<MCTreeNode>(state, params_, initialAction_);
		return std::make_unique<MCTree>(state, rootNode, params_);
	}

	void MCTSPlanner::resetTree(const MCState& state) {
		tree_ = buildTree_(state);
	}

	plan_t MCTSPlanner::convertMCPlanToPoses_(const MCPlan& mcPlan) const {
		plan_t plan;
		pos_t rootInitialPos = (*mcPlan.begin())->getAction()->getInitialRobotState().pos;
		plan.push_back(posToPose(rootInitialPos));
		for (auto node_ptr : mcPlan) {
			plan.push_back(posToPose(node_ptr->getAction()->getFinalRobotState().pos));
		}
		return plan;
	}

	MCState MCTSPlanner::getCurrentState_() {
		return {getPosition(), robotDiameter_, getMap(), clusterHandler_};
	}

	pos_t MCTSPlanner::getRootPosition_() const {
		return tree_->getRoot()->getFinalRobotState().pos;
	}

	std::pair<bool, plan_t> MCTSPlanner::getLatestPlan_() {
		auto failure = std::make_pair(false, plan_t{});

		if (revertToFallback_) {
			return goToFallbackPlan_();
		}

		if (!tree_ || tree_->getNRollouts() < params_.minRollouts)
			return failure;

		MCState currentState = getCurrentState_();
		MCPlan mcPlan;

		double sqDistFromRoot = (currentState.robot.pos - getRootPosition_()).squaredNorm();
		bool reset = sqDistFromRoot > params_.navigationCutoff*params_.navigationCutoff;
		if (reset) {
			ROS_WARN("[MCTSPlanner::getLatestPlan_] Resetting due to distance from root!");
			resetTree(currentState);
			return failure;
		}

		mcPlan = tree_->getCurrentBestPlan();
		if (mcPlan.size() < params_.minPlanDepth) {
			return failure;
		}

		if (params_.minPlanValue > 0) {
			double initialEntropy = currentState.map.getRelativeEntropy();
			MCState simState = currentState;
			for (MCNodePtr& np : mcPlan) {
				np->getAction()->simulate(simState, 1-initialEntropy);
			}
			double finalEntropy = simState.map.getRelativeEntropy();
			double planValue = (initialEntropy - finalEntropy);
			if (planValue < params_.minPlanValue) {
				return goToFallbackPlan_();
			}
		}

		if (params_.reuseBranches)
			tree_->changeRoot(mcPlan.front());
		else {
			MCState state = getCurrentState_();
			state.robot = mcPlan.front()->getFinalRobotState();
			resetTree(state);
		}
		plan_t posPlan = convertMCPlanToPoses_(mcPlan);
		return std::make_pair(true, posPlan);
	}

	std::pair<bool, plan_t> MCTSPlanner::goToFallbackPlan_() {
		ClusteredFrontierPlanner frontierPlanner(robotDiameter_);
		frontierPlanner.setPosition(getPosition());
		FrontierClustering fc;
		fc.fromMapFrontiers(getMap());
		frontierPlanner.frontierClusterCallback(fc.getMessage());
		frontierPlanner.setMap(getMap());
		frontierPlanner.updatePlan();
		auto frontierPlan = frontierPlanner.getLatestPlan();
		if (frontierPlan.first) {
			auto newState = getCurrentState_();
			auto newPos = frontierPlan.second.front().pose.position;
			newState.robot.pos = {newPos.x, newPos.y};
			resetTree(newState);
			ROS_WARN("[MCTSPlanner] Fell back on ClusteredFrontierPlanner.");
		} else
			ROS_ERROR("[MCTSPlanner] ClusteredFrontierPlanner fallback failed!");
		return frontierPlan;
	}

	void MCTSPlanner::updatePlan_() {
		MCState currentState = getCurrentState_();

		if (firstUpdate_) {
			firstUpdate_ = false;
			tree_ = buildTree_(currentState);
		} else {
			tree_->updateState(currentState);
		}

		{
			bool iterationSuccess;
			MCNodePtr node;
			std::tie(iterationSuccess, node) = tree_->iterate();
			if (!iterationSuccess) {
				++consecutiveIterationFailures_;
				if (consecutiveIterationFailures_ >= 25) {
					ROS_WARN("[MCTSPlanner::updatePlan] Resetting tree due to %u consecutive iteration failures.", consecutiveIterationFailures_);
					resetTree(currentState);
					consecutiveIterationFailures_ = 0;
					revertToFallback_ = true;
				}
			} else {
				consecutiveIterationFailures_ = 0;
				revertToFallback_ = false;
			}
		}

		if (outputUpdateRate_) {
			ros::Time curTime = ros::Time::now();
			latestTimings_.push((curTime - timeOfLastUpdate_).toSec());
			timeOfLastUpdate_ = curTime;
			double timingSum = 0;
			latestTimings_.reduce<double>(timingSum,
				[](double& r, const double& e) {
					r += e;
				});
			double avgTime = timingSum / latestTimings_.size();
			ROS_INFO("[MCTSPlanner::updatePlan] Update rate: %.2f Hz", 1/avgTime);
		}
	}
}


