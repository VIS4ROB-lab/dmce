#include "mre_dmcts_mcplanner/MCTree.hpp"

namespace mre_dmcts {
	std::pair<bool, MCNodePtr> MCTree::selection_(MCState& state, const double& idleValue) {
		// std::cout << "SELECTION" << std::endl;
		MCNodePtr selectedNode = rootNode_;
		selectedNode->getAction()->simulate(state, idleValue);

		while (!selectedNode->hasPotentialChildren()) {
			if (!selectedNode->hasChildren()) {
				return {false, selectedNode};
			}

			MCNodePtr bestChild = getBestChild_(
				selectedNode,
				[](const MCNodePtr& n) { return n->getUCB(); }
			);

			bool isFeasible = bestChild->getAction()->isFeasible(currentState_);
			if (isFeasible) {
				selectedNode = bestChild;
				selectedNode->getAction()->simulate(state, idleValue);
			} else {
				// ROS_INFO("[MCTree::selection_] Pruning infeasible node: %s", bestChild->getAction()->toString().c_str());
				selectedNode->pruneChild(bestChild);
			}
		}

		return {true, selectedNode};
	}


	std::pair<bool, MCNodePtr> MCTree::expansion_(MCState& state, MCNodePtr leafNode) {
		// std::cout << "EXPANSION" << std::endl;
		auto result = leafNode->expand(state, leafNode);
		return result;
	}

	double MCTree::rollout(MCState state, MCNodePtr newNode, const double& idleValue) {
		// std::cout << "ROLLOUT" << std::endl;
		unsigned int rolloutDepth = params_.rolloutDepth;
		MCActionGenerator actionGenerator;
		double value = newNode->getAction()->simulate(state, idleValue);
		for (unsigned int i = 0; i < rolloutDepth; i++) {
			auto potentialChildren = actionGenerator.generateFeasibleActions(state, params_);
			if (potentialChildren.size() == 0)
				return value;
			size_t idx = utils::randomIndex(potentialChildren.size());
			value = potentialChildren[idx]->simulate(state, idleValue);
		}
		return value;
	}

	void MCTree::backPropagation_(MCNodePtr startingNode, const double& value) {
		// std::cout << "BACKPROPAGATION" << std::endl;
		double curVal = value;
		startingNode->addVisit(value);
		MCNodePtr target = startingNode;
		while (!target->isRootNode()) {
			// curVal *= 0.95;
			target = target->parentNode;
			target->addVisit(curVal);
		}
	}

	std::pair<double, MCNodePtr> MCTree::iterate() {
		// std::cout << "iterate" << std::endl;
		MCState state = currentState_;
		double idleValue = 1 - currentState_.map.getRelativeEntropy();
		MCNodePtr selectedNode, newNode;
		bool selectionSuccess, expansionSuccess;
		std::tie(selectionSuccess, selectedNode) = selection_(state, idleValue);
		if (selectionSuccess) {
			std::tie(expansionSuccess, newNode) = expansion_(state, selectedNode);
			if (expansionSuccess) {
				double value = rollout(state, newNode, idleValue);
				backPropagation_(newNode, value);
			} else {
				// ROS_INFO("Expansion hit dead end at x=%.1f y=%.1f", state.robot.pos.x(), state.robot.pos.y());
			}
		} else {
			// ROS_INFO("Selection hit dead end at x=%.1f y=%.1f", state.robot.pos.x(), state.robot.pos.y());
		}

		if (!selectionSuccess || !expansionSuccess) {
			backPropagation_(selectedNode, -1);
			selectedNode->resetPotentialChildren(state);
			return {false, selectedNode};
		}
		return {true, newNode};
	}

	MCNodePtr MCTree::getBestChild_(
		const MCNodePtr& parent,
		double (*valueFcn)(const MCNodePtr&)
	) const {
		MCNodePtr bestChild = nullptr;
		double bestValue = -std::numeric_limits<double>::infinity();

		if (!parent->hasChildren()) {
			throw std::runtime_error("[MCTree::getBestChild_] Called on node with no children!");
		}

		auto it = parent->childNodes.begin();
		for (; it != parent->childNodes.end(); ++it) {
			MCNodePtr candidate = *it;
			double candidateValue = valueFcn(candidate);
			if (candidateValue > bestValue) {
				bestChild = candidate;
				bestValue = candidateValue;
			}
		}

		return bestChild;
	}

	MCPlan MCTree::getCurrentBestPlan() {
		MCPlan plan;
		MCNodePtr currentNode = rootNode_;
		while (currentNode->hasChildren()) {
			currentNode = getBestChild_(
				currentNode,
				[](const MCNodePtr& n) { return n->getEstimatedValue(); }
			);

			bool isFeasible = currentNode->getAction()->isFeasible(currentState_);
			if (!isFeasible) {
				// ROS_WARN("[MCTree::getCurrentBestPlan] Encountered infeasible node! Pruning: %s", currentNode->getAction()->toString().c_str());
				currentNode->parentNode->pruneChild(currentNode);
				return { };
			}

			plan.push_back(currentNode);
		}

		return plan;
	}

	unsigned int MCTree::size() const {
		return 1 + rootNode_->countDescendants();
	}

	MCNodePtr MCTree::getRoot() const {
		return rootNode_;
	}

	void printPlan(const MCPlan& plan) {
		std::stringstream msg;
		for (auto nodePtr : plan) {
			msg << nodePtr->getAction()->toString() << " | ";
		}
		ROS_INFO("Plan: %s", msg.str().c_str());
	}

	void MCTree::changeRoot(MCNodePtr newRoot) {
		rootNode_ = newRoot;
		newRoot->parentNode.reset();
		MCState stateCopy = currentState_;
		stateCopy.robot = rootNode_->getAction()->getFinalRobotState();
		rootNode_->resetPotentialChildren(stateCopy, true);
	}

	unsigned int MCTree::getNRollouts() const {
		return rootNode_->getNVisits();
	}
};

