#include "dmce_mcplanner/MCTreeNode.hpp"
#include "dmce_mcplanner/MCActionGenerator.hpp"

namespace dmce {
	MCTreeNode::MCTreeNode(
			const MCState& initialState,
			const MCParams& params,
			MCActionPtr action,
			MCNodePtr parentNode
		) : action_(action), parentNode(parentNode), params_(params), explorationFactor(params.explorationFactor)
	{
		// std::cout << "[MCTreeNode] InitialPos x: " << initialState.robot.pos.x() << " y: " << initialState.robot.pos.y() << std::endl;
		resetPotentialChildren(initialState, parentNode == nullptr);
	}

	void MCTreeNode::resetPotentialChildren(const MCState& initialState, bool useFrontiers) {
		potentialChildren_.clear();
		MCState finalState = initialState;
		finalState.robot = getFinalRobotState();
		MCActionGenerator generator;
		potentialChildren_ = generator.generateFeasibleActions(
				finalState,
				params_,
				parentNode,
				useFrontiers
			);

		std::shuffle(
			potentialChildren_.begin(),
			potentialChildren_.end(),
			utils::RNG::get()
		);
	}
	
	void MCTreeNode::addVisit(double visitValue) {
		double gamma = params_.iterationDiscountFactor;
		discountedValue_ = discountedValue_*gamma + visitValue;
		discountedVisits_ = discountedVisits_*gamma + 1;
		nVisits_++;
	}

	double MCTreeNode::getEstimatedValue() const {
		if (discountedVisits_ == 0) return 0;
		else {
			double expectation = discountedValue_ / discountedVisits_;
			double duration = action_->getDuration();
			double timeDiscount = std::min(1., std::pow(params_.timeDiscountFactor, duration));
			return timeDiscount * expectation;
		}
	}

	double MCTreeNode::getDiscountedVisits() const {
		return discountedVisits_;
	}

	double MCTreeNode::getUCB() const {
		if (isRootNode() || discountedVisits_ == 0)
			return 0;
		return getEstimatedValue() + explorationFactor * std::sqrt(std::log(parentNode->getDiscountedVisits()) / discountedVisits_);
	}

	bool MCTreeNode::isRootNode() const {
		return parentNode == nullptr;
	}

	bool MCTreeNode::isExpandable(MCState& refState) {
		MCState initialState = refState;
		initialState.robot = getFinalRobotState();

		for (auto it = potentialChildren_.begin(); it != potentialChildren_.end(); ) {
			if (!(*it)->isFeasible(initialState)) {
				it = potentialChildren_.erase(it);
			} else {
				++it;
			}
		}

		return potentialChildren_.size() > 0;
	}

	std::pair<bool, MCNodePtr> MCTreeNode::expand(MCState& refState, MCNodePtr parentPtr) {
		MCState initialState = refState;
		initialState.robot = getFinalRobotState();

		if (!isExpandable(refState)) {
			return {false, nullptr};
		}

		MCNodePtr newNode = std::make_shared<MCTreeNode>(initialState, params_, potentialChildren_.back(), parentPtr);
		childNodes.push_back(newNode);
		potentialChildren_.pop_back();
		return {true, newNode};
	}

	MCRobotState MCTreeNode::getFinalRobotState() const {
		return action_->getFinalRobotState();
	}

	MCRobotState MCTreeNode::getInitialRobotState() const {
		return action_->getInitialRobotState();
	}

	unsigned int MCTreeNode::countDescendants() const {
		unsigned int descendantCount = 0;
		auto it = childNodes.begin();
		for ( ; it != childNodes.end(); ++it) {
			descendantCount += 1 + (*it)->countDescendants();
		}
		return descendantCount;
	}

	MCActionPtr MCTreeNode::getAction() const {
		return action_;
	}

	void MCTreeNode::pruneChild(MCNodePtr& child) {
		auto it = childNodes.begin();
		for ( ; it != childNodes.end(); ) {
			if (*it == child) {
				it = childNodes.erase(it);
			} else {
				++it;
			}
		}
		child.reset();
	}

	unsigned int MCTreeNode::getNVisits() const {
		return nVisits_;
	}
}

