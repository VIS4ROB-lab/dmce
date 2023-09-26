#pragma once

#include "MCAction.hpp"
#include "MCState.hpp"
#include <map>

namespace dmce {
	class MCTreeNode;

	using MCNodePtr = std::shared_ptr<MCTreeNode>;

	class MCTreeNode {
		double explorationFactor;

		std::vector<MCActionPtr> potentialChildren_;
		MCActionPtr action_ = nullptr;
		unsigned int nVisits_ = 0;
		double discountedVisits_ = 0;
		double discountedValue_ = 0;
		const MCParams& params_;

	public:

		MCNodePtr parentNode = nullptr;
		std::list<MCNodePtr> childNodes;

		MCTreeNode(
			const MCState& initialState,
			const MCParams& params,
			MCActionPtr action = nullptr,
			MCNodePtr parentNode = nullptr
		);

		/**
		 * Increments the visit counter and updates the
		 * estimated value.
		 */
		void addVisit(double visitValue);
		
		/**
		 * Returns the estimated value for this node.
		 */
		double getEstimatedValue() const;

		/**
		 * Returns the number of times this node has been visited.
		 */
		double getDiscountedVisits() const;

		/**
		 * Returns the current UCB value for this node.
		 */
		double getUCB() const;

		/**
		 * Returns true if this node is a root (has no parent).
		 */
		bool isRootNode() const;

		/**
		 * Returns true if there is at least one potential child
		 * that can be expanded from the given world state.
		 * Infeasible children are removed from the list of
		 * potential children.
		 */
		bool isExpandable(MCState& refState);

		/**
		 * Returns true if this node has at least one expanded child.
		 */
		bool hasChildren() const {
			return childNodes.size() > 0;
		}

		/**
		 * Returns true if this node has one or more feasible un-expanded children.
		 */
		bool hasPotentialChildren() const {
			return potentialChildren_.size() > 0;
		}

		/**
		 * If this node is a leaf node, pick one of the unexpanded children
		 * at random and expand it.
		 * @param initialState The state to be used for the expansion
		 * @param parentPtr A shared pointer to THIS node, to be passed to the child.
		 * return.first is true on success, false if this node is not a leaf.
		 * return.second is the created node, if one was created; nullptr otherwise.
		 */
		std::pair<bool, MCNodePtr> expand(MCState& initialState, MCNodePtr parentPtr);

		/**
		 * Returns the position resulting from performing the action of this node
		 * on the given initial state.
		 */
		MCRobotState getFinalRobotState() const;

		/**
		 * Returns the initial robot state associated with this node.
		 */
		MCRobotState getInitialRobotState() const;

		/**
		 * Count the number of descendants this node has.
		 */
		unsigned int countDescendants() const;

		/**
		 * Get a pointer to this node's action.
		 */
		MCActionPtr getAction() const;

		/**
		 * Remove the given child from the search tree.
		 */
		void pruneChild(MCNodePtr& child);

		/**
		 * Re-evaluates the set of potential children.
		 */
		void resetPotentialChildren(const MCState& initialState, bool useFrontiers = false);

		/**
		 * Returns the number of visits to this node.
		 */
		unsigned int getNVisits() const;
	};
};

