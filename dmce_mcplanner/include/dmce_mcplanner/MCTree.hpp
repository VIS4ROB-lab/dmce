#pragma once

#include "dmce_mcplanner/MCTreeNode.hpp"
#include "dmce_mcplanner/MCActionGenerator.hpp"
#include "dmce_core/TypeDefs.hpp"
#include "dmce_core/utils.hpp"

namespace dmce {

	using MCPlan = std::list<MCNodePtr>;

	/**
	 * Utility function. Print a representation of a given
	 * MCTS plan (node sequence) to stdout.
	 */
	void printPlan(const MCPlan& plan);

	/**
	 * A representation of the Monte Carlo Search Tree.
	 */
	class MCTree {
	public:
		MCTree(MCState initialState, MCNodePtr rootNode, const MCParams& params)
			: currentState_(initialState), rootNode_(rootNode), params_(params)
		{ }

		/**
		 * Update the current world state.
		 */
		void updateState(MCState newState) {
			currentState_ = newState;
		}

		/**
		 * Perform an iteration of the MCTS algorithm.
		 */
		std::pair<double, MCNodePtr> iterate();

		/**
		 * Returns the current best plan as a sequence of tree nodes.
		 */
		MCPlan getCurrentBestPlan();

		/**
		 * Counts the number of nodes in the tree.
		 */
		unsigned int size() const;

		/**
		 * Returns a shared pointer to the root node of the tree.
		 */
		MCNodePtr getRoot() const;

		/**
		 * Change the root of the tree to an existing node in the tree,
		 * keeping the nodes below it and discarding the rest of the tree.
		 */
		void changeRoot(MCNodePtr newRoot);

		/**
		 * Rollout step of the MCTS algorithm.
		 */
		double rollout(MCState state, MCNodePtr newNode, const double& idleValue);

		/**
		 * Get the number of rollouts that have been performed on the tree.
		 */
		unsigned int getNRollouts() const;

	private:

		MCActionGenerator actionGenerator_;
		MCNodePtr rootNode_;
		MCState currentState_;
		const MCParams& params_;

		/**
		 * Selection step of the MCTS algorithm.
		 */
		std::pair<bool, MCNodePtr> selection_(MCState& state, const double& idleValue);

		/**
		 * Expansion step of the MCTS algorithm.
		 */
		std::pair<bool, MCNodePtr> expansion_(MCState& state, MCNodePtr leafNode);

		/**
		 * Backpropagation step of the MCTS algorithm.
		 */
		void backPropagation_(MCNodePtr startingNode, const double& value);

		/**
		 * Returns the child of the given node with the highest estimated value.
		 */
		MCNodePtr getBestChild_(const MCNodePtr& parent, double (*valueFcn)(const MCNodePtr&)) const;
	};
}
