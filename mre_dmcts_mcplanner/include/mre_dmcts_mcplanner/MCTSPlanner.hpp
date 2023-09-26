#pragma once

#include "mre_dmcts_core/Planner.hpp"
#include "mre_dmcts_core/ClusteredFrontierPlanner.hpp"
#include "mre_dmcts_core/FiniteLog.hpp"
#include "mre_dmcts_mcplanner/MCParams.hpp"
#include "mre_dmcts_mcplanner/MCTree.hpp"

namespace mre_dmcts {

	/**
	 * This action planner runs MCTS and returns the current best plan.
	 */
	class MCTSPlanner : public Planner {
		FiniteLog<double> latestTimings_;
		static const size_t timingsLogSize_ = 100;
		ros::Time timeOfLastUpdate_;
		bool outputUpdateRate_ = false;
		FrontierClustering clusterHandler_;

		unsigned int consecutiveIterationFailures_ = 0;
		bool revertToFallback_ = false;

	protected:
		std::unique_ptr<MCTree> tree_;
		MCActionPtr initialAction_;
		bool firstUpdate_ = true;
		MCParams params_;

		/**
		 * Construct a new MCTS tree at the given starting position.
		 */
		std::unique_ptr<MCTree> buildTree_(MCState state);

		/**
		 * Convert a plan composed of MCTS nodes to
		 * a sequence of positions.
		 */
		plan_t convertMCPlanToPoses_(const MCPlan& mcPlan) const;

		/**
		 * Return the current combined world and robot state.
		 */
		virtual MCState getCurrentState_();

		/**
		 * Get the final position of the root node of the MCTS tree.
		 */
		pos_t getRootPosition_() const;

		/**
		 * Called by the planner server in the update loop.
		 * Performs one MCTS iteration.
		 */
		virtual void updatePlan_() override;

		/**
		 * Return the current best plan based on the
		 * current world and tree state.
		 * result.first is false on failure.
		 */
		std::pair<bool, plan_t> getLatestPlan_() override;

		/**
		 * Returns a fallback plan generated with
		 * a different planner.
		 */
		std::pair<bool, plan_t> goToFallbackPlan_();

	public:
		MCTSPlanner(double robotDiameter, MCParams params = MCParams{1,1,1,1,1,1})
			: Planner(robotDiameter), params_(params),
			  latestTimings_(timingsLogSize_), timeOfLastUpdate_(0)
		{ }

		/**
		 * Reset and reinitialise the search tree.
		 */
		void resetTree(const MCState& state);

		/**
		 * Signal called by planner server.
		 * Resets the search tree.
		 */
		virtual void signalNavigationFailure() override {
			resetTree(getCurrentState_());
		}

		void frontierClusterCallback(const mre_dmcts_msgs::FrontierClusters& msg) override {
			clusterHandler_.fromMessage(msg);
		}
	};
}
