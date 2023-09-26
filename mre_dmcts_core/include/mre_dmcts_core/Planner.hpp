#pragma once

#include "mre_dmcts_core/TypeDefs.hpp"
#include "mre_dmcts_core/RobotMap.hpp"
#include "mre_dmcts_core/FrontierClustering.hpp"
#include "mre_dmcts_msgs/RobotPlan.h"

namespace mre_dmcts {

	class Planner {
		RobotMap map_;
		bool hasMap_;

		pos_t pos_;
		bool hasPosition_;

		bool isReady_() const {
			return hasMap_ && hasPosition_;
		}

	protected:
		/**
		 * Called as part of the main loop.
		 */
		virtual void updatePlan_() = 0;

		/**
		 * Return the latest valid plan.
		 */
		virtual std::pair<bool, plan_t> getLatestPlan_() = 0;

		/**
		 * Return a plan to be shared with peer robots.
		 */
		virtual std::pair<bool, plan_t> getPlanToShare_() {
			return std::make_pair(false, plan_t{});
		}

		const double robotDiameter_;

	public:
		Planner(const double& robotDiameter)
			: hasMap_(false), hasPosition_(false), robotDiameter_(robotDiameter)
		{ }

		void updatePlan() {
			if (!isReady_())
				return;
			updatePlan_();
		}

		RobotMap getMap() const {
			return map_;
		}

		void setMap(const RobotMap& map) {
			hasMap_ = true;
			map_ = map;
		}

		pos_t getPosition() const {
			return pos_;
		}

		void setPosition(const pos_t& position) {
			hasPosition_ = true;
			pos_ = position;
		}

		std::pair<bool, plan_t> getLatestPlan() {
			if (!isReady_())
				return std::make_pair(false, plan_t{});
			return getLatestPlan_();
		}

		std::pair<bool, plan_t> getPlanToShare() {
			if (!isReady_())
				return std::make_pair(false, plan_t{});
			return getPlanToShare_();
		}

		/**
		 * Communicates to the planner that there was a failure
		 * in the navigation server when attempting to realise the
		 * last global plan provided.
		 */
		virtual void signalNavigationFailure() {

		}

		/**
		 * Called by PlannerServer whenever a plan is received from a peer.
		 */
		virtual void peerPlanCallback(const mre_dmcts_msgs::RobotPlan& msg) {

		}

		/**
		 * Called by PlannerServer whenever updated fontier clusters are received.
		 */
		virtual void frontierClusterCallback(const mre_dmcts_msgs::FrontierClusters& msg) {

		}
	};
}
