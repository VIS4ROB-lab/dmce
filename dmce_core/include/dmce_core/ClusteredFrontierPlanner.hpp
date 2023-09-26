#pragma once

#include "dmce_core/Planner.hpp"
#include "dmce_core/FrontierClustering.hpp"

namespace dmce {

	/**
	 * This planner simply returns the closest frontier cluster centroid.
	 */
	class ClusteredFrontierPlanner : public Planner {
		plan_t latestPlan_;
		FrontierClustering clusterHandler_;

	public:
		using Planner::Planner;

		void frontierClusterCallback(const dmce_msgs::FrontierClusters& msg) override {
			clusterHandler_.fromMessage(msg);
		}

	protected:
		std::pair<bool, plan_t> getLatestPlan_() override {
			auto plan = latestPlan_;
			latestPlan_.clear();
			return std::make_pair((plan.size() > 0), plan);
		}

		void updatePlan_() override {
			auto centroids = clusterHandler_.getCentroidPositions();
			pos_t robotPos = getPosition();
			plan_t plan{};

			if (!centroids.empty()) {
				pos_t closestCentroid = centroids[0];
				double minDist = (centroids[0] - robotPos).squaredNorm();
				for (unsigned i = 1; i < centroids.size(); i++) {
					double dist = (centroids[i] - robotPos).squaredNorm();
					if (dist < minDist) {
						minDist = dist;
						closestCentroid = centroids[i];
					}
				}
				plan.push_back(posToPose(closestCentroid));
			}

			latestPlan_ = plan;
		}
	};
}
