#pragma once

#include "Planner.hpp"

namespace dmce {

	/**
	 * This action planner simply returns the closest frontier cell.
	 */
	class FrontierPlanner : public Planner {
		plan_t latestPlan_;
	public:
		using Planner::Planner;

	protected:
		std::pair<bool, plan_t> getLatestPlan_() override {
			auto plan = latestPlan_;
			latestPlan_.clear();
			return std::make_pair((plan.size() > 0), plan);
		}

		void updatePlan_() override {
			auto map = getMap();
			auto res = map.getResolution();
			auto frontier = map.getFrontier();
			pos_t robotPos = getPosition();
			plan_t plan;

			if (frontier.size() > 0) {
				pos_t candidatePos, closestFrontier;
				map.getPosition(frontier[0], closestFrontier);
				double minDist = (closestFrontier - robotPos).squaredNorm();
				for (unsigned i = 1; i < frontier.size(); i++) {
					map.getPosition(frontier[i], candidatePos);
					double dist = (candidatePos - robotPos).squaredNorm();
					if (dist < minDist) {
						minDist = dist;
						closestFrontier = candidatePos;
					}
				}
				plan.push_back(posToPose(closestFrontier));
			}

			latestPlan_ = plan;
		}
	};
}
