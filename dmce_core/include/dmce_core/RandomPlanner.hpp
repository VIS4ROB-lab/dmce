#pragma once

#include "Planner.hpp"

namespace dmce {

	/**
	 * This action planner simply returns a random point on the map.
	 */
	class RandomPlanner : public Planner {
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
			Eigen::Vector2d mapSize = map.getLength();
			Eigen::Vector2d mapPos = map.getPosition();
			Eigen::Vector2d rand = Eigen::Vector2d::Random();
			pos_t target = mapPos - 0.5*mapSize.cwiseProduct(rand);
			plan_t plan;
			plan.push_back(posToPose(target));
			latestPlan_ = plan;
		}
	};
}
