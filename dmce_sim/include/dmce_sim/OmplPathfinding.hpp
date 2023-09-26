#pragma once

#include "dmce_core/OccupancyMap.hpp"
#include "dmce_sim/OmplAdapter.hpp"

namespace dmce {
	/**
	 * Pathfinding based on OMPL.
	 */
	class OmplPathfinding {
		OccupancyMap occMap_;
		costmap_2d::Costmap2D costMap_;

		bool isStraightLineValidPlan_(const pos_t& from, const pos_t& to) const;

	public:
		OmplPathfinding(const OccupancyMap& map);

		void updateMap(const OccupancyMap& map);

		std::pair<bool, plan_t> makePath(const pos_t& from, const pos_t& to);
	};
}
