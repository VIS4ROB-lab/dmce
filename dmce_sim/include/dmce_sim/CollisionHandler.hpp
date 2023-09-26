#pragma once

#include "dmce_sim/GroundTruthMap.hpp"

namespace dmce {
	class CollisionHandler {
		const GroundTruthMap map_;
		const double robotRadius_;
		const double obstacleCellRadius_;
		const unsigned int max_iter_ = 100;

		bool getClosestCollidingObstacle_(
			const grid_map::Position& robotPos,
			grid_map::Position closestPos,
			double& minDist) const
		{
			minDist = std::numeric_limits<double>::infinity();
			auto it = map_.getCircleIterator(robotPos, robotRadius_);
			for (; !it.isPastEnd(); ++it) {
				if (map_.isOccupied(*it)) {
					grid_map::Position pos;
					map_.getPosition(*it, pos);
					double dist = (robotPos - pos).norm();
					minDist = std::min(minDist, dist);
					map_.getPosition(*it, closestPos);
				}
			}
			return minDist < robotRadius_;
		}

	public:
		CollisionHandler(
			const GroundTruthMap map,
			double robotDiameter = 1,
			double mapCellSize = 1)
			: map_(map),
			  robotRadius_(robotDiameter/2),
			  obstacleCellRadius_(mapCellSize/2)
		{

		}

		bool collides(const grid_map::Position& robotPosition) const {
			double minDistSq = std::numeric_limits<double>::infinity();
			auto it = map_.getCircleIterator(robotPosition, robotRadius_);
			for (; !it.isPastEnd(); ++it) {
				if (map_.isOccupied(*it)) {
					return true;
				}
			}
			return false;
		}

		bool handleCollisions(grid_map::Position& robotPosition) const {
			bool moved = false;

			double minDist = std::numeric_limits<double>::infinity();
			grid_map::Position closestPos{0,0};
			unsigned int iters = 0;

			while (++iters < max_iter_
				&& getClosestCollidingObstacle_(robotPosition, closestPos, minDist))
			{
				moved = true;
				Eigen::Vector2d dir = (robotPosition - closestPos).normalized();
				robotPosition += dir * (robotRadius_ - minDist + obstacleCellRadius_);
			}

			return moved;
		}
	};
};

