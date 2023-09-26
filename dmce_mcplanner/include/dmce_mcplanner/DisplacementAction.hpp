#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include <Eigen/Core>

#include "dmce_core/utils.hpp"
#include "dmce_core/TypeDefs.hpp"
#include "dmce_core/RobotMap.hpp"
#include "dmce_mcplanner/CachedMCAction.hpp"

namespace dmce {
	enum Displacement {
		NORTH, EAST, SOUTH, WEST, NONE, RANDOM
	};

	/**
	 * This class represents a simple 1m translation
	 * along one of the cartesian directions.
	 */
	template<Displacement displacement, int multiplier = 1>
	class DisplacementAction : public CachedMCAction {
		const pos_t targetPosition_ = {10, -10};
		Eigen::Vector2d delta_ = {0, 0};
		bool infeasibleFlag_ = false;

		hashKey_t hash_;
		void initHash_();

		template<typename Iterator>
		bool noCollision_(const OccupancyMap& map, Iterator it) const;

		bool isFeasible_(const MCState& state) const override;

	public:
		DisplacementAction(const MCState& initialState, const MCParams& params, const std::vector<MCActionPtr>& otherActions = {}, const MCActionPtr parent = nullptr);

		MCRobotState getFinalRobotState() const override;
		std::string getName() const;
		std::string toString() const override;
		hashKey_t getHash() const override;

		static double getDisplacementAmount() {
			return 2 * multiplier;
		}
	};

	using NoDisplacement = DisplacementAction<Displacement::NONE>;

	using NorthDisplacement = DisplacementAction<Displacement::NORTH>;
	using SouthDisplacement = DisplacementAction<Displacement::SOUTH>;
	using EastDisplacement = DisplacementAction<Displacement::EAST>;
	using WestDisplacement = DisplacementAction<Displacement::WEST>;
	using RandomDisplacement = DisplacementAction<Displacement::RANDOM>;

	using BigNorthDisplacement = DisplacementAction<Displacement::NORTH, 5>;
	using BigSouthDisplacement = DisplacementAction<Displacement::SOUTH, 5>;
	using BigEastDisplacement = DisplacementAction<Displacement::EAST, 5>;
	using BigWestDisplacement = DisplacementAction<Displacement::WEST, 5>;
}
