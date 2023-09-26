#pragma once

#include "dmce_core/RobotMap.hpp"
#include "dmce_core/FrontierClustering.hpp"
#include "dmce_mcplanner/MCRobotState.hpp"

namespace dmce {
	/**
	 * Data structure representing a world state
	 * in the Monte Carlo Tree Search.
	 */
	struct MCState {
		MCRobotState robot;
		RobotMap map;
		FrontierClustering clusters;
	};
}
