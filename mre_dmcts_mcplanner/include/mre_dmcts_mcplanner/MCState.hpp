#pragma once

#include "mre_dmcts_core/RobotMap.hpp"
#include "mre_dmcts_core/FrontierClustering.hpp"
#include "mre_dmcts_mcplanner/MCRobotState.hpp"

namespace mre_dmcts {
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
