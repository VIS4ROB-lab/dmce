#pragma once

#include "dmce_core/TypeDefs.hpp"

namespace dmce {
	/**
	 * Data structure representing the state of a single robot.
	 */
	struct MCRobotState {
		pos_t pos;
		double diameter;
	};
}
