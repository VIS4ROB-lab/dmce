#pragma once

#include "mre_dmcts_core/TypeDefs.hpp"

namespace mre_dmcts {
	/**
	 * Data structure representing the state of a single robot.
	 */
	struct MCRobotState {
		pos_t pos;
		double diameter;
	};
}
