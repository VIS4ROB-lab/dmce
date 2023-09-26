#include "mre_dmcts_mcplanner/MCAction.hpp"

namespace mre_dmcts {

	double MCAction::simulate_(MCState& state, const double& idleValue) const {
		unsigned int rayCount_ = params_.robotLidarRayCount;
		double sensorRange_ = params_.robotSensorRange;

		double angleIncrement = 2 * M_PI / rayCount_;
		double actualRange = sensorRange_ - state.map.getResolution();
		for (unsigned int i = 0; i < rayCount_; i++) {
			double angle = i * angleIncrement;
			auto rayTarget = state.robot.pos;
			rayTarget.x() += std::cos(angle) * actualRange;
			rayTarget.y() += std::sin(angle) * actualRange;
			castRay_(state.robot.pos, rayTarget, state.map);
		}

		double value = 1 - state.map.getRelativeEntropy();
		if (params_.useLocalReward)
			return value - idleValue;
		else
			return value;
	}

	bool MCAction::castRay_(const pos_t& from, const pos_t& to, RobotMap& map) {
		try {
			auto it = map.getLineIterator(from, to);
			grid_map::Position cellPos;
			bool stop = false;
			if (it.isPastEnd()) {
				return false;
			}
			do {
				map.getPosition(*it, cellPos);
				bool isOccupied = map.isOccupied(*it);
				if (!isOccupied)
					map.setOccupancy(*it, map.freeValue);
				++it;
				stop = isOccupied || it.isPastEnd();
			} while (!stop);
		} catch (std::invalid_argument e) {
			return false;
		}
		return true;
	}
}
