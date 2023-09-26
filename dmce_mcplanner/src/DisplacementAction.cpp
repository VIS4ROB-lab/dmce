#include "dmce_mcplanner/DisplacementAction.hpp"

namespace dmce {

	template<Displacement displacement, int multiplier>
	DisplacementAction<displacement, multiplier>::DisplacementAction(
			const MCState& initialState,
			const MCParams& params,
			const std::vector<MCActionPtr>& otherActions,
			const MCActionPtr parent
		) : CachedMCAction(initialState, params)
	{
		if constexpr (displacement == Displacement::NORTH)
			delta_ = {0, getDisplacementAmount()};
		else if constexpr (displacement == Displacement::SOUTH)
			delta_ = {0, -getDisplacementAmount()};
		else if constexpr (displacement == Displacement::EAST)
			delta_ = {getDisplacementAmount(), 0};
		else if constexpr (displacement == Displacement::WEST)
			delta_ = {-getDisplacementAmount(), 0};
		else if constexpr (displacement == Displacement::NONE)
			delta_ = {0, 0};

		initHash_();
	}

	template<>
	DisplacementAction<Displacement::RANDOM>::DisplacementAction(
			const MCState& initialState,
			const MCParams& params,
			const std::vector<MCActionPtr>& otherActions,
			const MCActionPtr parent
		) : CachedMCAction(initialState, params)
	{
		double angleFrom = 0;
		double angleTo = 2*M_PI;
		std::vector<double> anglesToOthers;

		auto computeAngle = [&anglesToOthers, &initialState](const MCRobotState& other) {
			auto delta = other.pos - initialState.robot.pos;
			return std::atan2(delta.y(), delta.x());
		};

		auto degToRad = [] (const double& deg) -> double { return deg * (M_PI / 180); };

		if (parent != nullptr && (parent->getInitialRobotState().pos - initialState.robot.pos).squaredNorm() > 1) {
			double parentAngle = computeAngle(parent->getInitialRobotState());
			double maxTurnAngle = degToRad(params.randomDisplacementMaxTurnAngle);
			angleFrom = parentAngle + (M_PI - maxTurnAngle);
			angleTo = parentAngle + (M_PI + maxTurnAngle);
		}

		for (size_t i = 0; i < otherActions.size(); i++) {
			anglesToOthers.push_back(computeAngle(otherActions[i]->getFinalRobotState()));
		}

		auto isAngleValid = [&anglesToOthers, &degToRad, &params](const double& angle) -> bool {
			for (size_t i = 0; i < anglesToOthers.size(); i++) {
				if (std::abs(anglesToOthers[i] - angle) < degToRad(params.randomDisplacementMinSpread)) {
					return false;
				}
			}
			return true;
		};

		std::uniform_real_distribution<double> angle_distr(angleFrom, angleTo);
		size_t outerTries = 0, innerTries = 0, maxOuterTries = 25, maxInnerTries = 25;
		do {
			innerTries = 0;
			double alpha;
			do {
				alpha = angle_distr(utils::RNG::get());
			} while (!isAngleValid(alpha) && ++innerTries < maxInnerTries);
			if (innerTries >= maxInnerTries) {
				infeasibleFlag_ = true;
				break;
			}
			double r = params.randomDisplacementLength;
			delta_ = {r*std::cos(alpha), r*std::sin(alpha)};
		} while (!isFeasible_(initialState) && ++outerTries < maxOuterTries);

		if (outerTries >= maxOuterTries) {
			infeasibleFlag_ = true;
		}

		initHash_();
	}

	template<Displacement displacement, int multiplier>
	void DisplacementAction<displacement, multiplier>::initHash_() {
		auto finalPos = getFinalRobotState().pos;
		double binSize = 0.2;
		int binX = finalPos.x() / binSize;
		int binY = finalPos.y() / binSize;
		std::stringstream hash;
		hash << getName();
		hash <<  "->(" << binX << ", " << binY << ")";
		hash_ = hash.str();
	}

	template<Displacement displacement, int multiplier>
	std::string DisplacementAction<displacement, multiplier>::getName() const
	{
		std::string name;
		if constexpr (displacement == Displacement::NORTH)
			name = "NORTH";
		else if constexpr (displacement == Displacement::SOUTH)
			name = "SOUTH";
		else if constexpr (displacement == Displacement::EAST)
			name = "EAST";
		else if constexpr (displacement == Displacement::WEST)
			name = "WEST";
		else if constexpr (displacement == Displacement::NONE)
			name = "NONE";
		else if constexpr(displacement == Displacement::RANDOM)
			name = "RANDOM";

		if constexpr (multiplier > 1)
			name = "BIG" + name;

		return name;
	}

	template<Displacement displacement, int multiplier>
	std::string DisplacementAction<displacement, multiplier>::toString() const
	{
		std::stringstream result;
		result << getName();
		pos_t ipos = getInitialRobotState().pos;
		pos_t fpos = getFinalRobotState().pos;
		result << "[(" << ipos.x() << ", " << ipos.y() << ")->";
		result <<  "(" << fpos.x() << ", " << fpos.y() << ")]";
		return result.str();
	}

	template<Displacement displacement, int multiplier>
	CachedMCAction::hashKey_t DisplacementAction<displacement, multiplier>::getHash() const
	{
		return hash_;
	}


	template<Displacement displacement, int multiplier>
	template<typename Iterator>
	bool DisplacementAction<displacement, multiplier>::noCollision_(const OccupancyMap& map, Iterator it) const {
		for ( ; !it.isPastEnd(); ++it) {
			if (map.isOccupied(*it))
				return false;
		}
		return true;
	}

	template<Displacement displacement, int multiplier>
	bool DisplacementAction<displacement, multiplier>::isFeasible_(const MCState& state) const {
		if (infeasibleFlag_)
			return false;

		pos_t destination = getFinalRobotState().pos;
		if (!state.map.isInside(destination))
			return false;

		auto lineIterator = state.map.getLineIterator(getInitialRobotState().pos, destination);
		return noCollision_(state.map, lineIterator);
	}

	template<Displacement displacement, int multiplier>
	MCRobotState DisplacementAction<displacement, multiplier>::getFinalRobotState() const {
		auto result = getInitialRobotState();
		result.pos += delta_;
		return result;
	}

	// Explicit instantiations of the template arguments
	template class DisplacementAction<Displacement::NORTH>;
	template class DisplacementAction<Displacement::SOUTH>;
	template class DisplacementAction<Displacement::EAST>;
	template class DisplacementAction<Displacement::WEST>;

	template class DisplacementAction<Displacement::NONE>;
	template class DisplacementAction<Displacement::RANDOM>;

	template class DisplacementAction<Displacement::NORTH, 5>;
	template class DisplacementAction<Displacement::SOUTH, 5>;
	template class DisplacementAction<Displacement::EAST, 5>;
	template class DisplacementAction<Displacement::WEST, 5>;
}

