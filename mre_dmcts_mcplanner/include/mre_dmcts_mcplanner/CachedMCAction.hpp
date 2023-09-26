#pragma once

#include "MCAction.hpp"

namespace mre_dmcts {
	class CachedMCAction : public MCAction {
		bool wasInfeasible_ = false;
		bool hasNoExplorationValue_ = false;

	public:
		using MCAction::MCAction;

		bool isCacheable() const override {
			return true;
		}

		virtual hashKey_t getHash() const override = 0;

		virtual bool isFeasible(const MCState& state) override {
			if (params_.useActionCaching && wasInfeasible_)
				return false;

			bool ret = isFeasible_(state);
			wasInfeasible_ = wasInfeasible_ || !ret;
			return ret;
		}

		virtual double simulate(MCState& state, const double& idleValue) override {
			state.robot = this->getFinalRobotState();
			if (params_.useActionCaching && hasNoExplorationValue_)
				return 1 - state.map.getRelativeEntropy();

			unsigned int initUnknownCells = state.map.getNUnknownCells();
			double value = simulate_(state, idleValue);
			unsigned int finalUnknownCells = state.map.getNUnknownCells();
			if (finalUnknownCells == initUnknownCells) {
				hasNoExplorationValue_ = true;
			}
			return value;
		}
	};
}
