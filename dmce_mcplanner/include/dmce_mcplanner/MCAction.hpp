#pragma once

#include "dmce_mcplanner/MCParams.hpp"
#include "dmce_mcplanner/MCState.hpp"
#include "dmce_core/TypeDefs.hpp"

namespace dmce {
	class MCAction;
	using MCActionPtr = std::shared_ptr<MCAction>;

	/**
	 * Abstract base class representing a generic action
	 * that can be taken as part of the Monte Carlo tree search.
	 */
	class MCAction {
		MCRobotState iRobotState_;

	protected:
		const MCParams& params_;

		/**
		 * Simulate carrying out this action from the given state.
		 * The state will updated to reflect the state after executing the action.
		 * This includes robot position and map state.
		 * Return value is the estimated value for executing the action.
		 */
		virtual double simulate_(MCState& state, const double& idleValue) const;

		/**
		 * True if the action is feasible for the given state.
		 */
		virtual bool isFeasible_(const MCState& state) const = 0;

		/**
		 * Cast a ray between two positions on the given map
		 * and mark unexplored cells as free.
		 */
		static bool castRay_(const pos_t& from, const pos_t& to, RobotMap& map);

	public:
		using hashKey_t = std::string;

		MCAction(
			const MCState& initialState,
			const MCParams& params,
			const std::vector<MCActionPtr>& otherActions = {},
			const MCActionPtr parent = nullptr
		) : iRobotState_(initialState.robot), params_(params)
		{ }

		virtual ~MCAction() { }

		/**
		 * Simulate performing this action on the given map.
		 * The given robot map is updated to reflect the final state.
		 * Returns the predicted value of this action (entropy reduction).
		 */
		virtual double simulate(MCState& state, const double& idleValue) {
			state.robot = this->getFinalRobotState();
			return simulate_(state, idleValue);
		}

		/**
		 * Check if this action is feasible on the given map.
		 */
		virtual bool isFeasible(const MCState& state) {
			return isFeasible_(state);
		}

		/**
		 * Calculate the robot position resulting from performing
		 * this action on the given initial state.
		 */
		virtual MCRobotState getFinalRobotState() const = 0;

		/**
		 * Returns the robot state this action begins with.
		 */
		virtual MCRobotState getInitialRobotState() const {
			return iRobotState_;
		}

		/**
		 * Returns the esimated time required to perform the given action.
		 */
		virtual double getDuration() const {
			double travelDistance = (iRobotState_.pos - getFinalRobotState().pos).norm();
			return params_.actionBaseDuration + travelDistance / params_.robotSpeed;
		}

		/**
		 * Return the name of this action.
		 */
		virtual std::string toString() const = 0;

		/**
		 * Returns whether or not this action may be cached.
		 */
		virtual bool isCacheable() const { return false; }

		/**
		 * Returns a string representing a hashed version of this action.
		 */
		virtual hashKey_t getHash() const { return ""; }
	};
}
