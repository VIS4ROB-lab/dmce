#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "grid_map_core/TypeDefs.hpp"

#include "dmce_core/OccupancyMap.hpp"
#include "dmce_core/TypeDefs.hpp"

namespace dmce {

	/**
	 * Abstract base class for a generic path planner.
	 */
	class PathPlanner {
	protected:
		double speed_;
		double tolerance_;
		double robotDiameter_;

		plan_t plan_;
		std::recursive_mutex planMutex_;
		bool hasPlan_ = false;
		std::atomic<bool> targetChanged_ = false;

	private:
		mutex_t targetMutex_;
		bool isTargetSet_ = false;
		pos_t targetPosition_;

	public:
		/**
		 * @param speed [m/s] Robot movement speed.
		 * @param tolerance [m] Distance from target to be achieved.
		 * @param robotDiameter [m] Diameter of the robot, used for obstacle avoidance.
		 */
		PathPlanner(double speed = 1, double tolerance = 0.2, double robotDiameter = 1)
		// PathPlanner(double speed = 1, double tolerance = 0.2, double robotDiameter = 1)
			: speed_(speed), tolerance_(tolerance), robotDiameter_(robotDiameter)
		{

		}

		/**
		 * Calculates the next position on the path and updates the argument.
		 * Method to be overridden by child classes.
		 * @param timeStep [m] Time-step to move by.
		 * @param position [m] 2D position to be updated.
		 * @returns True if the position was changed, false otherwise.
		 */
		virtual bool move(const double& timeStep, pos_t& position) = 0;

		/**
		 * Update the map used internally for planning.
		 * This is a costly operation as it requires obstacle inflation.
		 */
		virtual void updateMap(OccupancyMap map) { }

		virtual bool makePlan(const pos_t& robotPosition) {
			getTarget();
			hasPlan_ = true;
			return true;
		}

		bool isPlanReady() const {
			return hasPlan_;
		}

		void clearPlan() {
			std::lock_guard<std::recursive_mutex> planGuard(planMutex_);
			plan_.clear();
			hasPlan_ = false;
		}

		plan_t getPlan() {
			if (!isPlanReady())
				throw std::runtime_error("[PathPlanner::getPlan] No plan has been made yet! Call makePlan.");
			std::lock_guard<std::recursive_mutex> guard(planMutex_);
			return plan_;
		}

		virtual ~PathPlanner() { }

		/**
		 * Set the target position this planner should move towards.
		 */
		void setTarget(pos_t target) {
			lockGuard_t guard(targetMutex_);
			targetPosition_ = target;
			isTargetSet_ = true;
			targetChanged_.store(true);
		}

		/**
		 * Get the target position.
		 * Throws error if target position has not been set yet.
		 */
		const pos_t& getTarget() {
			lockGuard_t guard(targetMutex_);
			if (!isTargetSet_)
				throw std::runtime_error("[PathPlanner::getTarget] Target not set!");
			return targetPosition_;
		}

	};
}
