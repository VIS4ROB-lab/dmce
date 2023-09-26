#pragma once

#include <ros/ros.h>

#include "dmce_core/OccupancyMap.hpp"
#include "dmce_sim/GroundTruthMap.hpp"

namespace dmce {
	/**
	 * This class handles moving the robot towards its goal,
	 * based on paths passed to it.
	 * It also checks collisions with the ground-truth map.
	 */
	class Navigation {
	public:
		/**
		 * truthMap is the ground-truth map used for collision checking.
		 * It is assumed to have inflated obstacles to account for the
		 * robot's size.
		 */
		Navigation(
				const GroundTruthMap& truthMap,
				const pos_t& initialPosition,
				double robotSpeed = 1,
				double navigationCutoff = 0.5
			);

		/**
		 * Checks current path, moves robot and sets flags.
		 * Throws exception if no navigation map has been passed.
		 */
		void moveRobotToGoal(const ros::Duration& timeStep);

		/**
		 * Update the navigation map. It is expected to have
		 * inflated obstacles to account for the robot's size.
		 */
		void updateNavigationMap(const OccupancyMap& map);

		/**
		 * Returns the current goal position for the robot.
		 */
		pos_t getCurrentGoal() const;

		/**
		 * Update the path to the current goal.
		 * Does not update path if it does not end at the current goal or is invalid.
		 * Does not update path if the current path is valid and shorter than the new one.
		 */
		void updatePath(plan_t newPath);

		/**
		 * Update the global plan.
		 * Discards previous plan and resets the plan failure flag.
		 */
		void updatePlan(plan_t newPlan);

		/**
		 * Status flag to be sent to planning server.
		 * False means the last global plan was aborted due to an error.
		 */
		bool wasLastPlanSuccessful() const;

		/**
		 * True if there is a plan with a goal.
		 * Does not check validity of the plan.
		 */
		bool hasGoal() const;

		/**
		 * Returns the current robot position.
		 */
		pos_t getRobotPosition() const;

		/**
		 * Checks if the current path is valid and does not
		 * collide based on the current navigation map.
		 * Returns true if the path is valid.
		 */
		bool hasValidPath() const;

		/**
		 * Returns the length of the current path on the navigation map.
		 * Returns infinity if current path collides with an obstacle,
		 * or if the current plan is empty.
		 * Throws exception if no map has been passed in.
		 */
		double getCurrentPathLength() const;

		/**
		 * Returns the current plan.
		 */
		plan_t getCurrentPlan() const;

		/**
		 * Returns the current path.
		 */
		plan_t getCurrentPath() const;

		/**
		 * If true, moveRobotToGoal can be called safely.
		 */
		bool isReady() const;

		/**
		 * Clear the current plan.
		 */
		void resetPlan(bool success = true);

	private:
		pos_t robotPosition_;
		bool navMapReady_ = false;
		const double robotSpeed_;
		const double navigationCutoff_;
		plan_t currentPath_;
		plan_t currentPlan_;
		OccupancyMap navigationMap_;
		bool planAborted_ = false;
		GroundTruthMap truthMap_;

		constexpr const static double INF = std::numeric_limits<double>::infinity();

		void skipPastWaypoints_(const pos_t& position, plan_t& path);

		void moveRobotOnPath_(double dt);

		bool isRobotAtGoal_() const;

		bool isGoalValid_() const;

		bool isStraightLineValidPath_(const pos_t& from, const pos_t& to) const;

		double linearPathCost_(const pos_t& from, const pos_t& to) const;

		double pathCost_(const pos_t& robotPosition, const plan_t& path) const;

		bool isPathValid_(const plan_t& path, double& pathCostOut) const;

		bool positionCollides_(const pos_t& pos) const;

		void moveRobotToClosestFreeSpace_();
	};
}

