#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "costmap_2d/costmap_2d.h"
#include "dmce_sim/PathPlanner.hpp"
#include "dmce_sim/OmplAdapter.hpp"
#include "dmce_core/OccupancyMap.hpp"

namespace dmce {
	/**
	 * Path planner based on OMPL.
	 */
	class OmplPathPlanner : public PathPlanner {
		costmap_2d::Costmap2D costMap_;
		OccupancyMap occMap_;
		std::mutex mapMutex_;

		double INF = std::numeric_limits<double>::infinity();

		bool isStraightLineValidPlan_(const pos_t& from, const pos_t& to) {
			lockGuard_t mapGuard(mapMutex_);
			for (auto it = occMap_.getLineIterator(from, to); !it.isPastEnd(); ++it) {
				if (occMap_.isOccupied(*it)) {
					return false;
				}
			}
			return true;
		}

		double lineCost(const pos_t& from, const pos_t& to) {
			if (!isStraightLineValidPlan_(from, to))
				return INF;

			return (to - from).norm();
		}

		double planCost_(const pos_t& robotPosition, const plan_t& plan) {
			if (plan.empty())
				return INF;

			double cost = lineCost(robotPosition, poseToPos(plan[0]));
			for (unsigned int i = 0; i < plan.size() - 1; i++) {
				cost += lineCost(poseToPos(plan[i]), poseToPos(plan[i+1]));
			}
			return cost;
		}

		void setPlan_(const plan_t& plan) {
			std::lock_guard<std::recursive_mutex> planGuard(planMutex_);
			if (!targetChanged_.load()) {
			    plan_ = plan;
				hasPlan_ = true;
			}
		}

	public:
		using PathPlanner::PathPlanner;

		void updateMap(OccupancyMap map) override {
			lockGuard_t guard(mapMutex_);
			occMap_ = map;
			costMap_ = occMap_.toCostmap();
		}

		bool makePlan(const pos_t& robotPosition) override {
			geometry_msgs::PoseStamped start;
			start.pose.position.x = robotPosition.x();
			start.pose.position.y = robotPosition.y();
			start.pose.position.z = 0;

			targetChanged_.store(false);
			pos_t targetPosition = getTarget();

			if (isStraightLineValidPlan_(robotPosition, targetPosition)) {
				setPlan_({posToPose(targetPosition)});
				return true;
			}

			geometry_msgs::PoseStamped goal;
			goal.pose.position.x = targetPosition.x();
			goal.pose.position.y = targetPosition.y();
			goal.pose.position.z = 0;

			tf2::Quaternion orientation;
			orientation.setRPY(0, 0, 0);
			start.pose.orientation = tf2::toMsg(orientation);
			goal.pose.orientation = tf2::toMsg(orientation);

			start.header.frame_id = "map";
			goal.header.frame_id = "map";

			plan_t newPlan;
			bool success = false;
			{
				lockGuard_t mapGuard(mapMutex_);
				OmplAdapter planner("planner", &costMap_, "map");
				success = planner.makePlan(start, goal, newPlan);
			}
			if (success) {
				planMutex_.lock();
				double oldPlanCost = planCost_(robotPosition, plan_);
				bool isBetter = planCost_(robotPosition, newPlan) < oldPlanCost;
				planMutex_.unlock();
				if (isBetter)
					setPlan_(newPlan);
				else if (oldPlanCost == INF)
					clearPlan();
			}
			return isPlanReady();
		}

		void skipPastWaypoints(const pos_t& position) {
			std::lock_guard<std::recursive_mutex> guard(planMutex_);
			unsigned int closestIdx = 0;
			double closestDist = std::numeric_limits<double>::infinity();
			auto dist =
				[&position] (const geometry_msgs::PoseStamped& waypoint_msg) {
					pos_t waypoint;
					waypoint.x() = waypoint_msg.pose.position.x;
					waypoint.y() = waypoint_msg.pose.position.y;
					return (waypoint - position).norm();
				};

			for (unsigned int i = 0; i < plan_.size(); i++) {
				double d = dist(plan_[i]);
				if (d < closestDist) {
					closestDist = d;
					closestIdx = i;
				}
			}

			if (closestDist < 1) {
				plan_.erase(plan_.begin(), plan_.begin() + closestIdx);
				unsigned int eraseUpTo = 0;
				for (unsigned int i = 0; i < plan_.size()-1; i++) {
					if (dist(plan_[i]) < 0.5*robotDiameter_) {
						eraseUpTo = i;
					}
				}
				plan_.erase(plan_.begin(), plan_.begin()+eraseUpTo);
			}

			hasPlan_ = !plan_.empty();
		}

		bool move(const double& timeStep, pos_t& position) override {
			// ROS_INFO("[OmplPathPlanner::move] %zu", plan_.size());
			skipPastWaypoints(position);

			if (!isPlanReady()) {
				return false;
			}

			std::lock_guard<std::recursive_mutex> guard(planMutex_);

			auto newPos_msg = plan_[0].pose.position;
			pos_t targetPosition;
			targetPosition.x() = newPos_msg.x;
			targetPosition.y() = newPos_msg.y;

			Eigen::Vector2d direction = targetPosition - position;
			double distance = direction.norm();
			if (distance > tolerance_) {
				direction /= distance;
				Eigen::Vector2d delta = direction * speed_ * timeStep;
				if (delta.norm() < distance) {
					position += delta;
				} else {
					position = targetPosition;
				}
				return true;
			} else {
				plan_.erase(plan_.begin());
				hasPlan_ = !plan_.empty();
				return false;
			}
		}
	};
}
