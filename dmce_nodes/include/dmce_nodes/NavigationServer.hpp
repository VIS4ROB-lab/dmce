#pragma once

#include <actionlib/client/simple_action_client.h>

#include "dmce_msgs/GetMap.h"
#include "dmce_msgs/NavigationFailureSignal.h"
#include "dmce_msgs/GetPlan.h"
#include "dmce_msgs/PathfindingAction.h"

#include "dmce_sim/GroundTruthMap.hpp"
#include "dmce_sim/Navigation.hpp"

#include "dmce_core/RobotMap.hpp"

#include "dmce_nodes/NodeServer.hpp"

namespace dmce {
	/**
	 * Handles moving the robot according to the plan provided by
	 * PlannerServer, avoiding obstacles and evaluating collisions.
	 * Uses an actionlib client to PathfindingServer to evaluate
	 * obstacle-avoiding paths.
	 */
	class NavigationServer : public NodeServer {
	public:
		NavigationServer(ros::NodeHandle nh, unsigned int robotId, double timeout = 10);

		virtual std::string getName() const override {
			return "NavigationServer";
		}

	protected:
		virtual void update_(ros::Duration timeStep) override;

	private:
		ros::ServiceClient globalPlanClient_;
		ros::Publisher positionPublisher_;
		ros::Publisher pathPublisher_;
		ros::Publisher globalPlanPublisher_;
		ros::Publisher failurePublisher_;
		ros::Subscriber inflatedMapSubscriber_;

		using ActionClient_t = actionlib::SimpleActionClient<dmce_msgs::PathfindingAction>;
		ActionClient_t actionClient_;

		std::unique_ptr<Navigation> navigation_;
		GroundTruthMap inflatedTruthMap_;
		double robotSpeed_;
		double robotDiameter_;
		double navigationCutoff_;
		bool randomiseInitialPosition_;
		double maxPlanAge_;
		double currentPlanAge_ = 0;
		bool signalFailure_ = false;
		int maxPathfindingFailures_;
		unsigned int consecutivePathfindingFailures_ = 0;

		void navigationMapCallback_(const grid_map_msgs::GridMap& mapMsg);

		pos_t getStartingPosition_(dmce_msgs::GetStartingArea::Response startingArea);

		void publishRobotPosition_();

		pos_t getCurrentRobotPosition_() const;

		void publishPlan_(const plan_t& plan, const ros::Publisher& publisher);

		void tryFetchNewPlan_(bool goalSuccess);

		void pathCallback_(const dmce_msgs::PathfindingFeedbackConstPtr& pathFeedback);

		void sendGoalToPathfinding_();

		void checkAbortConditions_(const ros::Duration& timeStep);

		bool navigateToGoal_(const ros::Duration& timeStep);
	};
}
