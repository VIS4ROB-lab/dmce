#include <thread>

#include "mre_dmcts_core/RandomPlanner.hpp"
#include "mre_dmcts_core/FrontierPlanner.hpp"
#include "mre_dmcts_core/ClusteredFrontierPlanner.hpp"
#include "mre_dmcts_core/ExternalPlanner.hpp"

#include "mre_dmcts_mcplanner/MCTSPlanner.hpp"
#include "mre_dmcts_mcplanner/DMCTSPlanner.hpp"
#include "mre_dmcts_mcplanner/MCParams.hpp"

#include "mre_dmcts_nodes/NodeServer.hpp"

#include "grid_map_msgs/GridMap.h"
#include "mre_dmcts_msgs/GetPlan.h"
#include "mre_dmcts_msgs/RobotPosition.h"

namespace mre_dmcts {
	class PlannerServer : public NodeServer {
		std::unique_ptr<Planner> planner_;
		ros::ServiceServer server_;
		ros::Subscriber mapSubscriber_;
		ros::Subscriber clusterSubscriber_;
		ros::Subscriber positionSubscriber_;
		std::vector<ros::Subscriber> otherRobotPlanSubscribers_;
		ros::Publisher planPublisher_;
		ros::Publisher markerPublisher_;

		ros::Duration timeSinceLastMapCheck_;
		RobotMap map_;
		bool explorationFinished_ = false;
		double explorationCompletionThreshold_;
		pos_t groundStationPosition_;
		double robotDiameter_;
		visualization_msgs::Marker arrowMarker_;

		/**
		 * Service callback which provides updates on the global plan.
		 */
		bool planServiceCallback_(mre_dmcts_msgs::GetPlan::Request& request, mre_dmcts_msgs::GetPlan::Response& response);

		/**
		 * Subscriber handler for plans shared by other robots.
		 */
		void peerPlanCallback_(const mre_dmcts_msgs::RobotPlan& msg);

		/**
		 * Subscriber callback which updates the RobotMap.
		 */
		void mapCallback_(const grid_map_msgs::GridMap& msg);

		/**
		 * Subscriber callback for frontier clusters.
		 */
		void clusterCallback_(const mre_dmcts_msgs::FrontierClusters& msg);

		/**
		 * Subscriber callback which updates the robot position.
		 */
		void positionCallback_(const mre_dmcts_msgs::RobotPosition& msg);

		/**
		 * This method will run in a worker thread.
		 * It calls planner_.updatePlan() in a loop until it is killed.
		 */
		void replanWorker_();

		/**
		 * Fetch the MCParams from the ROS parameter server.
		 */
		MCParams getMCParams();

		/**
		 * Checks if the exploration is done (no more frontiers).
		 */
		bool isMapFullyExplored_(const RobotMap& map) const;

		/**
		 * Publish the given partial plan, both to other robots and as RViz markers.
		 */
		void publishPlan_(const mre_dmcts_msgs::RobotPlan& planMsg);

	protected:
		void update_(ros::Duration timeStep) override;

	public:
		PlannerServer(ros::NodeHandle nh, unsigned int robotId, const double& timeout = 5);

		virtual std::string getName() const override {
			return "PlannerServer";
		}
	};

};
