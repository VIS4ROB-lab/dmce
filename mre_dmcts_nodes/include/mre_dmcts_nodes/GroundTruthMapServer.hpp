#include <random>

#include "mre_dmcts_sim/GroundTruthMap.hpp"

#include "mre_dmcts_nodes/NodeServer.hpp"

#include "mre_dmcts_msgs/GetMap.h"
#include "mre_dmcts_msgs/GetStartingArea.h"

namespace mre_dmcts {
	/**
	 * This node publishes the grount-truth map and the starting area position.
	 */
	class GroundTruthMapServer : public NodeServer {
		GroundTruthMap map_;
		ros::ServiceServer groundTruthMapServer_;
		ros::ServiceServer startingAreaServer_;
		mre_dmcts_msgs::GetStartingArea::Response startingArea_;

		/**
		 * Service callback providing the ground-truth map.
		 * Only meant to be invoked by the ROS service.
		 */
		bool groundTruthMapCallback_(mre_dmcts_msgs::GetMap::Request& request, mre_dmcts_msgs::GetMap::Response& response);

		/**
		 * Service callback providing information about the starting area.
		 */
		bool startingAreaCallback_(mre_dmcts_msgs::GetStartingArea::Request& request, mre_dmcts_msgs::GetStartingArea::Response& response);

		/**
		 * Initialise the robots' starting area.
		 */
		void initStartingArea_();

	protected:
		void update_(ros::Duration timeStep) override { }

	public:
		GroundTruthMapServer(ros::NodeHandle nh, unsigned int robotId);
		
		virtual std::string getName() const override {
			return "GroundTruthMapServer";
		}
	};
}
