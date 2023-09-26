#include <random>

#include "dmce_sim/GroundTruthMap.hpp"

#include "dmce_nodes/NodeServer.hpp"

#include "dmce_msgs/GetMap.h"
#include "dmce_msgs/GetStartingArea.h"

namespace dmce {
	/**
	 * This node publishes the grount-truth map and the starting area position.
	 */
	class GroundTruthMapServer : public NodeServer {
		GroundTruthMap map_;
		ros::ServiceServer groundTruthMapServer_;
		ros::ServiceServer startingAreaServer_;
		dmce_msgs::GetStartingArea::Response startingArea_;

		/**
		 * Service callback providing the ground-truth map.
		 * Only meant to be invoked by the ROS service.
		 */
		bool groundTruthMapCallback_(dmce_msgs::GetMap::Request& request, dmce_msgs::GetMap::Response& response);

		/**
		 * Service callback providing information about the starting area.
		 */
		bool startingAreaCallback_(dmce_msgs::GetStartingArea::Request& request, dmce_msgs::GetStartingArea::Response& response);

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
