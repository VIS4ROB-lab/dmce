#pragma once

#include "visualization_msgs/Marker.h"

#include "mre_dmcts_nodes/NodeServer.hpp"
#include "mre_dmcts_core/TypeDefs.hpp"
#include "mre_dmcts_core/OccupancyMap.hpp"
#include "mre_dmcts_core/FrontierClustering.hpp"

namespace mre_dmcts {
	/**
	 * This node is responsible for listening to updates in
	 * the robot's map and publishing frontier cluster information.
	 */
	class FrontierClusterServer : public NodeServer {
		ros::Subscriber mapSubscriber_;
		ros::Publisher clusterPublisher_;
		ros::Publisher markerPublisher_;

		bool receivedMap_ = false;
		OccupancyMap latestMap_;
		FrontierClustering clusterHandler_;
		double mapResolution_;

		/**
		 * If the map was updated, compute the frontier clusters
		 * and publish them.
		 */
		void update_(ros::Duration timeStep) override;

		/**
		 * Called when receiving a map from the RobotMapServer
		 */
		void mapCallback_(const grid_map_msgs::GridMap& msg);

		/**
		 * Publish markers for the cluster centroids.
		 */
		void publishCentroidMarkers_();

	public:
		FrontierClusterServer(ros::NodeHandle nh, unsigned int robotId);
	};
}
