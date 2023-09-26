#include "dmce_nodes/FrontierClusterServer.hpp"

namespace dmce {
	FrontierClusterServer::FrontierClusterServer(ros::NodeHandle nh, unsigned int robotId)
		: NodeServer(nh, robotId)
	{
		mapSubscriber_ = nodeHandle_.subscribe(
			"inflatedMap", 1,
			&FrontierClusterServer::mapCallback_,
			this
		);

		clusterPublisher_ =
			nodeHandle_.advertise<dmce_msgs::FrontierClusters>("FrontierClusters", 1);

		markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(
				"/viz/visualization_marker", 10
			);
	}

	void FrontierClusterServer::update_(ros::Duration timeStep) {
		if (!receivedMap_)
			return;

		clusterHandler_.fromMapFrontiers(latestMap_);
		clusterPublisher_.publish(clusterHandler_.getMessage());
		publishCentroidMarkers_();
		receivedMap_ = false;
	}

	void FrontierClusterServer::mapCallback_(const grid_map_msgs::GridMap& msg) {
		latestMap_ = OccupancyMap(msg);
		mapResolution_ = latestMap_.getResolution();
		receivedMap_ = true;
	}

	void FrontierClusterServer::publishCentroidMarkers_() {
		std::stringstream ss;
		ss << "robot" << robotId_ << "/clusters";
		std::string ns = ss.str();
		auto centroids = clusterHandler_.getCentroidPositions();
		for (unsigned int i = 0; i < clusterHandler_.getNClusters(); ++i) {
			auto marker = utils::getDefaultMarker_(mapResolution_);
			marker.ns = ns; marker.id = i;
			marker.pose.position.x = centroids[i].x();
			marker.pose.position.y = centroids[i].y();
			marker.lifetime = ros::Duration(1);
			marker.color.r = 0.7;
			marker.color.g = 0.2;
			markerPublisher_.publish(marker);
		}
	}
}
