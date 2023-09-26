#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <grid_map_core/TypeDefs.hpp>

#include "dmce_core/TypeDefs.hpp"
#include "dmce_core/RobotConnectivity.hpp"
#include "dmce_nodes/NodeServer.hpp"
#include "dmce_msgs/RobotPosition.h"

namespace dmce {
	/**
	 * This node publishes RViz markers for each robot and the ground station.
	 */
	class RobotMarkerServer : public NodeServer {
		std::vector<ros::Subscriber> positionSubscribers_;
		std::vector<pos_t> robotPositions_;
		std::vector<std::vector<geometry_msgs::Point>> robotTraces_;
		std::vector<bool> positionsReceived;
		ros::Subscriber connectivitySubscriber_;
		RobotConnectivity connectivityHandler_;
		ros::Publisher markerPublisher_;
		double robotDiameter_;
		std::vector<Eigen::Vector3d> robotColors_ = {{1., .0, .0}, {.0, 1., .0}, {.0, 1.0, 1.}, {.0, .0, 1.}};

		/**
		 * Called when a position update is received from one of the robots.
		 */
		void positionCallback_(const dmce_msgs::RobotPosition& msg);

		/**
		 * Called when a connectivity update arrives.
		 */
		void connectivityCallback_(const dmce::RobotConnectivity::Message& msg) {
			connectivityHandler_ = RobotConnectivity(msg);
		}

		geometry_msgs::Point posToPoint(const pos_t& pos) {
			geometry_msgs::Point point;
			point.x = pos.x();
			point.y = pos.y();
			return point;
		}

	protected:
		void update_(ros::Duration timeStep) override;

	public:

		RobotMarkerServer(ros::NodeHandle nh, unsigned int robotId)
			: NodeServer(nh, robotId),
			  robotPositions_(getRobotCount()+1),
			  robotTraces_(getRobotCount()+1),
			  positionsReceived(getRobotCount()+1, false),
			  connectivityHandler_(getRobotCount())
		{
			getRequiredParam("/robot/diameter", robotDiameter_);
			positionSubscribers_ = subscribeForEachRobot(
					"RobotPosition", 1, &RobotMarkerServer::positionCallback_, this
				);

			markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(
					"visualization_marker", 10
				);

			connectivitySubscriber_ = nodeHandle_.subscribe(
					"/groundStation/RobotConnectivity", 1,
					&RobotMarkerServer::connectivityCallback_, this
				);

			auto startingArea = fetchStartingArea_();
			robotPositions_[0] = { startingArea.position_x, startingArea.position_y };
			positionsReceived[0] = true;
		}
		
		virtual std::string getName() const override {
			return "RobotMarkerServer";
		}
	};
}

