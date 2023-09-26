#pragma once

#include <ros/ros.h>
#include <grid_map_core/TypeDefs.hpp>
#include <tf/transform_broadcaster.h>

#include "dmce_nodes/NodeServer.hpp"
#include "dmce_core/RobotConnectivity.hpp"
#include "dmce_msgs/RobotPosition.h"

namespace dmce {
	class TransformServer : public NodeServer {
		std::vector<ros::Subscriber> positionSubscribers_;

		tf::TransformBroadcaster broadcaster_;

		void positionCallback_(const dmce_msgs::RobotPosition& msg) {
			if (!canReceiveFromPeer(msg.robotId))
				return;

			std::stringstream ss;
			ss << "robot" << robotId_ << "/robot" << msg.robotId;
			std::string frame_id = ss.str();

			tf::Transform transform;
			transform.setOrigin( tf::Vector3(msg.x_position, msg.y_position, 0) );
			tf::Quaternion q;
			q.setRPY(0, 0, 0);
			transform.setRotation(q);
			broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", frame_id));
		}

	protected:
		void update_(ros::Duration timeStep) override { }

	public:

		TransformServer(ros::NodeHandle nh, unsigned int robotId)
			: NodeServer(nh, robotId)
		{
			positionSubscribers_ = subscribeForEachRobot("RobotPosition", 1, &TransformServer::positionCallback_, this);
		}
		
		virtual std::string getName() const override {
			return "TransformServer";
		}
	};
}
