#include "dmce_nodes/RobotMarkerServer.hpp"

namespace dmce {

	void RobotMarkerServer::positionCallback_(const dmce_msgs::RobotPosition& msg) {
		pos_t curPos = { msg.x_position, msg.y_position };
		visualization_msgs::Marker robotMarker = utils::getDefaultMarker_(robotDiameter_);
		robotMarker.id = msg.robotId;
		robotMarker.pose.position.x = curPos.x();
		robotMarker.pose.position.y = curPos.y();
		auto& color = robotColors_[msg.robotId % robotColors_.size()];
		robotMarker.color.r = color.x();
		robotMarker.color.g = color.y();
		robotMarker.color.b = color.z();
		markerPublisher_.publish(robotMarker);

		auto curPoint = posToPoint(curPos);
		if (!robotTraces_[msg.robotId].empty())
			robotTraces_[msg.robotId].push_back(posToPoint(robotPositions_[msg.robotId]));
		else
			robotTraces_[msg.robotId].push_back(curPoint);
		robotTraces_[msg.robotId].push_back(curPoint);

		robotPositions_[msg.robotId] = curPos;
		positionsReceived[msg.robotId] = true;
	}

	void RobotMarkerServer::update_(ros::Duration timeStep) {
		bool allReceived = true;
		for (bool iReceived : positionsReceived)
			allReceived &= iReceived;

		if (!allReceived)
			return;

		// Publish marker for gound-station
		visualization_msgs::Marker groundStationMarker = utils::getDefaultMarker_(robotDiameter_);
		groundStationMarker.id = 0;
		groundStationMarker.pose.position.x = robotPositions_[0].x();
		groundStationMarker.pose.position.y = robotPositions_[0].y();
		groundStationMarker.type = visualization_msgs::Marker::CUBE;
		markerPublisher_.publish(groundStationMarker);

		// Publish lines to represent connectivity
		visualization_msgs::Marker linesMarker = utils::getDefaultMarker_(robotDiameter_);
		linesMarker.type = visualization_msgs::Marker::LINE_LIST;
		linesMarker.ns = "connectivity";
		linesMarker.scale.x = 0.2*robotDiameter_; // Controls line thickness
		linesMarker.color.r = 0.9;
		linesMarker.color.g = 0.7;
		auto nrobots = getRobotCount();

		for (unsigned int from = 0; from <= nrobots; from++) {
			for (unsigned int to = from+1; to <= nrobots; to++) {
				if (connectivityHandler_.bothConnected(from, to)) {
					linesMarker.points.push_back(posToPoint(robotPositions_[from]));
					linesMarker.points.push_back(posToPoint(robotPositions_[to]));
				}
			}
		}
		markerPublisher_.publish(linesMarker);

		// Publish the robot's traces (past positions)
		for (unsigned int robotId = 1; robotId <= nrobots; robotId++) {
			visualization_msgs::Marker traceMarker = utils::getDefaultMarker_(robotDiameter_);
			traceMarker.type = visualization_msgs::Marker::LINE_LIST;
			std::stringstream ss; ss << "robot" << robotId << "/trace";
			traceMarker.ns = ss.str();
			traceMarker.scale.x = 0.2*robotDiameter_; // Controls line thickness
			auto& color = robotColors_[robotId % robotColors_.size()];
			traceMarker.color.r = color.x();
			traceMarker.color.g = color.y();
			traceMarker.color.b = color.z();
			traceMarker.points = robotTraces_[robotId];
			markerPublisher_.publish(traceMarker);
		}
	}
};
