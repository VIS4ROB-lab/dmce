#include "dmce_nodes/NodeServer.hpp"

namespace dmce {
	NodeServer::NodeServer(ros::NodeHandle nh, unsigned int robotId)
		: nodeHandle_(nh), robotId_(robotId), connectivityHandler_(1)
	{
		int nRobots = 1;
		getRequiredParam("/sim/nRobots", nRobots);
		nRobots_ = (nRobots < 0) ? 0 : nRobots;
		connectivityHandler_ = RobotConnectivity(nRobots_);

		connectivitySubscriber_ = nodeHandle_.subscribe(
			"/groundStation/RobotConnectivity", 1,
			&NodeServer::connectivityCallback_, this
		);
	}

	bool NodeServer::canReceiveFromPeer(unsigned int peerId) const {
		return connectivityHandler_.isConnected(peerId, robotId_);
	}

	void NodeServer::connectivityCallback_(const RobotConnectivity::Message& msg) {
		connectivityHandler_ = RobotConnectivity(msg);
	}

	dmce_msgs::GetStartingArea::Response NodeServer::fetchStartingArea_(const double& timeout) {
		ros::ServiceClient client = nodeHandle_.serviceClient<dmce_msgs::GetStartingArea>(
			"/groundStation/StartingAreaService"
		);
		if (!client.waitForExistence(ros::Duration(timeout))) {
			throw std::runtime_error(
				"[NodeServer] StartingAreaService not found!"
			);
		}
		dmce_msgs::GetStartingArea area_msg;
		if (!client.call(area_msg.request, area_msg.response)) {
			throw std::runtime_error(
				"[NodeServer] Error fetching starting area."
			);
		}
		return area_msg.response;
	}


	pos_t NodeServer::fetchGroundStationPosition(const double& timeout) {
		auto startingArea = fetchStartingArea_();
		return { startingArea.position_x, startingArea.position_y };
	}

	void NodeServer::update(ros::Duration timeStep) {
		this->update_(timeStep);
	}

	/**
	 * Returns a string representing the concrete node server's name.
	 */
	std::string NodeServer::getName() const {
		return "NodeServer";
	}

	unsigned int NodeServer::getRobotCount() const {
		return nRobots_;
	}

	unsigned int NodeServer::getRobotId() const {
		return robotId_;
	}


	GroundTruthMap NodeServer::fetchGroundTruthMap_(const double& timeout) {
		ros::ServiceClient client = nodeHandle_.serviceClient<dmce_msgs::GetMap>(
			"/groundStation/GroundTruthMapService"
		);
		std::stringstream err; err << "[" << this->getName() << "] ";
		if (!client.waitForExistence(ros::Duration(timeout))) {
			err << "GroundTruthMapService not found!";
			throw std::runtime_error(err.str());
		}
		dmce_msgs::GetMap map_msg;
		if (!client.call(map_msg.request, map_msg.response)) {
			err << "Error fetching ground-truth map.";
			throw std::runtime_error(err.str());
		}
		return map_msg.response.map;
	}

}
