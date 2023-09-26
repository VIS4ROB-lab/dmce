#pragma once

#include <ros/ros.h>
#include "dmce_sim/GroundTruthMap.hpp"
#include "dmce_msgs/GetMap.h"
#include "dmce_core/utils.hpp"
#include "dmce_core/TypeDefs.hpp"
#include "dmce_core/RobotConnectivity.hpp"
#include "dmce_msgs/GetStartingArea.h"

namespace dmce {
	/**
	 * Abstract base class for a generic ROS node server.
	 */
	class NodeServer {
		unsigned int nRobots_;
		RobotConnectivity connectivityHandler_;
		ros::Subscriber connectivitySubscriber_;

	protected:
		unsigned int preferredRate_ = 30; // [Hz] Preferred update rate
		const unsigned int robotId_;
		ros::NodeHandle nodeHandle_;

	public:

		NodeServer(ros::NodeHandle nh, unsigned int robotId);

		~NodeServer() { }

		/**
		 * This method is called by the main node instantiator at
		 * each iteration of the main loop.
		 */
		void update(ros::Duration timeStep);

		/**
		 * Returns a string representing the concrete node server's name.
		 */
		virtual std::string getName() const;

		/**
		 * Return total number of robots in simulation.
		 */
		unsigned int getRobotCount() const;

		/**
		 * Return this robot's ID.
		 */
		unsigned int getRobotId() const;

		/**
		 * True if messages from peerId can reach this robot.
		 */
		bool canReceiveFromPeer(unsigned int peerId) const;


		/**
		 * Returns the preferred rate (in Hz) for update calls.
		 */
		unsigned int getPreferredRate() const { return preferredRate_; }

	protected:
		/**
		 * Update method to be implemented by child classes.
		 */
		virtual void update_(ros::Duration timeStep) = 0;


		/**
		 * Call the starting area service and return the response.
		 * @param timeout [s] Max timeout to wait for service to come online.
		 */
		dmce_msgs::GetStartingArea::Response fetchStartingArea_(const double& timeout = 15);


		/**
		 * Return the position of the ground station. Note that this requires a poll
		 * to the starting area service.
		 * @param timeout [s] Max timeout to wait for starting area service to come online.
		 */
		pos_t fetchGroundStationPosition(const double& timeout = 15);

		/**
		 * Query the ground-truth map service for the ground-truth map.
		 */
		GroundTruthMap fetchGroundTruthMap_(const double& timeout = 15);

		/**
		 * Fetch a required parameter from the ROS parameter server,
		 * throwing an error if it cannot be found.
		 */
		template<typename T>
		inline void getRequiredParam(std::string name, T& var) {
			std::stringstream callerName;
			callerName << robotId_ << ":" << this->getName();
			utils::getRequiredParam(name, var, callerName.str());
		}

		/**
		 * Create a subscriber for each robot, including this one.
		 */
		template<typename FP, typename T>
		std::vector<ros::Subscriber> subscribeForEachRobot(const std::string& topic, unsigned int queue_size, FP callback, T* obj, bool includeGroundStation = false) {
			return utils::subscribeForEachRobot(nodeHandle_, nRobots_, topic, queue_size, callback, obj, -1, includeGroundStation);
		}

		/**
		 * Create a subscriber for each other robot.
		 */
		template<typename FP, typename T>
		std::vector<ros::Subscriber> subscribeForEachOtherRobot(const std::string& topic, unsigned int queue_size, FP callback, T* obj, bool includeGroundStation = false) {
			return utils::subscribeForEachRobot(nodeHandle_, nRobots_, topic, queue_size, callback, obj, robotId_, includeGroundStation);
		}

		template<typename... Args>
		void logInfo(std::string methodName, std::string message, Args... args) const {
			ROS_INFO((messageHeader_(methodName) + message).c_str(), args...);
		}

		template<typename... Args>
		void logWarn(std::string methodName, std::string message, Args... args) const {
			ROS_WARN((messageHeader_(methodName) + message).c_str(), args...);
		}

		template<typename... Args>
		void logError(std::string methodName, std::string message, Args... args) const {
			ROS_ERROR((messageHeader_(methodName) + message).c_str(), args...);
		}


	private:
		/**
		 * Updates the stored robot connectivity.
		 */
		void connectivityCallback_(const RobotConnectivity::Message& msg);

		std::string messageHeader_(std::string methodName) const {
			std::stringstream ss;
			ss << "[" << robotId_ << "::" << this->getName();
			if (!methodName.empty()) {
				ss << "::" << methodName;
			}
			ss << "] ";
			return ss.str();
		}
	};
}
