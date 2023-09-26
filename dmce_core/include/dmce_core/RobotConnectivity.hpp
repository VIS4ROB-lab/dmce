#pragma once

#include "std_msgs/MultiArrayLayout.h"
#include "dmce_msgs/RobotConnectivity.h"

namespace dmce {
	/**
	 * This class is used to read and write RobotConnectivity messages,
	 * which describes which robots can communicate which each other.
	 */
	class RobotConnectivity {
	public:
		using Message = dmce_msgs::RobotConnectivity;

		/**
		 * Default initialisation: all-to-all connectivity.
		 */
		RobotConnectivity(unsigned int nRobots);
		RobotConnectivity(const Message& message);
		RobotConnectivity(const RobotConnectivity& other);
		RobotConnectivity& operator=(const RobotConnectivity& other);

		/**
		 * Copy connectivity information from a ROS message.
		 */
		void loadMessage(const Message& message);

		/**
		 * Return connectivity as a ROS message.
		 */
		Message getMessage() const;

		/**
		 * True if transmitter fromId can send to toId, false otherwise.
		 * Note that connections are directional:
		 * isConnected(a, b) does not imply isConnected(b, a)
		 */
		bool isConnected(unsigned int fromId, unsigned int toId) const;

		/*
		 * True if there is a two-way connection between the peers.
		 */
		bool bothConnected(unsigned int fromId, unsigned int toId) const;

		/**
		 * Disable the drectional connection fromId->toId.
		 */
		void disconnect(unsigned int fromId, unsigned int toId);

		/**
		 * Disable both directions of the fromId<->toId connection.
		 */
		void disconnectBoth(unsigned int fromId, unsigned int toId);

		/**
		 * Enable the drectional connection fromId->toId.
		 */
		void connect(unsigned int fromId, unsigned int toId);

		/**
		 * Enable both directions of the fromId<->toId connection.
		 */
		void connectBoth(unsigned int fromId, unsigned int toId);

		/**
		 * Returns number of mobile robots in the modelled network (ground station not counted).
		 */
		unsigned int getNRobots() const;

		/**
		 * Returns number of peers in the modelled network (including ground station).
		 */
		unsigned int getNTransmitters() const;

	
	private:
		std_msgs::MultiArrayLayout layout_;
		using matrix_t = std::vector<uint8_t>;
		matrix_t connectivityMatrix_;
		unsigned int nTransmitters_;

		unsigned int getIdx_(unsigned int fromId, unsigned int toId) const;
	};
}
