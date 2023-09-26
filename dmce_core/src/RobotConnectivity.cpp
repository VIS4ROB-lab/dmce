#include "dmce_core/RobotConnectivity.hpp"

namespace dmce {
	RobotConnectivity::RobotConnectivity(unsigned int nRobots) {
		nTransmitters_ = nRobots + 1;
		std_msgs::MultiArrayDimension senderDim;
		senderDim.label = "senderId";
		senderDim.size = nTransmitters_;
		senderDim.stride = nTransmitters_*nTransmitters_;

		std_msgs::MultiArrayDimension receiverDim;
		receiverDim.label = "receiverId";
		receiverDim.size = nTransmitters_;
		receiverDim.stride = nTransmitters_;
		
		layout_.dim = {senderDim, receiverDim};
		layout_.data_offset = 0;
		connectivityMatrix_ = matrix_t(nTransmitters_*nTransmitters_, true);
	}

	RobotConnectivity::RobotConnectivity(const Message& message) {
		loadMessage(message);
	}

	RobotConnectivity::RobotConnectivity(const RobotConnectivity& other)
		: RobotConnectivity(other.getMessage())
	{ }

	RobotConnectivity& RobotConnectivity::operator=(const RobotConnectivity& other) {
		Message message = other.getMessage();
		loadMessage(message);
		return *this;
	}

	void RobotConnectivity::loadMessage(const Message& message) {
		nTransmitters_ = message.layout.dim[0].size;
		layout_ = message.layout;
		connectivityMatrix_ = message.data;
	}

	RobotConnectivity::Message RobotConnectivity::getMessage() const {
		dmce_msgs::RobotConnectivity message;
		message.layout = layout_;
		message.data = connectivityMatrix_;
		return message;
	}

	bool RobotConnectivity::isConnected(unsigned int fromId, unsigned int toId) const {
		return connectivityMatrix_[getIdx_(fromId, toId)];
	}

	bool RobotConnectivity::bothConnected(unsigned int fromId, unsigned int toId) const {
		return isConnected(fromId, toId) && isConnected(toId, fromId);
	}

	void RobotConnectivity::disconnect(unsigned int fromId, unsigned int toId) {
		connectivityMatrix_[getIdx_(fromId, toId)] = false;
	}

	void RobotConnectivity::disconnectBoth(unsigned int fromId, unsigned int toId) {
		disconnect(fromId, toId);
		disconnect(toId, fromId);
	}

	void RobotConnectivity::connect(unsigned int fromId, unsigned int toId) {
		connectivityMatrix_[getIdx_(fromId, toId)] = true;
	}

	void RobotConnectivity::connectBoth(unsigned int fromId, unsigned int toId) {
		connect(fromId, toId);
		connect(toId, fromId);
	}

	unsigned int RobotConnectivity::getNRobots() const {
		return getNTransmitters()-1;
	}

	unsigned int RobotConnectivity::getNTransmitters() const {
		return nTransmitters_;
	}

	unsigned int RobotConnectivity::getIdx_(unsigned int fromId, unsigned int toId) const {
		if (fromId >= nTransmitters_ || toId >= nTransmitters_) {
			throw std::out_of_range("Index out of range!");
		}
		return fromId*nTransmitters_ + toId;
	}
}
