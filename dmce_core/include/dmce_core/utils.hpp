#pragma once

#include <random>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace dmce {
namespace utils {
	/**
	 * Fetch a required parameter from the ROS parameter server,
	 * throwing an error if it cannot be found.
	 */
	template<typename T>
	void getRequiredParam(std::string name, T& var, std::string callerName = "") {
		if (!ros::param::has(name)) {
			std::stringstream ss;
			if (!callerName.empty())
				ss << "[" << callerName << "] ";
			ss << "ROS parameter '" << name << "' not found!";
			throw std::runtime_error(ss.str());
		}
		ros::param::get(name, var);
	}

	inline unsigned int getRobotCount() {
		int robotCount;
		getRequiredParam("/sim/nRobots", robotCount);
		return robotCount;
	}


	/**
	 * Create a subscriber for each robot other than myID.
	 * Each subscriber will listen to the same topic within a different
	 * robot's namespace, i.e. /robotN/my_topic for N=1..nRobots
	 * If includeGroundStation==true, /groundStation/my_topic will also
	 * be subcribed to, unless myID==0 (which is the ground station's ID).
	 */
	template<typename FP, typename T>
	std::vector<ros::Subscriber> subscribeForEachRobot(
			ros::NodeHandle& nh,
			unsigned int nRobots,
			const std::string& topic,
			unsigned int queue_size,
			FP callback,
			T* obj,
			int myID = -1,
			bool includeGroundStation = false
	) {
		std::vector<ros::Subscriber> subscribers;
		for (unsigned int i = 0; i <= nRobots; i++) {
			if (i == myID)
				continue;

			std::stringstream ss;
			if (i == 0 && includeGroundStation)
				ss << "/groundStation/" << topic;
			else
				ss << "/robot" << i << "/" << topic;

			subscribers.push_back(nh.subscribe(ss.str(), queue_size, callback, obj));
		}
		return subscribers;
	}


	/**
	 * This class gives access to the random number generator.
	 */
	class RNG {
	public:
		using generator_t = std::mt19937;

		/**
		 * Initialise and seed the random number generator.
		 * If the seed is 0, a (non-deterministic) random device is used instead.
		 */
		static void init(unsigned int seed = 0);

		/**
		 * Return a reference of the random number generator.
		 * If not already initialised, it will be initialised with
		 * a non-deterministic seed.
		 */
		static generator_t& get();

	private:
		static bool isInitialised_;
		static generator_t gen_;
	};

	/**
	 * Return a random integer in range [0; size)
	 */
	inline size_t randomIndex(const size_t& size) {
		if (size == 0)
			throw std::invalid_argument("Size must be at least 1!");
		else if (size == 1)
			return 0;

		std::uniform_int_distribution<> distr(0, size-1);
		return distr(RNG::get());
	}


	/**
	 * Returns a default-initialised default disc-shaped marker used for RViz.
	 */
	inline visualization_msgs::Marker getDefaultMarker_(const double& diameter) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		
		// Set the namespace and id for this marker.
		// This serves to create a unique ID. Any marker sent with
		// the same namespace and id will overwrite the old one
		marker.ns = "robots";
		marker.id = 0;
		
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		
		// Set the pose of the marker. This is a full 6DOF pose
		// relative to the frame/time specified in the header
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = diameter;
		marker.scale.y = diameter;
		marker.scale.z = 0.01;
		
		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.2f;
		marker.color.g = 0.7f;
		marker.color.b = 0.2f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		return marker;
	}
} // namespace utils
} // namespace dmce
