#include <chrono>
#include <thread>

#include <ros/ros.h>
#include "rosgraph_msgs/Clock.h"

using my_clock_t = std::chrono::steady_clock;
using time_point_t = std::chrono::time_point<my_clock_t>;
using duration_t = std::chrono::duration<double>;

template<typename T>
void getRequiredParam(std::string name, T& var) {
	if (!ros::param::has(name)) {
		std::stringstream ss;
		ss << "[clockServer] ROS parameter '" << name << "' not found!";
		throw std::runtime_error(ss.str());
	}
	ros::param::get(name, var);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "clockServer");
    ros::NodeHandle nodeHandle;
    auto nodeName = ros::this_node::getName();
	
    ros::Publisher clockPublisher = nodeHandle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    double time_multiplier;
    getRequiredParam("/sim_time/time_multiplier", time_multiplier);
    int topic_rate;
    getRequiredParam("/sim_time/topic_rate", topic_rate);
    bool scale_with_robots;
    getRequiredParam("/sim_time/scale_with_robots", scale_with_robots);
    if (scale_with_robots) {
        int nRobots;
        getRequiredParam("/sim/nRobots", nRobots);
        time_multiplier = 1. / nRobots;
    }

    const time_point_t start = my_clock_t::now();

    while (ros::ok()) {
        time_point_t curTime = my_clock_t::now();
        double secondsSinceStart = std::chrono::duration<double>(curTime - start).count();

        rosgraph_msgs::Clock msg;
        msg.clock = ros::Time(secondsSinceStart * time_multiplier);
        clockPublisher.publish(msg);

        std::this_thread::sleep_for(duration_t(1./topic_rate));
    }

    printf("[%s] shut down.\n", nodeName.c_str());

    return 0;
}

