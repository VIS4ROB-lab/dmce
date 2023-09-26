#pragma once

#include <ros/ros.h>
#include <sys/stat.h>
#include "grid_map_msgs/GridMap.h"

#include "dmce_core/TypeDefs.hpp"
#include "dmce_core/RobotMap.hpp"
#include "dmce_msgs/RobotPosition.h"
#include "dmce_msgs/NavigationFailureSignal.h"

#include "dmce_nodes/NodeServer.hpp"

#include <system_error>
#include <fstream>

namespace dmce {
	class LoggingServer : public NodeServer {
		ros::Subscriber globalMapSubscriber_;
		std::vector<ros::Subscriber> robotPosSubscribers_;
		std::vector<ros::Subscriber> robotMapSubscribers_;
		std::vector<ros::Subscriber> navFailureSubscribers_;

		std::vector<bool> initPosRecorded_;
		std::vector<pos_t> lastRobotPositions_;
		std::vector<double> robotDistances_;
		std::vector<double> robotEntropies_;
		double totalDistanceCovered_ = 0.0;
		double currentGlobalEntropy_ = 1.0;
		bool use_time_time_;
		double time_multiplier_;
		bool scale_with_robots_;
		double explorationFactor_;
		bool restrictComms_;
		int minRollouts_;
		bool reuseBranches_;
		bool useLocalReward_;
		unsigned int navigationFailureCount_ = 0;

		std::mutex dataMutex_;

		double logFrequency_;
		std::string logFileBaseName_ = "performance";
		std::string logFileType_ = ".csv";
		std::string logFileDir_;
		std::string fullLogFile_;
		std::string plannerType_;
		std::string scenarioName_;
		bool enabled_;

		double minPlanValue_;
		double iterationDiscountFactor_;
		bool useActionCaching_;

		double totalTime_ = 0;
		double timeSinceLastWrite_;

		/**
		 * Write the given string to the logfile.
		 */
		void writeToFile_(
			std::string toWrite,
			std::ios_base::openmode mode = std::ios_base::out
		);

		/**
		 * Create the log directory and initialise logFileDir_ to
		 * its correct value.
		 */
		void initLogDir_();

		/**
		 * Find a suitable file name and write the CSV header to it.
		 */
		void initLogFile_();

		/**
		 * Write meta-data and data column headers to the logfile.
		 */
		void writeFileHeader_();

		/**
		 * Returns true if the given file exists on disk.
		 */
		inline static bool fileExists_(const std::string& name);

	protected:
		void update_(ros::Duration timeStep) override;

	public:
		LoggingServer(ros::NodeHandle nh, unsigned int robotId);

		void robotMapCallback(const grid_map_msgs::GridMap& mapMsg);

		void globalMapCallback(const grid_map_msgs::GridMap& mapMsg);

		void positionCallback_(const dmce_msgs::RobotPosition& msg);

		void navFailureCallback(const dmce_msgs::NavigationFailureSignal& msg);

		virtual std::string getName() const override {
			return "LoggingServer";
		}
	};
}
