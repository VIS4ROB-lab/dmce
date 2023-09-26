#include "dmce_nodes/LoggingServer.hpp"

namespace dmce {
	LoggingServer::LoggingServer(ros::NodeHandle nh, unsigned int robotId)
		: NodeServer(nh, robotId),
		  initPosRecorded_(getRobotCount()+1, false),
		  lastRobotPositions_(getRobotCount()+1, {0,0}),
		  robotDistances_(getRobotCount()+1, 0),
		  robotEntropies_(getRobotCount()+1, 1)
	{
		getRequiredParam("/logger/enabled", enabled_);

		if (!enabled_)
			return;

		getRequiredParam("/logger/logDirectory", logFileDir_);
		getRequiredParam("/logger/logFrequency", logFrequency_);
		getRequiredParam("~plannerType", plannerType_);
		getRequiredParam("/mcts/reuseBranches", reuseBranches_);
		getRequiredParam("/mcts/useLocalReward", useLocalReward_);
		getRequiredParam("/mcts/minRollouts", minRollouts_);
		getRequiredParam("/mcts/minPlanValue", minPlanValue_);
		getRequiredParam("/mcts/iterationDiscountFactor", iterationDiscountFactor_);
		getRequiredParam("/mcts/useActionCaching", useActionCaching_);
		getRequiredParam("/mcts/explorationFactor", explorationFactor_);
		getRequiredParam("/use_sim_time", use_time_time_);
		getRequiredParam("/sim_time/time_multiplier", time_multiplier_);
		getRequiredParam("/sim_time/scale_with_robots", scale_with_robots_);
		getRequiredParam("/globalMap/scenarioName", scenarioName_);
		getRequiredParam("/robot/restrictComms", restrictComms_);

		if (!use_time_time_)
			time_multiplier_ = 1.0;
		else if (scale_with_robots_)
			time_multiplier_ = 1. / getRobotCount();

		initLogDir_();
		initLogFile_();

		globalMapSubscriber_ = nodeHandle_.subscribe(
			"/groundStation/GlobalMap", 1,
			&LoggingServer::globalMapCallback,
			this
		);

		robotPosSubscribers_ = subscribeForEachRobot(
			"RobotPosition", 1,
			&LoggingServer::positionCallback_, this
		);

		robotMapSubscribers_ = subscribeForEachRobot(
			"map", 1,
			&LoggingServer::robotMapCallback, this,
			true
		);

		navFailureSubscribers_ = subscribeForEachRobot(
			"navigationFailure", 10,
			&LoggingServer::navFailureCallback, this,
			true
		);
	}

	void LoggingServer::initLogDir_() {
		logFileDir_ = logFileDir_ + std::to_string(getRobotCount()) + "robots/";
		mkdir(logFileDir_.c_str(), ACCESSPERMS);
		logFileDir_ += plannerType_ + "/";
		mkdir(logFileDir_.c_str(), ACCESSPERMS);
	}

	void LoggingServer::initLogFile_() {
		logFileBaseName_ = logFileDir_ + logFileBaseName_;

		std::string logFilePre = logFileBaseName_;
		std::string logFilePost = logFileType_;
		auto makeFileName = [&logFilePre, &logFilePost](unsigned int i) -> std::string {
			std::stringstream filename;
			filename << logFilePre << std::setfill('0') << std::setw(4) << i;
			filename << logFilePost;
			return filename.str();
		};

		unsigned int i = 0;
		do {
			++i;
		} while (fileExists_(makeFileName(i)));
		fullLogFile_ = makeFileName(i);

		writeFileHeader_();
	}

	void LoggingServer::writeFileHeader_() {
		std::stringstream ss;
		ss << "Package version:," << dmce_nodes_VERSION << std::endl;
		ss << "Scenario:," << scenarioName_ << std::endl;
		ss << "Connectivity:," << (restrictComms_ ? "LoS only" : "global") << std::endl;
		ss << "Planner type:," << plannerType_ << std::endl;
		ss << "Robot count:," << getRobotCount() << std::endl;
		ss << "Branch reuse:," << (reuseBranches_ ? "enabled" : "disabled") << std::endl;
		ss << "Local reward:," << (useLocalReward_ ? "enabled" : "disabled") << std::endl;
		ss << "Min. tree age:," << minRollouts_ << std::endl;
		ss << "Iteration discount:," << iterationDiscountFactor_ << std::endl;
		ss << "Exploration factor:," << explorationFactor_ << std::endl;
		ss << "Min plan value:," << minPlanValue_ << std::endl;
		ss << "Action caching:," << (useActionCaching_ ? "enabled" : "disabled") << std::endl;
		ss << "ROS time multiplier:," << time_multiplier_ << std::endl;
		ss << std::endl;
		ss << "Time [s], Map uncertainty (global) [-], "
		   << "Map uncertainty (GS) [-], Total distance [m]";
		for (unsigned int i = 1; i <= getRobotCount(); i++) {
			ss << ", Distance (robot " << i << ") [m]";
			ss << ", Map uncertainty (robot " << i << ") [-]";
		}
		ss << ", Navigation failures";
		ss << std::endl;
		writeToFile_(ss.str());
	}

	void LoggingServer::globalMapCallback(const grid_map_msgs::GridMap& mapMsg) {
		lockGuard_t guard(dataMutex_);
		RobotMap map(mapMsg);
		currentGlobalEntropy_ = map.getRelativeEntropy();
	}

	void LoggingServer::robotMapCallback(const grid_map_msgs::GridMap& mapMsg) {
		lockGuard_t guard(dataMutex_);
		RobotMap map(mapMsg);
		unsigned int robotId = mapMsg.info.header.stamp.toSec();
		if (robotId > robotEntropies_.size())
			return;
		robotEntropies_[robotId] = map.getRelativeEntropy();
	}

	void LoggingServer::positionCallback_(const dmce_msgs::RobotPosition& msg) {
		lockGuard_t guard(dataMutex_);
		pos_t curPos{msg.x_position, msg.y_position};
		unsigned int id = msg.robotId;
		if (initPosRecorded_[id]) {
			double delta = (lastRobotPositions_[id] - curPos).norm();
			totalDistanceCovered_ += delta;
			robotDistances_[id] += delta;
		}
		lastRobotPositions_[id] = curPos;
		initPosRecorded_[id] = true;
	}

	void LoggingServer::update_(ros::Duration timeStep) {
		if (!enabled_)
			return;
		
		timeSinceLastWrite_ += timeStep.toSec();
		totalTime_ += timeStep.toSec();

		if (timeSinceLastWrite_ < 1/logFrequency_)
			return;

		lockGuard_t guard(dataMutex_);
		std::stringstream ss;
		ss << totalTime_ << ", ";
		ss << currentGlobalEntropy_ << ", ";
		ss << robotEntropies_[0] << ", ";
		ss << totalDistanceCovered_;
		for (unsigned int i = 1; i <= getRobotCount(); i++) {
			ss << ", " << robotDistances_[i];
			ss << ", " << robotEntropies_[i];
		}
		ss << ", " << navigationFailureCount_;
		ss << std::endl;
		writeToFile_(ss.str(), std::ios_base::app);
		timeSinceLastWrite_ = 0;
	}

	void LoggingServer::writeToFile_(std::string toWrite, std::ios_base::openmode mode) {
		std::ofstream of(fullLogFile_, mode);
		if (!of.is_open()) {
			logError("writeToFile_", "Could not open log file: '%s'", fullLogFile_.c_str());
			return;
		}
		of << toWrite;
	}

	inline bool LoggingServer::fileExists_(const std::string& name) {
		struct stat buffer;
		return (stat (name.c_str(), &buffer) == 0);
	}

	void LoggingServer::navFailureCallback(const dmce_msgs::NavigationFailureSignal& msg) {
		navigationFailureCount_++;
	}
}
