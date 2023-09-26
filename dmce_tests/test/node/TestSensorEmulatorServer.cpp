#include "std_msgs/String.h"

#include "dmce_test/common.hpp"

#include "dmce_nodes/GroundTruthMapServer.hpp"
#include "dmce_nodes/SensorEmulatorServer.hpp"

#include "dmce_msgs/RobotPosition.h"

namespace dmce_test {

struct PositionParameter {
	double x;
	double y;
};

class TestSensorEmulatorServer : public ::testing::Test, public testing::WithParamInterface<PositionParameter> {
protected:
	dmce::GroundTruthMapServer* map_srv_ = nullptr;
	dmce::SensorEmulatorServer* se_srv_ = nullptr;
	ros::NodeHandle nodeHandle_;

	ros::Publisher positionPublisher_;
	ros::Subscriber updateSubscriber_;

	unsigned int subscriberCallCount_ = 0;

	dmce_msgs::RobotMapUpdate latestUpdate_;

public:
	TestSensorEmulatorServer()
		: nodeHandle_(ros::NodeHandle())
	{
		updateSubscriber_ =
			nodeHandle_.subscribe(
				"RobotMapUpdates", 1,
				&TestSensorEmulatorServer::updateCallback,
				this
			);
		positionPublisher_ =
			nodeHandle_.advertise<dmce_msgs::RobotPosition>(
				"RobotPosition", 1, true
			);
	}

	void initMapServer() {
		map_srv_ = new dmce::GroundTruthMapServer(ros::NodeHandle("/groundStation"), 1);
	}

	void initSEServer(const double& timeout = 0.05) {
		se_srv_ = new dmce::SensorEmulatorServer(nodeHandle_, 1, timeout);
	}

	void publishPosition(double x, double y) {
		dmce_msgs::RobotPosition message;
		message.x_position = x;
		message.y_position = y;
		positionPublisher_.publish(message);
	}

	void updateCallback(const dmce_msgs::RobotMapUpdate& msg) {
		subscriberCallCount_++;
		latestUpdate_ = msg;
	}

	~TestSensorEmulatorServer() {
		if (map_srv_ != nullptr)
			delete map_srv_;
		if (se_srv_ != nullptr)
			delete se_srv_;
	}
};

TEST_F(TestSensorEmulatorServer, OnNoMapService_ThrowsError) {
	EXPECT_THROW(initSEServer(), std::runtime_error);
}

TEST_F(TestSensorEmulatorServer, OnMapServiceAvail_noError) {
	initMapServer();
	initSEServer();
}

TEST_F(TestSensorEmulatorServer, OnNoRangeParam_ThrowsError) {
	initMapServer();
	Funcs::requireParams({"/robot/sensorRange", "/robot/lidarRayCount"}, [&]{ initSEServer(); });
}

TEST_F(TestSensorEmulatorServer, OnPublishPosition_PublishesMapUpdate) {
	initMapServer();
	initSEServer();
	EXPECT_EQ(subscriberCallCount_, 0);
	publishPosition(0, 0);
	Funcs::takeANap();
	EXPECT_GT(subscriberCallCount_, 0);
}

TEST_P(TestSensorEmulatorServer, OnGetMapUpdate_ValuesAreCorrect) {
	auto robotPos = GetParam();
	Eigen::Vector2d _robotPos(robotPos.x, robotPos.y);

	double range;
	ros::param::get("/robot/sensorRange", range);
	initMapServer();
	initSEServer();

	std::string imageFile;
	ros::param::get("/globalMap/groundTruthImage", imageFile);
	double resolution;
	ros::param::get("/globalMap/resolution", resolution);
	dmce::GroundTruthMap groundTruth(imageFile, resolution, "map");

	publishPosition(robotPos.x, robotPos.y);
	Funcs::takeANap();

	for (uint i = 0; i < latestUpdate_.length; i++) {
		grid_map::Position pos{
			latestUpdate_.x_positions[i], latestUpdate_.y_positions[i]
		};
		EXPECT_LT((_robotPos - pos).norm(), range);
		EXPECT_FLOAT_EQ(latestUpdate_.values[i], groundTruth.getOccupancy(pos));
	}
}

INSTANTIATE_TEST_SUITE_P(Parmetric, TestSensorEmulatorServer,
	testing::Values(
		PositionParameter{0, 0},
		PositionParameter{1, 1},
		PositionParameter{3.7345, -2.34}
	)
);

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "SensorEmulatorServerTestNode");
	testing::InitGoogleTest(&argc, argv);

	std::thread t([]{while(ros::ok()) ros::spin();});
	
	auto res = RUN_ALL_TESTS();
	
	ros::shutdown();
	return res;
}

