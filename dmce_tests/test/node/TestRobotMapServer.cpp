#include "dmce_test/common.hpp"

#include "std_msgs/String.h"

#include "dmce_nodes/GroundTruthMapServer.hpp"
#include "dmce_nodes/RobotMapServer.hpp"

#include "dmce_msgs/RobotMapUpdate.h"

namespace dmce_test {

using namespace dmce;

class TestRobotMapServer : public ::testing::Test {
protected:
	std::unique_ptr<GroundTruthMapServer> truthMapSrv_;
	std::unique_ptr<RobotMapServer> mapServer;
	ros::Publisher updatePublisher;
	ros::Subscriber mapSubscriber;
	ros::Subscriber inflatedMapSubscriber;

	unsigned int mapCallbackCount = 0;
	unsigned int inflatedMapCallbackCount = 0;

public:
	TestRobotMapServer() {
		mapSubscriber =
			ros::NodeHandle().subscribe(
				"map", 1,
				&TestRobotMapServer::mapCallback,
				this
			);
		inflatedMapSubscriber =
			ros::NodeHandle().subscribe(
				"inflatedMap", 1,
				&TestRobotMapServer::inflatedMapCallback,
				this
			);
		updatePublisher =
			ros::NodeHandle().advertise<dmce_msgs::RobotMapUpdate>(
				"/robot1/RobotMapUpdates", true
			);
	}

	void initRobotMapServer() {
		mapServer = std::make_unique<RobotMapServer>(ros::NodeHandle(), 0, 0.1);
	}

	void initTruthMapServer() {
		truthMapSrv_= std::make_unique<GroundTruthMapServer>(ros::NodeHandle("/groundStation"), 1);
	}

	void mapCallback(const grid_map_msgs::GridMap& mapMsg) {
		mapCallbackCount += 1;
	}

	void inflatedMapCallback(const grid_map_msgs::GridMap& mapMsg) {
		inflatedMapCallbackCount += 1;
	}

	void publishUpdate() {
		dmce_msgs::RobotMapUpdate message;
		message.length = 4;
		message.values = {0, 0.1, 0.9, 1};
		message.x_positions = {0, 1, 2, -1};
		message.y_positions = {0, 1, 2, -1};

		updatePublisher.publish(message);
	}

	~TestRobotMapServer() {

	}
};

TEST_F(TestRobotMapServer, OnNoMapService_ThrowsError) {
	EXPECT_THROW(initRobotMapServer(), std::runtime_error);
}

TEST_F(TestRobotMapServer, OnMissingParams_throwsError) {
	initTruthMapServer();
	Funcs::requireParams(
		{
			"/robot/diameter",
			"/robot/map/resolution",
		},
		[this]{this->initRobotMapServer();}
	);
}

TEST_F(TestRobotMapServer, OnUpdate_PublishesMap) {
	initTruthMapServer();
	initRobotMapServer();
	unsigned int start_count = mapCallbackCount;
	unsigned int start_count2 = inflatedMapCallbackCount;
	mapServer->update(ros::Duration(1));
	Funcs::takeANap();
	EXPECT_GT(mapCallbackCount, start_count);
	EXPECT_GT(inflatedMapCallbackCount, start_count2);
}

}

int main(int argc, char** argv){
	ros::init(argc, argv, "RobotMapServerTestNode");
	testing::InitGoogleTest(&argc, argv);

	std::thread t([]{while(ros::ok()) ros::spin();});
	
	auto res = RUN_ALL_TESTS();
	
	ros::shutdown();
	return res;
}

