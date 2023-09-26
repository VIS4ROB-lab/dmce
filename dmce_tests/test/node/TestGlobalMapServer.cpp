#include "dmce_test/common.hpp"

#include "std_msgs/String.h"

#include "dmce_nodes/GroundTruthMapServer.hpp"
#include "dmce_nodes/GlobalMapServer.hpp"

#include "dmce_msgs/RobotMapUpdate.h"

namespace dmce_test {

using namespace dmce;
class TestGlobalMapServer : public ::testing::Test {
protected:
	std::unique_ptr<GroundTruthMapServer> truthMapSrv_;
	std::unique_ptr<dmce::GlobalMapServer> mapServer;
	ros::Publisher updatePublisher;
	ros::Subscriber mapSubscriber;

	unsigned int mapCallbackCount = 0;

public:
	TestGlobalMapServer() {
		mapSubscriber =
			ros::NodeHandle().subscribe(
				"GlobalMap", 1,
				&TestGlobalMapServer::mapCallback,
				this
			);
		updatePublisher =
			ros::NodeHandle().advertise<dmce_msgs::RobotMapUpdate>(
				"/robot1/RobotMapUpdates", true
			);
	}

	void instantiateMap() {
		mapServer = std::make_unique<GlobalMapServer>(ros::NodeHandle(), 0, 0.05);
	}

	void initTruthMapServer() {
		truthMapSrv_= std::make_unique<GroundTruthMapServer>(ros::NodeHandle("/groundStation"), 1);
	}

	void mapCallback(const grid_map_msgs::GridMap& mapMsg) {
		mapCallbackCount += 1;
	}

	void publishUpdate() {
		dmce_msgs::RobotMapUpdate message;
		message.length = 4;
		message.values = {0, 0.1, 0.9, 1};
		message.x_positions = {0, 1, 2, -1};
		message.y_positions = {0, 1, 2, -1};

		updatePublisher.publish(message);
	}

	~TestGlobalMapServer() {

	}
};

TEST_F(TestGlobalMapServer, OnNoMapService_ThrowsError) {
	EXPECT_THROW(instantiateMap(), std::runtime_error);
}

TEST_F(TestGlobalMapServer, OnMissingParams_throwsError) {
	initTruthMapServer();
	Funcs::requireParams(
		{"/globalMap/resolution"},
		[this]{this->instantiateMap();}
	);
}

TEST_F(TestGlobalMapServer, OnCreation_CallbackIsInvoked) {
	initTruthMapServer();
	instantiateMap();
	Funcs::takeANap();
	EXPECT_GE(mapCallbackCount, 1);
}

TEST_F(TestGlobalMapServer, OnPublishUpdate_PublishesMap) {
	initTruthMapServer();
	instantiateMap();
	unsigned int start_count = mapCallbackCount;
	publishUpdate();
	Funcs::takeANap();
	EXPECT_GT(mapCallbackCount, start_count);
}

}

int main(int argc, char** argv){
	ros::init(argc, argv, "GlobalMapServerTestNode");
	testing::InitGoogleTest(&argc, argv);

	std::thread t([]{while(ros::ok()) ros::spin();});
	
	auto res = RUN_ALL_TESTS();
	
	ros::shutdown();
	return res;
}

