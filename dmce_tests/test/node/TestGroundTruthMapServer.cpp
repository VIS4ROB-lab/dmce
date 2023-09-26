#include "std_msgs/String.h"

#include "dmce_test/common.hpp"

#include "dmce_nodes/GroundTruthMapServer.hpp"

namespace dmce_test {

class TestGroundTruthMapServer : public ::testing::Test {
protected:
	dmce::GroundTruthMapServer* map_srv_ = nullptr;
	ros::NodeHandle nodeHandle_;
	ros::ServiceClient client_;

public:
	TestGroundTruthMapServer()
		: nodeHandle_(ros::NodeHandle()),
		  client_(nodeHandle_.serviceClient<dmce_msgs::GetMap>("GroundTruthMapService")) 
	{

	}

	void initMapServer() {
		map_srv_ = new dmce::GroundTruthMapServer(nodeHandle_, 0);
	}

	~TestGroundTruthMapServer() {
		if (map_srv_ != nullptr)
			delete map_srv_;
	}
};

TEST_F(TestGroundTruthMapServer, OnMissingResolution_throwsError) {
	StealParameter<double> res("/globalMap/resolution");
	EXPECT_THROW(initMapServer(), std::runtime_error);
}

TEST_F(TestGroundTruthMapServer, OnMissingImageName_throwsError) {
	StealParameter<std::string> imageFile("/globalMap/groundTruthImage");
	EXPECT_THROW(initMapServer(), std::runtime_error);
}

TEST_F(TestGroundTruthMapServer, OnInstance_ServiceExists) {
	initMapServer();
	Funcs::takeANap();
	EXPECT_TRUE(client_.exists());
	dmce_msgs::GetMap getMap;
	EXPECT_TRUE(client_.call(getMap));
}

TEST_F(TestGroundTruthMapServer, OnCall_ReturnsCorrectMap) {
	initMapServer();
	Funcs::takeANap();
	dmce_msgs::GetMap getMap;
	EXPECT_TRUE(client_.call(getMap));
	dmce::GroundTruthMap map(getMap.response.map);

	double res;
	ros::param::get("/globalMap/resolution", res);
	Funcs::checkTestMap(map, res);
}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "GroundTruthMapServerTestNode");
	testing::InitGoogleTest(&argc, argv);

	std::thread t([]{while(ros::ok()) ros::spin();});
	
	auto res = RUN_ALL_TESTS();
	
	ros::shutdown();
	return res;
}

