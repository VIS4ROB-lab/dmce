#include "std_msgs/String.h"

#include "mre_dmcts_test/common.hpp"

#include "mre_dmcts_nodes/GroundTruthMapServer.hpp"

namespace mre_dmcts_test {

class TestGroundTruthMapServer : public ::testing::Test {
protected:
	mre_dmcts::GroundTruthMapServer* map_srv_ = nullptr;
	ros::NodeHandle nodeHandle_;
	ros::ServiceClient client_;

public:
	TestGroundTruthMapServer()
		: nodeHandle_(ros::NodeHandle()),
		  client_(nodeHandle_.serviceClient<mre_dmcts_msgs::GetMap>("GroundTruthMapService")) 
	{

	}

	void initMapServer() {
		map_srv_ = new mre_dmcts::GroundTruthMapServer(nodeHandle_, 0);
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
	mre_dmcts_msgs::GetMap getMap;
	EXPECT_TRUE(client_.call(getMap));
}

TEST_F(TestGroundTruthMapServer, OnCall_ReturnsCorrectMap) {
	initMapServer();
	Funcs::takeANap();
	mre_dmcts_msgs::GetMap getMap;
	EXPECT_TRUE(client_.call(getMap));
	mre_dmcts::GroundTruthMap map(getMap.response.map);

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

