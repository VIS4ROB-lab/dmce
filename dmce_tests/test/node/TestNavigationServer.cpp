#include "std_msgs/String.h"

#include "dmce_test/common.hpp"

#include "dmce_nodes/NavigationServer.hpp"
#include "dmce_nodes/GroundTruthMapServer.hpp"
#include "dmce_nodes/PlannerServer.hpp"

#include "dmce_msgs/RobotPosition.h"

namespace dmce_test {

struct PositionParameter {
	double x;
	double y;
};

using namespace dmce;

class TestNavigationServer : public ::testing::Test {
protected:
	PlannerServer* plan_srv_ = nullptr;
	GroundTruthMapServer* map_srv_ = nullptr;
	NavigationServer* nav_srv_ = nullptr;

	ros::NodeHandle nodeHandle_;

	ros::Subscriber positionSubscriber_;

	unsigned int subscriberCallCount_ = 0;

public:
	TestNavigationServer()
		: nodeHandle_(ros::NodeHandle())
	{
		positionSubscriber_ =
			nodeHandle_.subscribe(
				"RobotPosition", 1,
				&TestNavigationServer::positionCallback_,
				this
			);
	}

	void initMapServer() {
		map_srv_ = new GroundTruthMapServer(ros::NodeHandle("/groundStation"), 1);
	}

	void initPlannerServer() {
		plan_srv_ = new PlannerServer(nodeHandle_, 1);
	}

	void initNavServer() {
		nav_srv_ = new NavigationServer(nodeHandle_, 1, 0.05);
	}

	void positionCallback_(const dmce_msgs::RobotPosition& msg) {
		subscriberCallCount_++;
	}

	~TestNavigationServer() {
		if (plan_srv_ != nullptr)
			delete plan_srv_;
		if (map_srv_ != nullptr)
			delete map_srv_;
		if (nav_srv_ != nullptr)
			delete nav_srv_;
	}
};

TEST_F(TestNavigationServer, OnNoMapService_ThrowsError) {
	EXPECT_THROW(initNavServer(), std::runtime_error);
}

TEST_F(TestNavigationServer, OnNoPlannerService_ThrowsError) {
	initMapServer();
	EXPECT_THROW(initNavServer(), std::runtime_error);
}

TEST_F(TestNavigationServer, RequiresParameters) {
	initMapServer();
	initPlannerServer();
	Funcs::requireParams({
		"/robot/speed",
		"/robot/navigationCutoff",
		"/robot/diameter",
		"/robot/randomiseInitialPosition",
		"/robot/maxPathfindingFailures",
		"/robot/maxPlanAge"
		}, [&]{ initNavServer(); }
	);
}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "NavigationServerTestNode");
	testing::InitGoogleTest(&argc, argv);

	std::thread t([]{while(ros::ok()) ros::spin();});
	
	auto res = RUN_ALL_TESTS();
	
	ros::shutdown();
	return res;
}

