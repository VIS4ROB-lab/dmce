#include "std_msgs/String.h"

#include "dmce_test/common.hpp"

#include "dmce_msgs/GetPlan.h"
#include "dmce_nodes/PlannerServer.hpp"
#include "dmce_nodes/GroundTruthMapServer.hpp"

namespace dmce_test {

class TestPlannerServer : public ::testing::Test {
protected:
	dmce::PlannerServer* planner_srv_ = nullptr;
	ros::NodeHandle nodeHandle_;
	ros::ServiceClient plannerServiceClient_;

public:
	TestPlannerServer()
		: nodeHandle_(ros::NodeHandle()),
		  plannerServiceClient_(nodeHandle_.serviceClient<dmce_msgs::GetPlan>(
					  "GlobalPlannerService"
		  ))
	{

	}

	void initServer() {
		planner_srv_ = new dmce::PlannerServer(ros::NodeHandle(), 1, 0.05);
	}

	~TestPlannerServer() {
		if (planner_srv_ != nullptr)
			delete planner_srv_;
	}
};

TEST_F(TestPlannerServer, OnInstance_ServiceExists) {
	initServer();
	Funcs::takeANap();
	EXPECT_TRUE(plannerServiceClient_.exists());
}

TEST_F(TestPlannerServer, RequiredParams) {
	Funcs::requireParams({"/robot/diameter", "~plannerType"}, [&] { initServer(); });
}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "PlannerServerTestNode");
	testing::InitGoogleTest(&argc, argv);

	std::thread t([]{while(ros::ok()) ros::spin();});

	// Provides starting area service required by PlannerServer
	dmce::GroundTruthMapServer gtmapserver(ros::NodeHandle("groundStation"), 0);
	
	auto res = RUN_ALL_TESTS();
	
	ros::shutdown();
	return res;
}

