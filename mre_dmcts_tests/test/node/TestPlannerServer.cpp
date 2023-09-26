#include "std_msgs/String.h"

#include "mre_dmcts_test/common.hpp"

#include "mre_dmcts_msgs/GetPlan.h"
#include "mre_dmcts_nodes/PlannerServer.hpp"
#include "mre_dmcts_nodes/GroundTruthMapServer.hpp"

namespace mre_dmcts_test {

class TestPlannerServer : public ::testing::Test {
protected:
	mre_dmcts::PlannerServer* planner_srv_ = nullptr;
	ros::NodeHandle nodeHandle_;
	ros::ServiceClient plannerServiceClient_;

public:
	TestPlannerServer()
		: nodeHandle_(ros::NodeHandle()),
		  plannerServiceClient_(nodeHandle_.serviceClient<mre_dmcts_msgs::GetPlan>(
					  "GlobalPlannerService"
		  ))
	{

	}

	void initServer() {
		planner_srv_ = new mre_dmcts::PlannerServer(ros::NodeHandle(), 1, 0.05);
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
	mre_dmcts::GroundTruthMapServer gtmapserver(ros::NodeHandle("groundStation"), 0);
	
	auto res = RUN_ALL_TESTS();
	
	ros::shutdown();
	return res;
}

