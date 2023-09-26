#include "std_msgs/String.h"

#include "dmce_test/common.hpp"
#include <actionlib/client/simple_action_client.h>

#include "dmce_msgs/GetPlan.h"
#include "dmce_msgs/PathfindingAction.h"
#include "dmce_nodes/PathfindingServer.hpp"
#include "dmce_nodes/PlannerServer.hpp"
#include "dmce_nodes/GroundTruthMapServer.hpp"

namespace dmce_test {

class TestPathfindingServer : public ::testing::Test {
protected:
	using ActionClient = actionlib::SimpleActionClient<dmce_msgs::PathfindingAction>;

	ros::NodeHandle nodeHandle_;
	ActionClient actionClient_;
	dmce::PathfindingServer* path_srv_ = nullptr;
	dmce::RobotMap map_;
	dmce::pos_t robotPosition_;
	dmce::pos_t goalPosition_;
	ros::Publisher mapPublisher_;
	ros::Publisher posPublisher_;

	unsigned int doneCalls_ = 0;

	unsigned int feedbackCalls_ = 0;
	nav_msgs::Path latestFeedback_;

public:
	TestPathfindingServer()
		: nodeHandle_(ros::NodeHandle()),
		  map_({20,20}, 1, {0,0}, "map"),
		  robotPosition_{1,1}, goalPosition_({5,5}),
		  actionClient_("Pathfinding", false)
	{
		mapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("inflatedMap", 1, true);
		posPublisher_ =
			nodeHandle_.advertise<dmce_msgs::RobotPosition>("RobotPosition", 1, true);
	}

	void publishMapAndPosition() {
		mapPublisher_.publish(map_.getMessage());
		dmce_msgs::RobotPosition posMsg;
		posMsg.x_position = robotPosition_.x();
		posMsg.y_position = robotPosition_.y();
		posMsg.robotId = 0;
		posPublisher_.publish(posMsg);
	}

	void initServer() {
		path_srv_ = new dmce::PathfindingServer(nodeHandle_, 1);
	}

	void doneCallback_(const actionlib::SimpleClientGoalState& state,
			const dmce_msgs::PathfindingResultConstPtr& result) { doneCalls_++; }

	void feedbackCallback_(const dmce_msgs::PathfindingFeedbackConstPtr& feedback)
	{
		feedbackCalls_++;
		latestFeedback_ = feedback->path;
	}

	void sendGoalAndUpdate_() {
		dmce_msgs::PathfindingGoal goalMsg;
		goalMsg.robotId = 0;
		goalMsg.x_goal = goalPosition_.x();
		goalMsg.y_goal = goalPosition_.y();

		actionClient_.sendGoal(goalMsg,
			boost::bind(&TestPathfindingServer::doneCallback_, this, _1, _2),
			ActionClient::SimpleActiveCallback(),
			boost::bind(&TestPathfindingServer::feedbackCallback_, this, _1));

		Funcs::takeANap();
		path_srv_->update(ros::Duration(1));
		Funcs::takeANap();
	}

	~TestPathfindingServer() {
		if (path_srv_ != nullptr)
			delete path_srv_;
	}
};

TEST_F(TestPathfindingServer, RequiredParams) {
	Funcs::requireParams({
			"/robot/diameter",
			"/robot/speed",
			"/robot/navigationCutoff"
		},
		[&] { initServer(); }
	);
}

TEST_F(TestPathfindingServer, OnInstance_ServiceExists) {
	initServer();
	Funcs::takeANap();
	ASSERT_TRUE(actionClient_.waitForServer(ros::Duration(1)));
}

TEST_F(TestPathfindingServer, OnPublishMapAndPos_onSendGoal_receivesFeedback) {
	initServer();
	Funcs::takeANap();
	publishMapAndPosition();
	sendGoalAndUpdate_();

	ASSERT_GT(feedbackCalls_, 0);
	ASSERT_GE(latestFeedback_.poses.size(), 2);
	EXPECT_FLOAT_EQ(latestFeedback_.poses[0].pose.position.x, robotPosition_.x());
	EXPECT_FLOAT_EQ(latestFeedback_.poses[0].pose.position.y, robotPosition_.y());
	EXPECT_FLOAT_EQ(latestFeedback_.poses[1].pose.position.x, goalPosition_.x());
	EXPECT_FLOAT_EQ(latestFeedback_.poses[1].pose.position.y, goalPosition_.y());
}


TEST_F(TestPathfindingServer, OnCurrentPosIsGoal_update_finishesAction) {
	initServer();
	Funcs::takeANap();
	publishMapAndPosition();
	goalPosition_ = robotPosition_;
	sendGoalAndUpdate_();

	ASSERT_GT(doneCalls_, 0);
}

TEST_F(TestPathfindingServer, OnAbortGoal_finishesGoal) {
	initServer();
	Funcs::takeANap();
	publishMapAndPosition();
	sendGoalAndUpdate_();
	actionClient_.cancelGoal();
	Funcs::takeANap();

	ASSERT_GT(doneCalls_, 0);
}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "PathfindingServerTestNode");
	testing::InitGoogleTest(&argc, argv);

	std::thread t([]{while(ros::ok()) ros::spin();});

	auto res = RUN_ALL_TESTS();
	
	ros::shutdown();
	return res;
}

